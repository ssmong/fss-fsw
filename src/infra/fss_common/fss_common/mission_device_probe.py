"""Identify in-use Jetson hardware from kernel metadata without opening any device."""

import ipaddress
import os
from pathlib import Path
import time


_HARDWARE_DRIVERS = ("arm_driver", "arm_driver_node", "vicon_bridge_node", "valve_driver_node",
                     "xsens_mti_node", "xsens_mti_driver")


def _restricted_driver(process):
    try:
        argv = (process / "cmdline").read_bytes().split(b"\0", 3)
        program = Path(os.fsdecode(argv[0])).name
        if program.startswith("python") and len(argv) > 1:
            program = Path(os.fsdecode(argv[1])).name
        if program.removesuffix(".py") in _HARDWARE_DRIVERS:
            return program
    except OSError:
        pass
    try:
        comm = (process / "comm").read_text().strip()
    except OSError:
        return None
    # Linux comm is capped at 15 bytes; Python ROS entry points also set it.
    return next((name for name in _HARDWARE_DRIVERS if name[:15] == comm), None)


def _network_resources(proc, vicon_port, plc_ip, plc_port):
    sockets = {}
    for protocol in ("udp", "udp6", "tcp", "tcp6"):
        path = proc / "net" / protocol
        try:
            lines = path.read_text().splitlines()[1:]
        except FileNotFoundError:
            if protocol.endswith("6"):
                continue  # IPv6 can be disabled.
            raise
        for line in lines:
            fields = line.split()
            local, remote, state, inode = fields[1], fields[2], fields[3], fields[9]
            if inode == "0":
                continue  # TIME_WAIT and other entries without an open socket.
            if protocol.startswith("udp"):
                if int(local.split(":")[1], 16) == vicon_port:
                    sockets[inode] = f"Vicon UDP :{vicon_port}"
            elif state in ("01", "02", "03", "08"):
                address, port = remote.split(":")
                if int(port, 16) != plc_port:
                    continue
                # Linux lists each 32-bit word in host byte order (Jetson is little-endian).
                octets = b"".join(bytes.fromhex(address[i:i + 8])[::-1] for i in range(0, len(address), 8))
                peer = ipaddress.ip_address(octets)
                if isinstance(peer, ipaddress.IPv6Address) and peer.ipv4_mapped:
                    peer = peer.ipv4_mapped
                if peer == plc_ip:
                    sockets[inode] = f"PLC TCP {plc_ip}:{plc_port}"
    return sockets


def device_resource_owner(proc=Path("/proc"), *, dev=Path("/dev"), vicon_port=9870,
                          plc_ip="192.168.1.2", plc_port=502, arm_port=None):
    """Return the occupied resource and PID, or None when the visible devices are free."""
    proc, dev = Path(proc), Path(dev)
    try:
        deadline = time.monotonic() + 2.0
        sockets = _network_resources(proc, vicon_port, ipaddress.ip_address(plc_ip), plc_port)
        serial = {}
        # Resolve stable hardware identities, since USB tty numbers can change at boot.
        for pattern, name in (("usb-Xsens_*", "Xsens"), ("usb-FTDI_*FTAKRQ9X*", "팔 FTDI")):
            for link in (dev / "serial/by-id").glob(pattern):
                if link.exists():
                    serial[str(link.resolve())] = name
        if arm_port and Path(arm_port).exists():
            serial[str(Path(arm_port).resolve())] = "팔 FTDI"
        for process in proc.iterdir():
            if not process.name.isdecimal():
                continue
            if time.monotonic() > deadline:
                return "장치 점유 확인 시간 초과"
            try:
                for fd in (process / "fd").iterdir():
                    if time.monotonic() > deadline:
                        return "장치 점유 확인 시간 초과"
                    try:
                        target = os.readlink(fd)
                    except FileNotFoundError:
                        continue
                    resource = serial.get(target)
                    if resource:
                        resource = f"{resource} {target}"
                    elif target.startswith("socket:["):
                        resource = sockets.get(target[8:-1])
                    if resource:
                        return f"{resource} (PID {process.name})"
            except PermissionError:
                driver = _restricted_driver(process)
                if driver:
                    return f"{driver} (PID {process.name}) · 장치 점유 확인 권한 없음"
            except (FileNotFoundError, ProcessLookupError):
                continue
        # A network socket is visible even when the owning process restricts fd access.
        return next(iter(sockets.values()), None)
    except (OSError, ValueError, IndexError) as exc:
        return f"장치 점유 확인 실패 ({exc})"
