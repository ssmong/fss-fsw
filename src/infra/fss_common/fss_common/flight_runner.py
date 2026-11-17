"""Keep GUI-started flight processes and their hardware lease in one lifetime."""

import argparse
import os
import signal
import subprocess
import sys
import time

from fss_common.hardware_ownership import HARDWARE_FD


def stop_children(children):
    """Signal only process groups created by this runner, including descendants."""
    remaining = list(children)
    for signum, timeout in ((signal.SIGINT, 5.0), (signal.SIGTERM, 3.0), (signal.SIGKILL, 1.0)):
        for child in remaining:
            try:
                os.killpg(child.pid, signum)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + timeout
        while remaining and time.monotonic() < deadline:
            alive = []
            for child in remaining:
                child.poll()  # Reap exited group leaders before checking the group.
                try:
                    os.killpg(child.pid, 0)
                except ProcessLookupError:
                    continue
                alive.append(child)
            remaining = alive
            if remaining:
                time.sleep(0.05)
        if not remaining:
            return


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--imu-script")
    parser.add_argument("--imu-log")
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command or (args.imu_log and not args.imu_script):
        parser.error("a flight command is required; --imu-log requires --imu-script")
    try:
        lease_fd = int(os.environ[HARDWARE_FD])
        os.fstat(lease_fd)
    except (KeyError, ValueError, OSError) as exc:
        print(f"Flight runner requires an inherited hardware lease: {exc}", file=sys.stderr)
        return 1

    stopping = 0

    def on_signal(signum, _frame):
        nonlocal stopping
        stopping = signum

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    children = []
    imu = None
    imu_log = None
    try:
        if args.imu_script:
            if args.imu_log:
                imu_log = open(args.imu_log, "ab", buffering=0)
            imu = subprocess.Popen(["bash", args.imu_script], stdout=imu_log,
                                   stderr=subprocess.STDOUT, start_new_session=True,
                                   pass_fds=(lease_fd,))
            children.append(imu)
        if stopping:
            return 128 + stopping
        flight = subprocess.Popen(command, start_new_session=True, pass_fds=(lease_fd,))
        children.append(flight)
        while not stopping:
            code = flight.poll()
            if code is not None:
                print(f"Flight process exited ({code})", file=sys.stderr)
                return code if code >= 0 else 128 - code
            if imu is not None and imu.poll() is not None:
                print(f"IMU process exited while flight was running ({imu.returncode})", file=sys.stderr)
                return imu.returncode if imu.returncode > 0 else 1
            time.sleep(0.05)
        return 128 + stopping
    except OSError as exc:
        print(f"Flight runner failed: {exc}", file=sys.stderr)
        return 1
    finally:
        stop_children(children)
        if imu_log is not None:
            imu_log.close()
        os.close(lease_fd)


if __name__ == "__main__":
    sys.exit(main())
