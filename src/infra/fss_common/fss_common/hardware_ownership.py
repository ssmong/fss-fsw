"""Single Jetson-wide hardware lease, shared by native and legacy launchers."""

import errno
import os
import stat
from pathlib import Path


HARDWARE_LOCK = Path("/run/lock/fss-fsw-hardware.lock")
HARDWARE_FD = "FSS_HARDWARE_LEASE_FD"


def claim_hardware(lock_path=HARDWARE_LOCK, probe=None):
    """Return an open file; the last launcher to close its copy releases the lease."""
    import fcntl

    if probe is None:
        from fss_common.mission_device_probe import device_resource_owner
        probe = device_resource_owner
    flags = os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC | os.O_NONBLOCK
    try:
        fd = os.open(lock_path, flags | os.O_CREAT | os.O_EXCL, 0o644)
        os.fchmod(fd, 0o644)
    except FileExistsError:
        fd = os.open(lock_path, flags)
    lease = os.fdopen(fd, "rb")
    try:
        actual = os.fstat(fd)
        if not stat.S_ISREG(actual.st_mode):
            raise RuntimeError(f"기체 하드웨어 잠금 파일 확인 필요 · {lock_path}")
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            if exc.errno not in (errno.EAGAIN, errno.EACCES):
                raise
            inherited = os.environ.get(HARDWARE_FD)
            if inherited is not None:
                try:
                    existing = int(inherited)
                    if os.path.samestat(actual, os.fstat(existing)):
                        # flock succeeds here only for the owning open file description.
                        fcntl.flock(existing, fcntl.LOCK_EX | fcntl.LOCK_NB)
                        owned = os.fdopen(os.dup(existing), "rb")
                        lease.close()
                        return owned
                except (ValueError, OSError):
                    pass
            raise RuntimeError("다른 FSS가 이 Jetson의 하드웨어를 사용 중입니다.") from exc
        if owner := probe():
            raise RuntimeError(f"기체 하드웨어 사용 중 · {owner}")
        return lease
    except BaseException:
        lease.close()
        raise


def claim_launch(context):
    # The launching process retains the lease until its LaunchContext is released.
    if getattr(context, "_fss_hardware_lease", None) is None:
        context._fss_hardware_lease = claim_hardware()
