"""WSL/Linux에서 실제 로봇팔 dry-run 런치 기동과 단일 SIGINT 종료를 검사한다."""

import argparse
import os
import selectors
import signal
import subprocess
import sys
import sysconfig
import time
from pathlib import Path


def main() -> None:
    """자식 런치 프로세스만 종료하며 실제 장치 모드는 허용하지 않는다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--full-stack", action="store_true", help="SIL+manager+밸브+항법+팔 기동")
    parser.add_argument(
        "--with-body-gnc", action="store_true", help="전체 SIL에서 기존 추력 GNC도 기동"
    )
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "75"
    env["ROS_LOCALHOST_ONLY"] = "1"
    env["FSS_HYDRA_OVERRIDES"] = "arm_driver_node.dry_run=true"
    env["PYTHONPATH"] = os.pathsep.join([sysconfig.get_path("purelib"), env.get("PYTHONPATH", "")])
    command = [sys.executable, "/opt/ros/humble/bin/ros2", "launch", "fss_bringup", "arm.launch.py"]
    markers = ["arm ready: dry_run=True"]
    if args.full_stack:
        command[-1] = "fss_full.launch.py"
        command.extend(
            ["use_sim:=true", "use_arm:=true", f"use_gnc:={str(args.with_body_gnc).lower()}"]
        )
        markers.extend(
            ["SIL 플랜트 시작", "EKF 노드 시작", "시스템 관리자 시작", "밸브 드라이버 노드 시작"]
        )
        if args.with_body_gnc:
            markers.append("GNC 노드 시작")
    elif args.with_body_gnc:
        parser.error("--with-body-gnc requires --full-stack")
    with args.output.open("x", encoding="utf-8") as log:
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
        )
        ready = False
        lines = []
        try:
            selector = selectors.DefaultSelector()
            selector.register(process.stdout, selectors.EVENT_READ)
            deadline = time.monotonic() + 120
            ready_since = None
            while time.monotonic() < deadline and process.poll() is None:
                for key, _ in selector.select(timeout=0.5):
                    line = os.read(key.fd, 65536).decode("utf-8", errors="replace")
                    lines.append(line)
                    log.write(line)
                    log.flush()
                    if all(marker in "".join(lines) for marker in markers):
                        ready = True
                        if ready_since is None:
                            ready_since = time.monotonic()
                if ready and time.monotonic() - ready_since >= 2:
                    break
            selector.close()
            # 프로세스 그룹 전체에 보내면 launch와 노드에 중복 전달되므로 부모에만 보낸다.
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            remainder, _ = process.communicate(timeout=15)
            remainder = remainder.decode("utf-8", errors="replace")
            log.write(remainder)
            lines.append(remainder)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        output = "".join(lines)
        if not ready or "Traceback" in output or "process has died" in output:
            raise RuntimeError(f"launch verification failed; see {args.output}")
        if process.returncode != 0:
            raise RuntimeError(f"launch exit code {process.returncode}; see {args.output}")
        print(f"PASS: dry-run launch started and exited cleanly; {args.output}")


if __name__ == "__main__":
    main()
