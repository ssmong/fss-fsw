"""설치 전에도 로봇팔 순수 모듈을 검사할 수 있도록 경로를 등록한다."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src/hardware/fss_arm"))
