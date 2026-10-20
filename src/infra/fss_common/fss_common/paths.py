"""환경변수·설치 패키지의 설정과 소스 리포지토리 경로 탐색."""

import logging
import os
from pathlib import Path

logger = logging.getLogger(__name__)

# 리포 루트 판별 마커 (둘 다 존재해야 루트로 인정)
_ROOT_MARKERS = ("config", "src")


def find_repo_root(start: str | Path | None = None) -> Path:
    """start(기본: 이 파일)에서 상위로 올라가며 리포 루트를 찾는다.

    symlink-install 환경에서는 심볼릭 링크를 따라 소스 트리에서 탐색된다.

    Raises:
        RuntimeError: 루트를 찾지 못한 경우 (일반 colcon build로 복사-설치된
            경우 소스 트리를 알 수 없음 — --symlink-install 필요)
    """
    origin = Path(start) if start is not None else Path(__file__)
    p = origin.resolve()
    for parent in [p, *p.parents]:
        if all((parent / m).is_dir() for m in _ROOT_MARKERS):
            return parent
    raise RuntimeError(
        f"[fss_common.paths] 리포 루트를 찾을 수 없음 (탐색 시작: {origin}). "
        "colcon build --symlink-install로 빌드했는지 확인하세요."
    )


def config_file(*rel: str, start: str | Path | None = None) -> Path:
    """실행 환경의 config/ 하위 파일 경로를 반환한다. 없으면 예외.

    Raises:
        FileNotFoundError: 파일이 존재하지 않는 경우
    """
    path = config_dir(start=start) / Path(*rel)
    if not path.is_file():
        raise FileNotFoundError(f"[fss_common.paths] 설정 파일 없음: {path}")
    return path


def config_dir(*, start: str | Path | None = None) -> Path:
    """FSS_CONFIG_DIR → 설치된 bringup 설정 → 소스 트리 순서로 탐색한다.

    C++ 노드도 같은 환경변수와 bringup share/config를 사용한다. 환경변수를
    지정하면 소스/설치 위치와 무관하게 모든 노드가 같은 설정을 읽는다.
    """
    override = os.environ.get("FSS_CONFIG_DIR")
    if override:
        path = Path(override).expanduser().resolve()
    else:
        try:
            from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
        except ImportError:
            path = find_repo_root(start) / "config"
        else:
            try:
                path = Path(get_package_share_directory("fss_bringup")) / "config"
            except PackageNotFoundError:
                path = find_repo_root(start) / "config"
    if not path.is_dir():
        raise FileNotFoundError(f"[fss_common.paths] 설정 디렉토리 없음: {path}")
    return path
