# 002. fss_common — 공용 유틸리티

**상태: 완료** (2026-07-14 전면 개편)
**패키지 유형:** `ament_python`
**위치:** `src/infra/fss_common/`

## 개요

설정 파이프라인(SSoT)과 공용 수학 함수를 제공한다.
런치 파일·노드·스크립트가 모두 이 패키지를 통해 `config/` Hydra 트리에 접근한다.

## 모듈 구조

```
fss_common/
├── __init__.py
├── paths.py              # 리포 루트/설정 파일 탐색 (마커 기반, 실패 시 예외)
├── hydra_bridge.py       # Hydra 컴포즈 → 노드별 ROS2 파라미터 YAML 생성
├── launch_utils.py       # 런치용 node_params() + FSS_HYDRA_OVERRIDES 처리
└── math_utils.py         # 2D 수학 유틸리티 (wrap_angle, build_thruster_matrix 등)
```

## paths.py

마커 디렉토리(`config/`+`src/`) 기반 상향 탐색으로 리포 루트를 찾는다.

| 함수 | 설명 |
|------|------|
| `find_repo_root(start=None)` | 리포 루트 반환. 못 찾으면 `RuntimeError` (symlink-install 필요) |
| `config_file(*rel, start=None)` | `config/` 하위 파일 경로. 없으면 `FileNotFoundError` |

사용처: `gnc_node`(thruster_geometry, lqr_gains), `plant_model_node`(thruster_geometry).

## hydra_bridge.py

`config/` Hydra 트리(유일 설정 소스)를 컴포즈해 노드별 ROS2 파라미터 파일을 만든다.
모든 실패는 예외로 전파된다 (silent fallback 금지).

| 함수 | 설명 |
|------|------|
| `load_hydra_config(config_path, config_name, overrides)` | Hydra Compose API 로드 |
| `hydra_to_ros2_params(cfg, node_name)` | 노드 서브트리 추출 (`KeyError` on 누락) |
| `write_ros2_param_yaml(params, node_name, output_path)` | `노드명: ros__parameters:` 형식 저장 |
| `generate_node_param_files(node_names, out_dir, overrides, config_dir)` | 일괄 생성 (런치/스크립트 공용 진입점) |

## launch_utils.py

| 함수 | 설명 |
|------|------|
| `node_params(node_name)` | 해당 노드의 파라미터 YAML 생성 후 경로 반환 (런치에서 `parameters=[...]`에 사용) |
| `hydra_overrides_from_env()` | `FSS_HYDRA_OVERRIDES` 환경변수 파싱 (run_experiment가 설정) |

## math_utils.py

| 함수 | 설명 |
|------|------|
| `wrap_angle(angle)` | [-π, π) 정규화 — vicon_bridge에서 사용 |
| `build_thruster_matrix(thrusters)` | 3×8 B행렬 생성 — gnc_node/plant_model_node에서 사용 |
| `rotation_matrix_2d` / `body_to_table` / `table_to_body` / `clamp` | 2D 변환/유틸 |

## 검증

```bash
python scripts/validate_config.py   # config 트리 ↔ 노드 declare 목록 정합 검증
```
