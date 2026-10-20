# PLC 래더 프로그램 (XG5000 프로젝트)

LS Electric XBM-DN16H2 PLC용 래더 프로그램. **원호 연구원 작성 — 수정 금지**.

## 파일 구성

| 파일 | 용도 |
|------|------|
| `FSS.xgwx` | XG5000 메인 프로젝트 파일 (이것을 열면 됨) |
| `FSS.state` | 작업 상태 저장 (XG5000 자동 관리) |
| `FSS.xgwx_bkx0` ~ `FSS.xgwx_bkx2` | 자동 백업 |

## 주요 로직

- **에어베어링 제어**: `M00100` → `P00027` 디지털 출력
  - Modbus 쓰기: `write_register(99, 1)` → 래더에서 D00099 → M00100 변환
- **비례밸브 8개**: `D00100~D00107` → DC04A #1/#2 아날로그 전류 출력
  - 값 범위: 0–4000 → 4–20mA
  - OUTEN 신호 항상 활성 (`_ON → MOV h000F → _02_OUTEN`, `_03_OUTEN`)

## Modbus 매핑 (XG5000 설정)

| 영역 | 시작 주소 | 용도 |
|------|----------|------|
| 비트 (M) | M00000 | 에어베어링 on/off (M00100) |
| 워드 (D) | D00000 | 비례밸브 + 메인 밸브 (D00099, D00100~D00107) |

## PLC 네트워크 설정

- IP: `192.168.1.2`
- Modbus TCP 서버 포트: 502

## XG5000으로 래더 다운로드 절차

상세 절차는 `docs/hardware/` 참조.

1. Windows PC에 XG5000 설치 (LS Electric 홈페이지)
2. PC 이더넷 IP를 `192.168.1.x` 대역으로 설정 (예: 192.168.1.10)
3. XG5000 실행 → 파일 → 열기 → `FSS.xgwx` 선택
4. 온라인 → 접속 설정 → Ethernet, IP `192.168.1.2`
5. 온라인 → 접속
6. 온라인 → 쓰기 (프로그램 + 파라미터)
7. 온라인 → 모드 전환 → RUN
8. PLC 전면 RUN LED 점등 확인

## 관련 파일

- 하드웨어 설정: `config/hardware/plc.yaml`
- HAL 드라이버: `src/hardware/fss_hal/fss_hal/plc_hal.py`
- 레지스터 맵: `docs/hardware/003_HAL.md`
