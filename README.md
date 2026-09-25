# MORAI 자율주행 통합 저장소

현장에서 실행할 시험 순서·명령·통과 기준·결과 기록표는
[MORAI 현장 테스트 체크리스트 — 2026-09-25 개정](morai_ws/docs/FIELD_TEST_CHECKLIST_20260921_KO.md)에 정리했습니다.
현재 GitHub 코드의 `curvature` → `curvature_signal` → `obstacle` → `merge` → `full` 순서로 검증합니다.

Ubuntu 22.04 + RTX 4090 제어 PC와 Windows MORAI를 사용하는 경우:
**[Docker 처음 설치 → 재부팅 후 실행 → 기능별 테스트 안내](morai_ws/docs/TWO_PC_DOCKER_HIGHWAY_KO.md)**를 따릅니다.
IP는 Ubuntu `192.168.0.185`, MORAI `192.168.0.148` 기준입니다.

새로운 개발 기준은 `morai_ws/`입니다. 기존에 여러 팀과 실험에서 사용하던
실행 코드는 정리했으며, 대회 경로·MGeo·센서·팀 간 인터페이스 정보는
`morai_ws/docs`와 `morai_ws/data`에서 관리합니다.

카메라 포트는 다음으로 통일합니다.

```text
MORAI 1100 → Ubuntu 1101   전방
MORAI 1110 → Ubuntu 1111   좌측
MORAI 1120 → Ubuntu 1121   우측
MORAI 1130 → Ubuntu 1131   네 번째 카메라
```

시작할 때는 [morai_ws/README.md](morai_ws/README.md), [MGeo와 대회 경로 분석](morai_ws/docs/MAP_AND_PATH_ANALYSIS.md),
[이관 계획](morai_ws/docs/MIGRATION_PLAN.md), [세부 규정집 요약](morai_ws/docs/세부규정집_요약.md)을
먼저 확인합니다.

현재 코드의 폴더별 역할과 실제 실행 순서는
[현재 구성 및 실행 보고서](morai_ws/docs/현재_구성_및_실행_보고서.md)를 기준으로 확인합니다.

새 Docker를 만들고 `morai_msgs` 설치부터 곡률 기반 주행·GPS blackout까지 처음부터
검증하려면 [새 Docker 통합 테스트 가이드](TEST_FROM_SCRATCH_KO.md)를 먼저 실행합니다.
