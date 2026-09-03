# MORAI 자율주행 통합 저장소

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
