# MORAI 기반 AI융합자율주행 프로젝트 마스터 플랜

## 1. 프로젝트 목표

본 프로젝트의 목표는 MORAI SIM: Drive 환경에서 대회 규정을 만족하는 ROS1 기반 자율주행 시스템을 구축하고, 개발 과정 전체를 논문형 보고서로 누적 정리하는 것이다. 단순히 시뮬레이터 예제를 실행하는 것을 넘어, 환경 설정, 센서 설계, 통신 구조, 알고리즘 선택, 실험 검증, 대회 운영 절차까지 하나의 체계로 연결한다.

## 2. 기준 문서

- 대회 규정집: `2026 대학생_AI_SW_모빌리티_경진대회_AI융합자율주행_부문_규정.pdf`
- MORAI 공식 가이드: `https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/26.R1/`

규정집은 허용 환경, 센서, 네트워크, 실격 조건을 결정하는 최상위 문서로 사용한다. MORAI 공식 가이드는 시뮬레이터 설정, ROS 통신, 센서 설정, 예제 실행의 기준 문서로 사용한다.

## 3. 개발 단계

### Phase 1. 환경 및 규정 정렬

- Ubuntu 20.04 + ROS1 Noetic 기준 환경 구축
- MORAI multi-system 구조 이해
- 대회 허용 센서 및 네트워크 목록 정리
- 실행 후 PC 조작 불가 조건을 반영한 자동 실행 구조 설계

완료 기준:
- 대회 규정 제약 문서 작성
- ROS workspace skeleton 생성
- bash/alias/launch 실행 전략 초안 작성

### Phase 2. 통신 기반 구축

- MORAI와 참가팀 PC 간 LAN 연결 구조 정리
- `rosbridge_server` 사용 여부 검토
- UDP 제어 명령 및 ROS 센서 토픽 구조 정의
- rqt, rostopic, rosbag 기반 점검 루틴 작성

완료 기준:
- 센서별 입력 토픽 후보 정의
- 제어 출력 토픽/UDP 명령 정의
- pre-run network checklist 작성

### Phase 3. Localization Baseline

- GPS, IMU, vehicle status 기반 ego pose 추정
- map 좌표계와 vehicle 좌표계 관계 정리
- waypoint 기반 경로 추종에 필요한 상태 벡터 정의

완료 기준:
- 현재 위치, heading, velocity 추정 노드 초안
- localization 품질 확인용 로그 포맷 정의

### Phase 4. Perception Baseline

- 3D LiDAR 기반 장애물 후보 검출
- Camera 기반 신호/표지/AI 미션 요소 검토
- CollisionData를 디버깅용 안전 신호로 사용

완료 기준:
- LiDAR point cloud 수신 확인
- 장애물 bounding/cluster 후보 생성
- Camera 처리 파이프라인 구조 정의

### Phase 5. Planning and Control

- global waypoint follower 구현
- local obstacle avoidance 전략 추가
- speed profile, steering, accel/brake command 설계
- checkpoint 실패 복구 전략 수립

완료 기준:
- 정상 주행 baseline
- 장애물 회피 baseline
- 복구 가능한 실패 모드 정의

### Phase 6. AI Mission Strategy

- AI 미션 공지 내용에 따라 camera/LiDAR/상태 정보 기반 인식 모듈 추가
- 데이터셋 수집, 라벨링, 학습, 추론 파이프라인 설계
- 대회 당일 inference latency 관리

완료 기준:
- AI 미션별 입력/출력 명세
- 모델 추론 노드 및 fallback 동작 정의

### Phase 7. Competition Runbook

- 대회 당일 실행 순서 문서화
- one-command launch 구성
- 네트워크, 센서, 토픽, 제어 명령 사전 점검
- 실패 시 operator가 할 수 있는 범위와 할 수 없는 범위 구분

완료 기준:
- 실행 체크리스트 완성
- dry-run 로그 3회 이상 축적
- 최종 기술보고서 초안 완성

## 4. 보고서 운영 방식

보고서는 다음 구조로 누적한다.

1. 서론: 대회 목표와 문제 정의
2. 관련 기술: MORAI, ROS1, 센서, 자율주행 알고리즘
3. 시스템 설계: 하드웨어/소프트웨어/네트워크/토픽 구조
4. 알고리즘: localization, perception, planning, control, AI mission
5. 실험: 시나리오, 평가 기준, 로그, 실패 분석
6. 결론: 성능, 한계, 개선 방향

Notion에는 진행 로그와 보고서 초안을 누적하고, GitHub에는 코드와 실험 설정을 버전 관리한다.

