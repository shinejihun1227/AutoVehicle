# MORAI AI Convergence Autonomous Driving Project

이 저장소는 `2026 대학생 AI/SW 모빌리티 경진대회 AI융합자율주행 부문`을 목표로 MORAI SIM: Drive, Ubuntu 20.04, ROS1 Noetic 기반의 자율주행 시스템을 단계적으로 구축하기 위한 프로젝트입니다.

프로젝트 운영 원칙은 단순합니다.

1. 대회 규정을 최상위 제약조건으로 둔다.
2. MORAI 공식 가이드를 환경 및 통신 설정 기준 문서로 둔다.
3. 코드, 실험, 보고서를 동시에 관리한다.
4. 모든 실행은 대회 당일 재현 가능한 `one-command launch` 구조를 목표로 한다.

## Current Baseline

- Competition OS: Ubuntu 20.04
- ROS version: ROS1 Noetic
- Simulator: MORAI SIM: Drive
- Vehicle model: `2023_Hyundai_ioniq5`
- Competition map: `R-KR_PG_K-City_2025`
- Main architecture: `sensing -> localization -> perception -> planning -> control`

## Repository Layout

```text
.
├── configs/                 # MORAI, sensor, network, launch configuration drafts
├── docs/                    # Research-style project documents and reports
├── experiments/             # Scenario notes, test logs, and result summaries
├── ros_ws/                  # ROS1 Noetic catkin workspace skeleton
├── scripts/                 # Setup, launch, and pre-run check helpers
└── README.md
```

## First Milestones

1. 대회 규정 기반 개발 환경 고정
2. MORAI multi-system 통신 구조 이해
3. ROS bridge 및 UDP/ROS 센서 수신 전략 결정
4. GPS/IMU 기반 localization baseline 구현
5. LiDAR/Camera 기반 perception baseline 구현
6. 경로 추종 및 제어 baseline 구현
7. 대회 시나리오별 실험 로그와 보고서 누적

