# 대회 규정 기반 제약조건 정리

## 1. 최상위 원칙

대회 규정집은 이 프로젝트의 최상위 제약조건이다. 구현 편의나 연구 확장성이 있더라도 규정에 없는 센서, 네트워크, 실행 환경을 사용하면 실격 또는 감점 위험이 있다.

## 2. 운영 환경

- 참가팀 PC는 데스크톱 PC만 허용한다.
- OS는 Ubuntu 20.04로 설정한다.
- ROS는 ROS1 Noetic을 사용한다.
- Ubuntu OS 세팅은 로컬 환경으로 준비해야 한다.
- VMware, VirtualBox 사용법에 대한 지원은 불가하다고 명시되어 있다.
- 대회 당일 제공되는 판정 프로그램 및 연습용 프로그램으로 네트워크 연결을 사전 확인해야 한다.

## 3. 경기 진행 환경

경기 환경은 multi-system 형태다.

- Server PC: Windows 11, 시뮬레이터 구동, 동역학 및 제어 입력 처리
- Client PC: Windows 11, 시뮬레이터 구동, 판정 프로그램 실행
- 참가팀 PC: Ubuntu 20.04, 자율주행 알고리즘 실행, ros_bridge 실행 가능

연결 방식:
- Server PC, Client PC, 참가팀 PC는 switching hub를 통해 LAN-to-LAN으로 연결한다.
- 세팅 완료 후 모든 팀이 동시에 주행을 시작한다.

## 4. 경기 운영상 중요한 조건

- 미션별 주행 기회는 2회 부여된다.
- PC 제출 후 코드 수정은 불가능하다. 단, 환경 세팅 관련 사항은 예외로 취급될 수 있다.
- 참가팀 PC 제출 시간이 안내된 시간보다 늦으면 미션별 점수에서 30점 추가 감점될 수 있다.
- 자율주행 코드 실행 이후 operator는 PC를 조작할 수 없다.
- 주행 불능 판정 시 10초 이후 마지막 체크포인트로 돌아갈 기회가 주어진다.
- 동일 체크포인트로 3회 돌아갔으나 더 이상 진행 불가하면 다음 체크포인트로 넘어가는 기회가 부여된다.

## 5. 차량 및 지도

- 차량 모델: `2023_Hyundai_ioniq5`
- 사용 맵: `R-KR_PG_K-City_2025`
- 최소 회전 반경: 5.87 m
- 최대 휠 각도: 40 deg
- 차량 길이: 4.635 m
- 차량 폭: 1.892 m
- 차량 높이: 2.434 m
- 축거: 3.000 m
- 전방 오버행: 0.845 m
- 후방 오버행: 0.79 m

## 6. 허용 센서

### GPS

- 개수: 최대 1대
- Data Rate: 자유
- 네트워크: UDP or ROS

### IMU

- 개수: 최대 1대
- Data Rate: 자유
- 네트워크: UDP or ROS

### 3D LiDAR

- 개수: 최대 2대
- Intensity type: Intensity
- 사용 가능 모델: VLP16, HDL32
- Rotation Rate: 자유
- 네트워크: UDP or ROS
- ROS 사용 시 ROS-Velodyne 드라이버 설치 및 launch 실행 필요

### Camera

- 개수: 미션별 안내 예정
- 해상도: 자유
- Frame Rate: 자유
- 네트워크: UDP or ROS
- pitch angle은 +-30도 이내 준수

## 7. 센서 설치 가능 범위

- 육안으로 확인 가능한 차량 외부 바디에만 장착 가능하다.
- 바디 기준 직선거리 0.4 m 이내 이격 장착은 허용된다.
- 차량 내부인 샤시 및 바닥에는 장착할 수 없다.
- Camera pitch angle은 +-30도 이내로 제한된다.

## 8. 사용 가능한 네트워크 및 정보

대회에서 사용 가능한 네트워크 및 정보는 다음과 같다.

- `[UDP] Ego Ctrl Cmd`
- `[UDP] CollisionData`
- `[UDP] Competition Vehicle Status`
- `[UDP or ROS] GPS sensor`
- `[UDP or ROS] IMU sensor`
- `[UDP or ROS] Camera sensor`
- `[UDP or ROS] 3D LiDAR sensor`

중요:
- `Ego Ctrl Cmd`는 종방향 및 횡방향 제어에 사용한다.
- `Ctrl_cmd` 제어 시 longi type 1번, 즉 accel/brake 제어를 사용한다.
- 규정 2.5에서 제시한 센서와 네트워크가 아닌 것을 사용하면 실격 사유가 될 수 있다.

## 9. 심사 구조

- 자율주행 미션: 70%, 700점
- AI 미션: 30%, 300점
- 상세 배점은 미션 공개와 동시에 공개된다.

## 10. 프로젝트 설계에 미치는 영향

- ROS2가 아니라 ROS1 Noetic 기준으로 개발한다.
- 최종 대회 실행 환경은 실제 Ubuntu 20.04 desktop PC로 맞춘다.
- 센서 수신은 ROS 중심으로 통합하되, 제어 출력은 UDP 명령을 우선 기준으로 설계한다.
- 실행 후 PC 조작 불가 조건을 만족하기 위해 launch 파일과 자동 점검 스크립트가 필수다.
- 허용 센서 외 정보를 쓰는 shortcut은 금지한다.

