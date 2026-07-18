# Sensor Strategy

## 1. Sensor Selection

규정상 사용 가능한 센서는 GPS, IMU, 3D LiDAR, Camera다. 센서 수와 네트워크는 규정 범위 안에서만 선택한다.

초기 추천 구성:

```text
GPS: 1
IMU: 1
3D LiDAR: 1 front, optional 1 rear
Camera: front, mission-dependent
```

## 2. GPS

목적:
- global position 추정
- waypoint tracking 기준 위치 제공

사용처:
- localization
- path progress calculation
- checkpoint distance estimation

주의:
- raw GPS만으로 steering을 만들면 heading noise에 취약하다.
- IMU yaw와 vehicle status를 함께 사용한다.

## 3. IMU

목적:
- yaw, angular velocity, acceleration 추정
- GPS 기반 위치 추정의 방향 안정화

사용처:
- localization
- controller heading error calculation
- motion state estimation

주의:
- MORAI IMU 좌표계와 ROS base_link 좌표계를 확인해야 한다.

## 4. 3D LiDAR

목적:
- 전방 장애물 감지
- 정적 장애물 회피
- local drivable area 추정

초기 설치:
- front roof/body 외부 장착
- 차량 바디 기준 0.4 m 이내

추천 처리:
- point cloud crop
- ground removal
- clustering
- obstacle distance estimation

주의:
- ROS 사용 시 ROS-Velodyne 드라이버와 launch 파일이 필요하다.
- LiDAR data rate와 rosbridge 수신 주기가 크게 어긋나면 주행이 불안정해질 수 있다.

## 5. Camera

목적:
- AI 미션 객체 인식
- 신호, 표지, 차선, 특수 상황 인식 가능성 검토

초기 설치:
- front-facing camera
- pitch angle +-30도 이내

주의:
- 카메라 개수는 미션별 안내 예정이므로 미리 확정하지 않는다.
- frame rate와 inference latency를 반드시 측정한다.

## 6. Sensor Fusion Baseline

초기 단계에서는 복잡한 fusion보다 신뢰 가능한 최소 구조를 우선한다.

```text
GPS + IMU + vehicle status -> ego state
LiDAR -> obstacle candidates
Camera -> mission objects
ego state + obstacles + mission objects -> planning
```

## 7. Sensor Validation Checklist

- 토픽이 생성되는가?
- message timestamp가 갱신되는가?
- data rate가 설정값과 비슷한가?
- 좌표계 방향이 맞는가?
- 센서 frame이 base_link와 연결되는가?
- 주행 중 지연이 커지지 않는가?

