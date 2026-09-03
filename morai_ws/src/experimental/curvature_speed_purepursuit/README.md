# curvature_speed_purepursuit

기존 `purepursuit_mgeo`, `control_mux`, `/ctrl_cmd`와 연결하지 않는 곡률 기반 속도 계획 실험 패키지다.

기존 주행 코드를 보존한 채 곡률 주행과 localization 입력 노이즈 내성을 함께
시험할 수 있도록 구성되어 있다.

## 처리 내용

1. `2026_molit_comp_global_path.txt`를 읽는다.
2. 연속 중복점을 제거한다. 첫 점과 마지막 점이 같은 한 바퀴 종료 표시는 유지한다.
3. 세 점 기반 부호 있는 곡률을 계산하고 median smoothing한다.
4. `v_curve = sqrt(a_y_max / abs(kappa))`로 곡률 속도 상한을 계산한다.
5. 가속도 제한을 순방향으로, 감속 제한을 종료점에서 역방향으로 적용한다.
6. 마지막 점의 목표속도를 0m/s로 두고 한 바퀴 종료 시 정지한다.
7. 현재 pose를 경로 선분에 투영하고, 진행거리 기준으로 Pure Pursuit 조향을 계산한다.

## 기본 미리보기 실행

```bash
roslaunch curvature_speed_purepursuit curvature_speed_purepursuit.launch \
  path_file:=/home/<user>/morai_ws/data/routes/2026_molit_comp_global_path.txt \
  pose_topic:=/localization/odometry
```

기본값은 명령을 발행하지 않는다. 다음 토픽으로 결과를 확인할 수 있다.

```text
/experimental/curvature_reference_path
/experimental/curvature_lookahead_point
/experimental/curvature_value
/experimental/curvature_speed_limit
/experimental/curvature_speed_command
/experimental/curvature_steering
/experimental/curvature_progress
/experimental/curvature_goal_reached
```

## 독립 폐루프 테스트

기존 차량 모델 노드를 테스트용으로만 재사용하며, 모든 입력·출력 토픽을
`/experimental/*`로 분리한다.

```bash
roslaunch curvature_speed_purepursuit curvature_speed_purepursuit_closed_loop.launch \
  path_file:=/home/<user>/morai_ws/data/routes/2026_molit_comp_global_path.txt
```

이 launch는 기존 Pure Pursuit를 실행하지 않는다.

## 노이즈 포함 미리보기

노이즈 포함 시험에서는 다음 순서로 데이터가 흐른다.

```text
raw odometry
  -> artificial white noise + bias random walk + dropout
  -> noisy odometry
  -> median + EMA + jump/speed/yaw rejection
  -> filtered odometry
  -> curvature-speed Pure Pursuit
  -> /experimental/curvature_ctrl_cmd
```

실행 명령:

```bash
roslaunch curvature_speed_purepursuit curvature_speed_purepursuit_noisy.launch \
  path_file:=/home/<user>/morai_ws/data/routes/2026_molit_comp_global_path.txt \
  input_topic:=/localization/odometry
```

기본값은 `publish_command=false`이므로 실험 명령을 발행하지 않는다. 결과 확인이
끝난 뒤에만 다음처럼 별도 명령 토픽 발행을 켤 수 있다.

```bash
roslaunch curvature_speed_purepursuit curvature_speed_purepursuit_noisy.launch \
  publish_command:=true
```

노이즈 시험 관련 토픽은 다음과 같다.

```text
/experimental/curvature_noisy_odometry
/experimental/curvature_filtered_odometry
/experimental/curvature_noise_filter_status
/experimental/curvature_ctrl_cmd
```

## 노이즈 포함 독립 폐루프 테스트

테스트용 kinematic vehicle이 만드는 raw odometry에 노이즈와 dropout을 넣은 뒤,
필터 출력으로 곡률 기반 Pure Pursuit를 주행시킨다.

```bash
roslaunch curvature_speed_purepursuit \
  curvature_speed_purepursuit_noisy_closed_loop.launch \
  path_file:=/home/<user>/morai_ws/data/routes/2026_molit_comp_global_path.txt
```

이 폐루프에서 사용하는 토픽은 모두 `/experimental/*`이며 기존 `/ctrl_cmd`,
기존 localization/EKF, 기존 Pure Pursuit와 연결되지 않는다.

주의: 이 노이즈 노드는 `/gps`, `/Imu`의 원시 센서 노이즈를 재현하는 것이 아니라
곡률 제어기에 들어가는 `nav_msgs/Odometry` 입력의 noise robustness를 시험한다.
실제 GPS/IMU/EKF 단계의 노이즈 검증은 기존 `stability_stack` 실험과 별도로 수행한다.

## 주요 파라미터

```text
max_speed_mps                 직선 및 전체 속도 상한
lateral_accel_limit_mps2     허용 횡가속도
max_accel_mps2                속도 profile의 가속 제한
max_decel_mps2                속도 profile의 감속 제한
curvature_half_window_points  3점 곡률 계산 간격
curvature_smoothing_window    곡률 median window
lookahead_min_m               최소 lookahead
lookahead_gain                속도에 따른 lookahead 증가량
final_speed_mps               마지막 종료점 목표속도
pose_timeout_sec              filtered odometry가 끊겼을 때 정지하는 시간
```

노이즈/필터 launch 파라미터:

```text
position_noise_std_m          위치 white noise 표준편차
yaw_noise_std_rad             yaw white noise 표준편차
velocity_noise_std_mps        속도 white noise 표준편차
position_bias_random_walk_m_sqrt_s  위치 bias random walk 세기
yaw_bias_random_walk_rad_sqrt_s     yaw bias random walk 세기
velocity_bias_random_walk_mps_sqrt_s 속도 bias random walk 세기
dropout_probability           측정 dropout 확률
median_window_size            median 필터 창 크기
ema_alpha                     EMA 반응 비율
max_position_jump_m           위치 jump 거부 기준
max_measurement_speed_mps     측정 위치로 계산한 속도 거부 기준(위치 noise 차분을 고려해 초기값 50)
max_yaw_jump_rad              yaw jump 거부 기준
```

현재 값은 동작 확인을 위한 초기 시험값이다. 실제 센서 주기와 차량 속도,
GPS/IMU 오차 수준에 맞춰 노이즈 크기와 rejection threshold를 조정해야 한다.

모든 속도는 m/s, 곡률은 1/m, 가속도는 m/s² 단위다.
