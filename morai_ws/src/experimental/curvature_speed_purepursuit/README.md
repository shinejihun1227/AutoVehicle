# 곡률 기반 속도 계획 Pure Pursuit 실험

기본 실행은 기존 `purepursuit_mgeo`, `control_mux`, `/ctrl_cmd`와 분리된
미리보기 모드다. 통합 launch에서 선택하면 이 패키지가 nominal 제어기로 연결된다.

기존 주행 코드를 보존한 채 곡률 기반 주행 제어를 독립적으로 시험할 수 있도록
구성되어 있다. 센서 입력은 MORAI에서 들어온 원본 토픽을 그대로 사용한다.

## 처리 내용

1. `2026_molit_comp_global_path.txt`를 읽는다.
2. 연속 중복점을 제거한다. 첫 점과 마지막 점이 같은 한 바퀴 종료 표시는 유지한다.
3. 세 점 기반 부호 있는 곡률을 계산하고 median smoothing한다.
4. `v_curve = sqrt(a_y_max / abs(kappa))`로 곡률 속도 상한을 계산한다.
5. 가속도 제한을 순방향으로, 감속 제한을 종료점에서 역방향으로 적용한다.
6. 마지막 점의 목표속도를 0km/h로 두고 한 바퀴 종료 시 정지한다.
7. 현재 pose를 경로 선분에 투영하고, 진행거리 기준으로 Pure Pursuit 조향을 계산한다.
8. 목표속도와 현재속도의 오차를 PI 제어해 MORAI accel/brake 명령으로 변환한다.

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
/experimental/curvature_accel_command
/experimental/curvature_brake_command
/experimental/curvature_steering
/experimental/curvature_progress
/experimental/curvature_goal_reached
```

## 주요 파라미터

```text
max_speed_kph                 직선 및 전체 속도 상한
lateral_accel_limit_mps2     허용 횡가속도
max_accel_mps2                속도 profile의 가속 제한
max_decel_mps2                속도 profile의 감속 제한
speed_kp                      목표속도 PI 비례 게인
speed_ki                      목표속도 PI 적분 게인
speed_integral_limit_kph_s   km/h·s 기준 PI 적분항 제한
speed_error_deadband_kph     km/h 기준 속도 오차 deadband
curvature_half_window_points  3점 곡률 계산 간격
curvature_smoothing_window    곡률 median window
lookahead_min_m               최소 lookahead
lookahead_gain                속도에 따른 lookahead 증가량
final_speed_mps               마지막 종료점 목표속도
pose_timeout_sec              odometry가 끊겼을 때 정지하는 시간
```

현재 값은 동작 확인을 위한 초기 시험값이다. 실제 차량의 경로·속도·조향 방향에
맞춰 제어 파라미터를 조정해야 한다.

속도 입력·목표속도·속도 모니터링 토픽은 km/h 단위다. 곡률은 1/m,
경로의 거리와 가속도 제한은 각각 m, m/s² 단위를 사용하며 내부 물리 계산 전에
속도를 m/s로 변환한다.
