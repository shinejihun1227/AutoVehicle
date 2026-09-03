1. Ubuntu 호스트에서 Docker 실행
xhost +SI:localuser:root

sudo docker start morai_noetic_gui

sudo docker exec -it \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  morai_noetic_gui \
  bash
컨테이너가 없으면 먼저 확인합니다.
sudo docker ps -a
2. Docker 내부에서 GitHub 코드 받기
처음 받는 경우:
cd /root

git clone \
  -b codex/curvature-only-drive \
  https://github.com/shinejihun1227/AutoVehicle.git \
  AutoVehicle

export MORAI_WS_PATH=/root/AutoVehicle/morai_ws
cd "$MORAI_WS_PATH"
이미 clone한 경우:
cd /root/AutoVehicle

git fetch origin
git checkout codex/curvature-only-drive
git pull --ff-only origin codex/curvature-only-drive

export MORAI_WS_PATH=/root/AutoVehicle/morai_ws
cd "$MORAI_WS_PATH"
3. ROS workspace 빌드
source /opt/ros/noetic/setup.bash

cd "$MORAI_WS_PATH"
catkin_make

source devel/setup.bash
새 터미널을 열 때마다 다음을 실행합니다.
source /opt/ros/noetic/setup.bash
source /root/AutoVehicle/morai_ws/devel/setup.bash
4. ROS Master 실행
새 Docker 터미널에서 실행합니다.
roscore
그 다음 다른 Docker 터미널을 열어 workspace를 source합니다.
sudo docker exec -it \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  morai_noetic_gui \
  bash
5. 센서 토픽 확인
rostopic list | grep -E 'gps|Imu|camera|lidar'
GPS와 IMU가 들어오는지 확인합니다.
rostopic hz /gps
rostopic hz /Imu
수신되지 않으면 코드 문제가 아니라 MORAI의 destination IP, Docker UDP 포트, 방화벽 또는 네트워크 설정을 먼저 확인해야 합니다.
192.168.0.148은 현재 알려진 MORAI PC 주소 예시입니다. 0902최신 설정 파일의 실제 MORAI IP가 다르면 아래 명령의 값을 바꿔야 합니다.
6. 곡률 주행만 테스트
먼저 제어를 끈 상태로 확인합니다.
roslaunch stability_stack morai_udp_ekf_curvature_only.launch \
  workspace_path:=/root/AutoVehicle/morai_ws \
  path_file:=/root/AutoVehicle/morai_ws/data/routes/2026_molit_comp_global_path.txt \
  morai_host_ip:=192.168.0.148 \
  enable_control:=false
다른 터미널에서 확인합니다.
rostopic hz /localization/gps
rostopic hz /localization/gps_filtered
rostopic hz /Imu_filtered
rostopic hz /localization/odometry
곡률과 속도 결과를 확인합니다.
rostopic echo /experimental/curvature_value
rostopic echo /experimental/curvature_speed_limit
rostopic echo /experimental/curvature_speed_command
rostopic echo /experimental/curvature_steering
rostopic echo /experimental/curvature_progress
rostopic echo /experimental/curvature_goal_reached
토픽 흐름이 정상이라면 기존 launch를 Ctrl+C로 종료한 뒤 실제 제어를 켭니다.
roslaunch stability_stack morai_udp_ekf_curvature_only.launch \
  workspace_path:=/root/AutoVehicle/morai_ws \
  path_file:=/root/AutoVehicle/morai_ws/data/routes/2026_molit_comp_global_path.txt \
  morai_host_ip:=192.168.0.148 \
  enable_control:=true
7. 카메라 차선 인식 통합 테스트
곡률 주행 launch를 먼저 종료해야 합니다. 두 launch를 동시에 실행하면 /ctrl_cmd와 UDP 포트가 충돌할 수 있습니다.
roslaunch stability_stack morai_udp_ekf_curvature_camera_fallback.launch \
  workspace_path:=/root/AutoVehicle/morai_ws \
  path_file:=/root/AutoVehicle/morai_ws/data/routes/2026_molit_comp_global_path.txt \
  morai_host_ip:=192.168.0.148 \
  enable_control:=false \
  use_camera_team_model:=true \
  show_camera_windows:=true
카메라 토픽을 확인합니다.
rostopic type /camera/front/image/compressed
rostopic hz /detection/lane
rostopic echo -n 1 /detection/lane
rostopic hz /detection/lane_debug/compressed
rostopic hz /detection/lane_model_mask/compressed
정상이라면 다음 두 창이 표시됩니다.
- MORAI Front Camera - Raw
- MORAI Front Camera - Lane Debug
차선 인식과 디버그 영상이 정상인 것을 확인한 뒤 실제 제어를 켭니다.
roslaunch stability_stack morai_udp_ekf_curvature_camera_fallback.launch \
  workspace_path:=/root/AutoVehicle/morai_ws \
  path_file:=/root/AutoVehicle/morai_ws/data/routes/2026_molit_comp_global_path.txt \
  morai_host_ip:=192.168.0.148 \
  enable_control:=true \
  use_camera_team_model:=true \
  show_camera_windows:=true
현재 카메라 통합은 /camera/front/image/compressed 형식을 사용합니다. 실제 카메라 토픽명이 다르면 다음처럼 바꿉니다.
front_camera_topic:=/실제/카메라/토픽
8. 실제 GPS·IMU 노이즈 측정
인위적인 noise injector를 사용하지 않고 MORAI 원본 센서의 변동을 측정하려면 주행 launch를 종료한 뒤 실행합니다.
roslaunch sensor_noise_estimator sensor_noise_measurement.launch \
  csv_path:=/root/AutoVehicle/morai_ws/data/measurements/raw_sensor_noise.csv
결과 확인:
rostopic echo /localization/noise_statistics
rostopic echo /localization/noise_statistics_json
rostopic hz /localization/noise_statistics
실제 MORAI 주행 테스트에서는 다음 두 launch를 우선 사용하면 됩니다.
- 곡선 주행만: morai_udp_ekf_curvature_only.launch
- 카메라 fallback 포함: morai_udp_ekf_curvature_camera_fallback.launch
