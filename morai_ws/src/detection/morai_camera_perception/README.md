# CAMERA팀 차선 인식 통합

이 패키지는 CAMERA팀의 `lane_segmentation.onnx` 모델을 현재 주행 스택의
차선 메시지와 연결한다. CAMERA팀 저장소의 UDP Pure Pursuit 제어부는 가져오지
않고, 카메라 인식 결과만 기존 주행 코드에 전달한다.

## 데이터 흐름

```text
/camera/front/image/compressed  (sensor_msgs/CompressedImage)
        |
        v
camera_lane_team_node.py
  - cv2.dnn ONNX inference
  - lane line mask 후처리
  - 차선 중심/방향/신뢰도 계산
        |
        +--> /detection/lane (morai_perception_msgs/LaneDetection)
        |       |
        |       +--> camera_localization_fallback_controller
        |              |
        |              +--> control_mux --> /ctrl_cmd
        |
        +--> /detection/lane_debug/compressed
        +--> /detection/lane_model_mask/compressed
```

## 출력 메시지

현재 주행 코드와의 결합을 위해 별도의 새 메시지를 만들지 않고 기존
`morai_perception_msgs/LaneDetection`을 재사용한다.

| 필드 | 의미 |
|---|---|
| `lateral_offset_m` | 차선 중심과 영상 중심의 횡오차(m) |
| `heading_error_rad` | 차선 중심선의 영상 방향 오차(rad) |
| `confidence` | 양쪽 차선·coverage·fit residual·모델 확률을 합친 0~1 품질값 |
| `valid` | 주행용 차선 형상이 유효한지 여부 |

현재 fallback controller의 `lane_sign=1.0`과 호환되도록 출력 부호는 기존
검출기의 부호를 유지한다. 좌회전·우회전에서 반대로 움직이면 launch의
`output_sign`을 바꾸기 전에 영상 오버레이와 `/detection/lane` 값을 먼저 함께
확인한다.

## 모델과 카메라 규칙

- 모델 입력: `640x360`, BGR 영상에 `1/255`, RGB 교환
- 모델 출력: CAMERA팀 export의 `ll` 차선 segmentation을 우선 사용
- 출력 이름이 바뀐 ONNX도 2채널 segmentation tensor를 이용해 보정 탐색
- 전방 카메라 기준 MORAI 설정: 1280x720, 수평 FOV 90도, 장착 위치 `(1.9, 0, 1.2)`,
  pitch 2도
- 모델은 픽셀 차선을 출력하므로 `lane_width_m=3.5`와 화면상의 차선 폭으로
  미터 단위 횡오차를 계산한다. 실제 주행 전에 카메라 calibration으로 재검증한다.

## 실행

먼저 제어를 끄고 영상과 메시지를 확인한다.

```bash
roslaunch stability_stack morai_udp_ekf_curvature_camera_fallback.launch \
  workspace_path:=/root/morai_ws \
  path_file:=/root/morai_ws/data/routes/2026_molit_comp_global_path.txt \
  morai_host_ip:=192.168.0.148 \
  enable_control:=false \
  use_camera_team_model:=true \
  show_camera_windows:=true
```

확인 토픽:

```bash
rostopic type /detection/lane
rostopic hz /detection/lane
rostopic echo -n 1 /detection/lane
rostopic hz /detection/lane_debug/compressed
rostopic hz /detection/lane_model_mask/compressed
```

`MORAI Front Camera - Raw` 창은 원본이고, `MORAI Front Camera - Lane Debug` 창은
모델 mask·차선 경계·중심선과 `confidence`, `offset`, `heading`을 표시한다.

## 기존 검출기와 A/B 비교

기존 색상/Hough 방식은 삭제하지 않고 아래처럼 선택적으로 실행할 수 있다.

```bash
use_camera_team_model:=false
```

두 노드를 동시에 실행하면 둘 다 `/detection/lane`을 발행하므로 비교할 때는
반드시 한 번에 하나만 켠다.
