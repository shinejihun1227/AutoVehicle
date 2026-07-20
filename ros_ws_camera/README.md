# ROS Workspace Camera

`ros_ws_camera` is a camera-focused MORAI workspace. It is separated from the driving controller workspaces so camera perception can be tested without changing path-following code.

Main packages:

- `yeoms_camera_bridge`: receives MORAI camera UDP packets and publishes ROS compressed images.
- `yeoms_camera_perception`: runs simple Front camera perception for lane offset, traffic-light color candidates, and stop-line candidates.
- `yeoms_camera_bringup`: launch files for camera-only tests.

Competition-based camera layout:

| Camera | Fixed pose | Max resolution | FOV | Default Ubuntu receive port | ROS topic |
| --- | --- | ---: | ---: | ---: | --- |
| Front | `(1.9, 0.0, 1.2), rpy=(0, 2, 0)` | `1280 x 720` | `90 deg` | `1001` | `/sensors/camera/front/compressed` |
| Left | `(1.15, 0.65, 1.2), rpy=(0, 10, 70)` | `640 x 480` | `130 deg` | `1011` | `/sensors/camera/left/compressed` |
| Right | `(1.15, -0.65, 1.2), rpy=(0, 10, 290)` | `640 x 480` | `130 deg` | `1021` | `/sensors/camera/right/compressed` |
| Aux | Rule-range free camera | Rule-range dependent | Rule-range dependent | `1031` | `/sensors/camera/aux/compressed` |

The first implementation is a baseline, not the final contest detector. It verifies image input, records samples, and creates ROS topics that later YOLO/lane/traffic-light models can replace.

See `SENSOR_RULES.md` for the rule-based sensor summary.
