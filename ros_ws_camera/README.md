# ROS Workspace Camera

`ros_ws_camera` is a camera-focused MORAI workspace. It is intentionally separated from the driving controller workspaces so camera perception can be tested without changing path-following code.

Main packages:

- `yeoms_camera_bridge`: receives MORAI camera UDP packets and publishes ROS compressed images.
- `yeoms_camera_perception`: runs simple camera perception for lane offset and traffic-light color candidates.
- `yeoms_camera_bringup`: launch files for camera-only tests.

Default MORAI camera setting:

- MORAI camera source port: `1000`
- Ubuntu algorithm receive port: `1001`
- ROS image topic: `/sensors/camera/front/compressed`

The first implementation is a baseline, not the final contest detector. It is meant to verify image input, record samples, and create ROS topics that later YOLO/lane/traffic-light models can replace.
