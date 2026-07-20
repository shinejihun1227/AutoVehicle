# Ubuntu Commands For `ros_ws_camera`

Update and copy the camera packages:

```bash
cd ~/AutoVehicle
git pull

cd ~/morai_stanley_ws

rm -rf src/yeoms_camera_bridge
rm -rf src/yeoms_camera_perception
rm -rf src/yeoms_camera_bringup

cp -r ~/AutoVehicle/ros_ws_camera/src/yeoms_camera_bridge src/
cp -r ~/AutoVehicle/ros_ws_camera/src/yeoms_camera_perception src/
cp -r ~/AutoVehicle/ros_ws_camera/src/yeoms_camera_bringup src/

chmod +x src/yeoms_camera_bridge/scripts/*.py
chmod +x src/yeoms_camera_perception/scripts/*.py

source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
rospack profile
```

MORAI camera UDP destination ports used by this repo:

```text
Front camera -> Ubuntu IP, destination port 1001
Left camera  -> Ubuntu IP, destination port 1011
Right camera -> Ubuntu IP, destination port 1021
Aux camera   -> Ubuntu IP, destination port 1031
```

Run Front camera UDP receiver only:

```bash
roslaunch yeoms_camera_bringup camera_udp_only.launch camera_port:=1001
```

Run Front/Left/Right camera UDP receivers:

```bash
roslaunch yeoms_camera_bringup multi_camera_udp_only.launch \
  front_port:=1001 \
  left_port:=1011 \
  right_port:=1021
```

Run Front camera receiver and baseline perception:

```bash
roslaunch yeoms_camera_bringup camera_perception.launch camera_port:=1001
```

Useful Front perception topics:

```bash
rostopic hz /sensors/camera/front/compressed
rostopic echo /perception/camera/front/frame_info
rostopic echo /perception/camera/front/lane_offset_px
rostopic echo /perception/camera/front/traffic_light_state
rostopic echo /perception/camera/front/stop_line_detected
```

Useful multi-camera image topics:

```bash
rostopic hz /sensors/camera/front/compressed
rostopic hz /sensors/camera/left/compressed
rostopic hz /sensors/camera/right/compressed
```

Install OpenCV if perception node reports that `cv2` is missing:

```bash
sudo apt update
sudo apt install -y python3-opencv
```
