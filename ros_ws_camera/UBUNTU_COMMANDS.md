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

Run camera UDP receiver only:

```bash
roslaunch yeoms_camera_bringup camera_udp_only.launch camera_port:=1001
```

Run camera receiver and baseline perception:

```bash
roslaunch yeoms_camera_bringup camera_perception.launch camera_port:=1001
```

Useful topics:

```bash
rostopic hz /sensors/camera/front/compressed
rostopic echo /perception/camera/frame_info
rostopic echo /perception/camera/lane_offset_px
rostopic echo /perception/camera/traffic_light_state
rostopic echo /perception/camera/stop_line_detected
```

Install OpenCV if perception node reports that `cv2` is missing:

```bash
sudo apt update
sudo apt install -y python3-opencv
```
