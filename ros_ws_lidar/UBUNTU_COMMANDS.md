# Ubuntu Commands For `ros_ws_lidar`

Update and copy the LiDAR packages:

```bash
cd ~/AutoVehicle
git pull

cd ~/morai_stanley_ws

rm -rf src/yeoms_lidar_bridge
rm -rf src/yeoms_lidar_perception
rm -rf src/yeoms_lidar_bringup

cp -r ~/AutoVehicle/ros_ws_lidar/src/yeoms_lidar_bridge src/
cp -r ~/AutoVehicle/ros_ws_lidar/src/yeoms_lidar_perception src/
cp -r ~/AutoVehicle/ros_ws_lidar/src/yeoms_lidar_bringup src/

chmod +x src/yeoms_lidar_bridge/scripts/*.py
chmod +x src/yeoms_lidar_perception/scripts/*.py

source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
rospack profile
```

MORAI LiDAR UDP destination:

```text
VLP16 LiDAR -> Ubuntu IP, destination port 2001
```

Run LiDAR UDP receiver only:

```bash
roslaunch yeoms_lidar_bringup lidar_udp_only.launch lidar_port:=2001
```

Run LiDAR receiver and baseline obstacle perception:

```bash
roslaunch yeoms_lidar_bringup lidar_obstacle_perception.launch lidar_port:=2001
```

Useful topics:

```bash
rostopic hz /sensors/lidar/points
rostopic echo /perception/lidar/packet_info
rostopic echo /perception/lidar/nearest_obstacle_m
rostopic echo /perception/lidar/obstacle_stop_required
rostopic echo /perception/lidar/obstacle_summary
```

If RViz is available, visualize:

```bash
rviz
```

Then add `PointCloud2` displays for:

```text
/sensors/lidar/points
/perception/lidar/roi_points
```
