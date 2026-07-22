# camera+lidar

Standalone sensor validation tools for MORAI manual-driving tests.

These programs only receive and inspect sensor UDP packets. They do not
publish vehicle control commands and do not start Pure Pursuit, Stanley, or
any other driving controller. You can drive the ego vehicle manually in
MORAI while running them.

## Contents

- camera_monitor.py: Front-camera UDP reassembly, JPEG decoding, basic
  traffic/stop-line/lane observations, optional display, and JPEG saving.
- front_camera_udp.py: MORAI MOR camera packet parser and JPEG fragment
  reassembler.
- front_camera_perception.py: lightweight OpenCV prototype perception.
- lidar_monitor.py: pure-Python VLP16 packet parser, packet/point rate,
  front-ROI nearest-distance output, and optional CSV saving.
- run_camera_monitor.sh: camera monitor launcher.
- run_lidar_monitor.sh: LiDAR monitor launcher.

## MORAI settings used here

Use the destination IP of the Ubuntu VM.

| Sensor | MORAI source/host | Ubuntu destination/bind |
|---|---:|---:|
| Front camera | 192.168.0.151:1100 | 0.0.0.0:1101 |
| Left camera | 192.168.0.151:1110 | 0.0.0.0:1111 |
| Right camera | 192.168.0.151:1120 | 0.0.0.0:1121 |
| Fourth camera | 192.168.0.151:1130 | 0.0.0.0:1131 |
| VLP16 LiDAR | source port 2000 | 0.0.0.0:2001 |

The camera monitor can test another camera by changing --port, but the
default perception ROIs are tuned for the Front camera.

## Ubuntu installation

The monitor scripts do not require the ROI workspace or a ROS message
package. Install only the camera decoding dependencies:

~~~bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy
~~~

LiDAR monitoring itself uses only the Python standard library.

## Camera test while driving manually

Without a GUI, save one JPEG every 30 completed frames:

~~~bash
cd ~/AutoVehicle
python3 camera+lidar/camera_monitor.py \
  --bind-ip 0.0.0.0 \
  --port 1101 \
  --save-dir /tmp/morai_front
~~~

If the Ubuntu VM has a working OpenCV display:

~~~bash
cd ~/AutoVehicle
python3 camera+lidar/camera_monitor.py \
  --bind-ip 0.0.0.0 \
  --port 1101 \
  --display
~~~

Expected output includes:

~~~text
frames=... fragments=... resolution=1280x720 ...
~~~

If frames stays at zero, check the MORAI destination IP/port and Ubuntu
firewall. If frames arrive but the resolution is wrong, check the camera
resolution in MORAI. The parser drops incomplete or malformed JPEG frames.

## LiDAR test while driving manually

~~~bash
cd ~/AutoVehicle
python3 camera+lidar/lidar_monitor.py \
  --bind-ip 0.0.0.0 \
  --port 2001 \
  --save-dir /tmp/morai_lidar
~~~

Expected output includes:

~~~text
scan=... packets=... points=... front_roi_points=... nearest_front=... m
~~~

CSV point-cloud snapshots are written to /tmp/morai_lidar. The default
front ROI is x=0.5..30 m, abs(y)<=3 m, z=-1.5..2 m. Adjust it if the
LiDAR coordinate orientation is different:

~~~bash
python3 camera+lidar/lidar_monitor.py \
  --port 2001 \
  --invert-y
~~~

Use --invert-z only after confirming the vertical sign from the point
coordinates.

## Two-terminal validation order

1. Start the camera monitor.
2. Start the LiDAR monitor.
3. Start MORAI and manually move/rotate the vehicle.
4. Confirm camera frame counts and saved JPEGs.
5. Confirm LiDAR packet counts, point counts, and nearest-front distance.
6. Only after both streams are stable, connect them to perception or a
   driving controller.

Neither monitor opens UDP control port 9093/9094, so they are safe to run
during manual driving.

## One-command visual launch

The RViz layout is stored in lidar_manual.rviz. To start the LiDAR ROS
bridge and open RViz with the PointCloud2 topic already configured:

~~~bash
cd ~/AutoVehicle
bash camera+lidar/run_lidar_rviz.sh
~~~

To open the camera OpenCV window and LiDAR RViz together:

~~~bash
cd ~/AutoVehicle
bash camera+lidar/run_sensor_viewers.sh
~~~

Close RViz or press Ctrl+C to stop the bridge and camera monitor. The first
run may build ros_ws_lidar automatically if its devel/setup.bash is missing.
