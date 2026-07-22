# run+camera

MORAI raw-UDP Pure Pursuit with Front-camera integration.

This folder is a standalone fork of the ROI \`dev/stanley\` UDP runtime. It
receives Front camera packets on UDP port \`1101\`, reconstructs MORAI JPEG
frames, runs a conservative OpenCV prototype, and applies a brake override
when red-light and stop-line evidence are both present.

The GPS/IMU Pure Pursuit controller remains the primary controller. Camera
lane steering is disabled by default until it is calibrated on the
competition map.

## Ubuntu prerequisites

The integrated runtime reuses the existing ROI \`path_planning\` package.
Clone/update both repositories and install the camera dependencies:

\`\`\`bash
cd ~
if [ -d ROI/.git ]; then
  cd ROI && git pull origin dev/stanley
else
  git clone -b dev/stanley --single-branch https://github.com/hyunho0429/ROI.git ROI
fi

cd ~
if [ -d AutoVehicle/.git ]; then
  cd AutoVehicle && git pull origin main
else
  git clone https://github.com/shinejihun1227/AutoVehicle.git AutoVehicle
fi

sudo apt update
sudo apt install -y python3-opencv python3-numpy

cd ~/ROI
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
\`\`\`

No new ROS installation or ROS message package is required for this raw-UDP
runner.

## Network defaults

- Front camera: MORAI \`192.168.0.151:1100\` -> Ubuntu \`192.168.0.200:1101\`
- Ego Ctrl Cmd: Ubuntu source \`0.0.0.0:9094\` -> MORAI \`192.168.0.151:9093\`
- GPS: Ubuntu \`3001\`
- IMU: Ubuntu \`4001\`
- Competition status: Ubuntu \`9081\`
- Collision data: Ubuntu \`9092\`

Change the IPs if the VM network differs.

## Run

\`\`\`bash
cd ~/AutoVehicle
source /opt/ros/noetic/setup.bash
source ~/ROI/devel/setup.bash
bash 'run+camera/run_camera.sh'
\`\`\`

The helper defaults to:

- MORAI control IP \`192.168.0.151\`
- Front camera port \`1101\`
- route file \`~/ROI/src/path_planning/data/2026_molit_comp_global_path.txt\`

Override them without editing code:

\`\`\`bash
MORAI_PC_IP=192.168.0.151 \\
CAMERA_PORT=1101 \\
ROI_PATH=~/ROI/src/path_planning/data/2026_molit_comp_global_path.txt \\
bash 'run+camera/run_camera.sh'
\`\`\`

## Camera-only test

Run this before moving the vehicle:

\`\`\`bash
cd ~/AutoVehicle
python3 'run+camera/front_camera_debug.py' \\
  --bind-ip 0.0.0.0 --port 1101 --save-dir /tmp/morai_front
\`\`\`

Complete reconstructed JPEG files are written to \`/tmp/morai_front\`.
Incomplete or corrupt UDP frames are discarded before OpenCV decoding.

