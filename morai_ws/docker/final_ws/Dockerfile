# Fresh official ROS base: no project, models, MORAI SDK, or user configuration.
# Build context MUST be morai_ws, not the repository root.
ARG ROS_BASE=ros:noetic-ros-base-focal
FROM ${ROS_BASE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG TORCH_FLAVOR=cpu
ARG MORAI_MSGS_REF
ARG CODE_REVISION=unrecorded
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8 PYTHONUNBUFFERED=1 \
    MORAI_WS=/opt/AutoVehicle/morai_ws \
    VIRTUAL_ENV=/opt/morai-venv \
    ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 \
    YOLO_AUTOINSTALL=false QT_X11_NO_MITSHM=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    git ca-certificates curl build-essential cmake \
    python3-dev python3-pip python3-venv python3-catkin-pkg python3-rospkg \
    python3-empy python3-yaml \
    python3-rosdep python3-nose python3-numpy python3-scipy python3-opencv \
    ros-noetic-catkin ros-noetic-rospy ros-noetic-message-generation \
    ros-noetic-message-runtime ros-noetic-tf ros-noetic-tf2-ros \
    ros-noetic-nav-msgs ros-noetic-sensor-msgs ros-noetic-geometry-msgs \
    ros-noetic-diagnostic-msgs ros-noetic-visualization-msgs \
    ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-rviz \
    iproute2 iputils-ping tcpdump xvfb xauth libgl1 libglib2.0-0 \
    libsm6 libxext6 libxrender1 libxkbcommon-x11-0 libxcb-xinerama0 \
    && rm -rf /var/lib/apt/lists/*

# Preserve ROS's apt Python modules; use the SAME venv for catkin and runtime.
RUN python3 -m venv --system-site-packages /opt/morai-venv
ENV PATH=/opt/morai-venv/bin:${PATH}
COPY docker/final_ws/requirements.txt /opt/morai-build/requirements.txt
RUN python -m pip install --no-cache-dir pip==24.3.1 setuptools==69.5.1 wheel==0.45.1 \
    && [[ "${TORCH_FLAVOR}" == cpu || "${TORCH_FLAVOR}" == cu121 ]] \
    && python -m pip install --no-cache-dir torch==2.4.1 torchvision==0.19.1 \
       --index-url "https://download.pytorch.org/whl/${TORCH_FLAVOR}" \
    && python -m pip install --no-cache-dir -r /opt/morai-build/requirements.txt \
    && python -m pip check

WORKDIR /opt/AutoVehicle/morai_ws
COPY src ./src
COPY config ./config
COPY data ./data
COPY docker/final_ws ./docker/final_ws

# Explicit SHA avoids an unnoticed change of the external ROS1 message API.
# Check this revision against the competition SDK before using the simulator.
RUN [[ "${MORAI_MSGS_REF}" =~ ^[0-9a-fA-F]{40}$ ]] \
    && git clone https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs.git src/common/morai_msgs \
    && git -C src/common/morai_msgs checkout --detach "${MORAI_MSGS_REF}" \
    && test -f src/common/morai_msgs/msg/CtrlCmd.msg \
    && test -f src/common/morai_msgs/msg/GPSMessage.msg \
    && test -f src/common/morai_msgs/msg/EgoVehicleStatus.msg

RUN source /opt/ros/noetic/setup.bash \
    && catkin_make -j2 -l2 -DPYTHON_EXECUTABLE=/opt/morai-venv/bin/python3 \
    && source devel/setup.bash \
    && python -c 'import rospy, cv2, torch, torchvision, ultralytics; from morai_msgs.msg import CtrlCmd, GPSMessage, EgoVehicleStatus; from morai_perception_msgs.msg import StopLineDetection; from common.msg import ObjectInfoArray' \
    && python -m pip freeze > /opt/morai-build/pip-freeze.txt \
    && dpkg-query -W > /opt/morai-build/apt-packages.txt \
    && printf '%s\n' "${CODE_REVISION}" > /opt/morai-build/code-revision.txt \
    && git -C src/common/morai_msgs rev-parse HEAD > /opt/morai-build/morai-msgs-revision.txt

LABEL org.opencontainers.image.revision=${CODE_REVISION}
COPY docker/final_ws/entrypoint.sh /usr/local/bin/morai-entrypoint
RUN chmod 755 /usr/local/bin/morai-entrypoint
ENTRYPOINT ["/usr/local/bin/morai-entrypoint"]
# Never start sensors or vehicle control just by starting a container.
CMD ["bash"]
