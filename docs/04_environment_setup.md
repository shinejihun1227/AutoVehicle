# Environment Setup Plan

## 1. Target Environment

대회 최종 실행 환경은 다음 기준으로 고정한다.

```text
OS: Ubuntu 20.04
ROS: ROS1 Noetic
PC type: Desktop PC
Simulator connection: LAN via switching hub
```

## 1.1 Current Development Environment

현재 개발 환경은 Windows host에서 MORAI SIM을 실행하고, VM 내부의 Ubuntu 20.04 + ROS1 Noetic을 사용한다.

이 구성은 초기 학습과 개발에는 유용하지만, 대회 규정상 최종 실행 환경은 native Ubuntu 20.04 desktop PC로 이전하는 것을 목표로 한다. 특히 VM network mode에 따라 UDP/ROS 통신 지연, 브로드캐스트, 포트 포워딩 문제가 달라질 수 있으므로, MORAI 통신을 붙이기 전 VM network mode를 반드시 기록한다.

## 2. Required ROS Packages

초기 후보:

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo apt install ros-noetic-rosbridge-server
sudo apt install ros-noetic-velodyne
sudo apt install ros-noetic-cv-bridge
sudo apt install ros-noetic-image-transport
sudo apt install ros-noetic-tf2-ros
sudo apt install ros-noetic-diagnostic-updater
```

실제 설치 목록은 대회 제공 예제 패키지와 MORAI 공식 가이드의 요구사항을 확인한 뒤 고정한다.

## 3. Catkin Workspace

```bash
mkdir -p ~/morai_competition_ws/src
cd ~/morai_competition_ws
catkin_make
source devel/setup.bash
```

## 4. Bash Settings Draft

```bash
source /opt/ros/noetic/setup.bash
source ~/morai_competition_ws/devel/setup.bash

alias cw='cd ~/morai_competition_ws'
alias cs='cd ~/morai_competition_ws/src'
alias cm='cd ~/morai_competition_ws && catkin_make'
alias rl='rostopic list'
alias rgp='rqt_graph'
alias morai_run='roslaunch morai_competition_bringup competition.launch'
```

## 5. Network Preparation

대회 당일 multi-system 구조를 고려해 다음을 기록한다.

```text
Server PC IP:
Client PC IP:
Team PC IP:
ROS_MASTER_URI:
ROS_IP:
UDP control target IP:
UDP control target port:
Sensor receive ports:
```

## 6. Pre-run Check

대회 실행 전 확인할 것:

- Ubuntu 20.04 부팅 확인
- ROS Noetic source 확인
- LAN 연결 확인
- MORAI sensor setting file load 확인
- rosbridge 또는 UDP bridge 실행 확인
- 각 센서 데이터 주기 확인
- `/control/ctrl_cmd` 생성 확인
- UDP control packet 송신 확인
- logging 경로 확인
