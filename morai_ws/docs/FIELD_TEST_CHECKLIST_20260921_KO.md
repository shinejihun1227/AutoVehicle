# MORAI 주행 테스트 — 5가지 실행 명령

**모든 명령은 Ubuntu 일반 터미널에서 실행한다. 스크립트가 Docker 안의 ROS를 실행한다.**
Ubuntu `192.168.0.185` / MORAI `192.168.0.147` 기준이다.

이번 업데이트를 처음 받는다면 맨 아래 **업데이트 적용**을 먼저 진행한다. 이후에는 다음 두 명령으로 시작한다.

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
```

```bash
bash run_highway.sh start
```

**한 번에 하나만 실행한다. 시험을 바꿀 때는 MORAI에서 차량을 정지시킨 뒤 실행 터미널에서 `Ctrl+C`를 누른다.**

## 1. 곡률 기반 주행

```bash
bash run_test.sh drive 1
```

직선 가속 → 커브 감속 → 커브 탈출 후 재가속을 확인한다. 신호 정지·장애물 회피는 꺼져 있다.

## 2. 곡률 주행 + 정지선·신호등

```bash
bash run_test.sh drive 2
```

**새 Ubuntu 터미널에서 CAM1·CAM4 인식 화면을 RViz로 연다.**

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws" && bash run_test.sh rviz
```

브라우저로 보려면 `http://127.0.0.1:8765`를 연다. 브라우저에는 정지 사유도 표시된다.
영상이 안 나오거나 이전 버전이라면 [카메라 화면 업데이트·실행 안내](CAMERA_VISUAL_TEST_KO.md)를 따른다.

적색 신호에서 정지선 앞 정지 → 허용 신호에서 재출발을 확인한다.
Cam4 보정이 완료되어야 한다. `signal_camera_uncalibrated`로 멈추면 [보정 안내](CURVATURE_SIGNAL_ONLY_KO.md)를 따른다.

출발하지 않으면 새 터미널에서 `bash run_test.sh diagnose 2`를 실행한다. [진단·업데이트 안내](CAMERA_VISUAL_TEST_KO.md).

## 3. 곡률 주행 + 정지선·신호등 + 장애물 회피

```bash
bash run_test.sh drive 3
```

정적 장애물 우회 → 원래 경로 복귀를 확인한다. 요청에 의한 끼어들기는 꺼져 있다.

## 4. 곡률 주행 + 정지선·신호등 + 끼어들기

```bash
bash run_test.sh drive 4
```

아래 **끼어들기 요청**을 함께 실행한다. 자동 장애물 우회는 꺼져 있으며, 장애물이 현재 경로를 막으면 정지한다.

## 5. 곡률 주행 + 정지선·신호등 + 장애물 회피 + 끼어들기

```bash
bash run_test.sh drive 5
```

장애물 우회와 요청에 의한 끼어들기를 모두 켠다. 끼어들기는 아래 요청 명령이 필요하다.

### 끼어들기 요청 — 4·5번 공통

끼어들기는 **우리 차량의 왼쪽 차로 변경**이다. 주행 터미널을 켜 둔 채 **새 Ubuntu 터미널**에서 실행한다.

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws" && bash run_test.sh request-merge
```

왼쪽 점선·목표 차로 경계·앞뒤 간격·충돌 예상시간 조건이 충족되면 진행한다. 요청 터미널은 차로 변경이 끝날 때까지 유지하고, 완료 후 `Ctrl+C`로 종료한다. 기본값은 실행당 1회다.

**신호 제어 범위:** 2번은 경로 방향에 맞춘 신호 판단을 사용한다. 3~5번은 기존 통합 주행의 일반 신호·정지선 제어를 사용하므로, 2번의 방향별 신호 판단까지 통합된 구성은 아니다. 3~5번의 신호 시험은 직진 적색·녹색부터 진행한다.

## 속도 조절 — 다섯 시험 공통

아래 `10`을 원하는 **최고속도(km/h)**로 바꾼다.

```bash
bash run_test.sh speed 10
```

`highway-test.env`의 `MAX_SPEED_KPH`가 저장된다. **실행 중인 주행을 종료하고 원하는 번호로 다시 실행해야 적용된다.** 실제 속도는 곡률·선행차·신호 정지 조건에 따라 낮아질 수 있다. 커브 속도는 같은 파일의 `LATERAL_ACCEL_LIMIT_MPS2`, 가속 정도는 `MAX_ACCEL_MPS2`가 결정한다.

차량 제어를 보내지 않고 확인하려면 `drive` 대신 `monitor`를 쓴다. 예: `bash run_test.sh monitor 2`.

<details>
<summary><strong>업데이트 적용 — 이번 버전으로 바꿀 때 한 번</strong></summary>

기존 주행을 종료한 뒤, 아래 블록을 하나씩 실행한다. 실패한 단계가 있으면 다음 단계로 넘어가지 않는다.

```bash
cd "$HOME/AutoVehicle" && git pull --ff-only origin final_ws
```

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
```

설정 파일이 없을 때만 만든다.

```bash
[ -f highway.env ] || cp highway.env.example highway.env
```

```bash
[ -f highway-test.env ] || cp highway-test.env.example highway-test.env
```

현재 IP를 지정한다. `CONTAINER_NAME`은 실제 사용하는 컨테이너 이름을 유지한다.

```bash
sed -i.bak -e 's/^UBUNTU_IP=.*/UBUNTU_IP=192.168.0.185/' -e 's/^MORAI_IP=.*/MORAI_IP=192.168.0.147/' highway.env
```

호스트에서 받은 코드를 기존 컨테이너에도 복사한다. 기존 파일은 백업하며 신호 카메라 보정값은 보존한다.

```bash
bash run_highway.sh stop && bash install_avoidance_transport.sh && bash install_curvature_signal.sh && bash run_highway.sh start
```

```bash
source highway.env && sudo docker --context default exec "$CONTAINER_NAME" printenv ROS_IP ROS_MASTER_URI
```

출력은 `192.168.0.185`, `http://192.168.0.185:11311`이어야 한다. 다른 주소이면 [컨테이너 네트워크 갱신 안내](TWO_PC_DOCKER_HIGHWAY_KO.md)를 먼저 따른다.

첫 시험 최고속도를 5 km/h로 맞춘다.

```bash
bash run_test.sh speed 5
```

</details>

센서 설정·보정·정지 원인·로그 수집은 [상세 참고 문서](FIELD_TEST_DETAILED_REFERENCE_KO.md)에 모았다. 위 실행 구성은 오프라인 검사 대상이며, MORAI 주행 검증은 별도로 필요하다.
