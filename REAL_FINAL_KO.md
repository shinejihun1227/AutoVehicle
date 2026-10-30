# real_final 실행과 변경 내용

기반: `whddbs3061/car`, `test/roundabout-highway`, `4259fc184149bc9b3c53f6b929e8a21754febeee`.
배포: `shinejihun1227/AutoVehicle`, `real_final`. 기존 브랜치는 수정하지 않습니다.
이 브랜치에는 앞서 전달한 10개 소스의 신호·4번째 우회전·GPS 음영 수정이 포함돼 있습니다.
별도로 그 ZIP을 다시 덮어쓸 필요가 없습니다.

## 변경한 동작

- 신호: 같은 교차로의 허용 신호로 진입한 뒤 신호 소실 때문에 재정지하지 않습니다.
  신호 신뢰도는 0.6 초과만 사용합니다. 현재 경로와 다른 신호를 화면 전체 최고값으로 고르지 않습니다.
- 4번째 우회전: 유효한 경로 녹색이면 즉시 진행, 적색이면 목표 정차 후 짧은 정차 확인을 거쳐 진행합니다.
- 위치: Competition 전방 속도와 GPS/IMU를 사용합니다. 초기 GPS 위치를 얻은 뒤 음영 구간은
  IMU 자세·각속도와 Competition 속도로 예측합니다. 일반 Ego의 위치/가속도 슬롯은 사용하지 않습니다.
- 차선: 두 도색 경계가 실제로 관측돼야 합니다. 7~14m 범위가 부족하면 관측된 5~10m, 4~8m를
  차례로 사용합니다. 같은 범위의 MGeo 곡률을 보상해 커브를 차로 오차로 잘못 보정하지 않습니다.
  유도선·예측만 있는 경계·잘못된 폭·교차 차로·오래된 관측은 계속 제외합니다.
  보정 가중치 0.45→0.55, 최대 보정 0.065→0.075rad, 신뢰도 0.60 이상,
  안정 프레임 3개/최소 0.2초, 필터 0.20초, 증가율 0.14rad/s로 조정했습니다.
  CUDA가 없는 PC의 강제 cuda 설정은 CPU로 전환하고 경고합니다. 기본 장치는 auto입니다.
- 속도: 일반 구간 상한 35→42km/h, 횡가속도 설정 1.0→2.25m/s²입니다.
  `v=sqrt(a/|곡률|)` 기준으로 같은 곡률에서 14→21km/h 목표가 됩니다.
  모든 곡선의 실측 속도에 정확히 7을 더하는 최저속도 기능은 아닙니다. 더 급한 곡선과
  정지선·신호·장애물·회전교차로 대기는 여전히 감속/정지합니다. 가속·공간 제동 제한은 유지합니다.
  경로 제어기와 신호 제어기의 회전 속도 한도를 같은 곡률 설정으로 연결했습니다.
- 고속도로: escape4 차선 변경/명령 선택기 연결을 보존합니다. 순항 목표 54→61km/h,
  최고 79.2km/h와 고속도로 끝 28km/h는 유지합니다. 이 구간의 의도적인 차선 변경에는
  일반 구간의 추가 중앙 보정을 억지로 적용하지 않습니다.
- 설치: ZIP의 600 권한이 sudo Docker 복사 후 검증을 막지 않도록 복사할 공개 소스 권한을 정규화합니다.

## Ubuntu에서 새 폴더로 받기

먼저 기존 주행을 Ctrl+C로 종료하고, 이전 폴더의 `bash run_highway.sh stop`으로
이전 컨테이너를 정지합니다. 여러 런치를 동시에 실행하지 않습니다.

```bash
git clone --single-branch --branch real_final https://github.com/shinejihun1227/AutoVehicle.git ~/AutoVehicle_real_final &&
cd ~/AutoVehicle_real_final &&
bash RUN_REAL_FINAL.sh setup &&
bash RUN_REAL_FINAL.sh tune &&
bash RUN_REAL_FINAL.sh build &&
bash RUN_REAL_FINAL.sh install &&
bash RUN_REAL_FINAL.sh show &&
bash RUN_REAL_FINAL.sh drive
```

build는 처음 한 번 필요합니다. 완료돼야 이후 명령을 실행합니다. 기본 CPU 이미지 이름은
`morai-real-final:latest`, 개인 시험 컨테이너는 `morai-real-final-net10`입니다.
이미지가 준비되면 이후에는 `bash RUN_REAL_FINAL.sh drive`만 실행합니다.
관측만 할 때는 `bash RUN_REAL_FINAL.sh monitor`이며 이때 차량 제어는 보내지 않습니다.
카메라 대시보드는 Ubuntu 브라우저 `http://127.0.0.1:8765`입니다.

설정은 `morai_ws/docker/final_ws/highway.env`(IP/이미지/컨테이너)와
`highway-test.env`(속도/포트/차선/정차 여유)에 저장됩니다. setup은 이미 존재하는 파일을 덮지 않습니다.
이전 정차 여유·센서 포트를 유지하려면 setup 후 기존 highway-test.env를 별도로 백업/복사하고 tune을 실행합니다.
tune은 정차 여유·포트·고속도로/회전교차로 스위치를 유지하고 속도/차선 튜닝만 갱신합니다.
추가 소스 수정은 build 또는 install로 컨테이너에 반영해야 합니다.

## 속도 변경과 차선 확인

실행 중인 주행을 Ctrl+C로 종료한 뒤:

```bash
cd ~/AutoVehicle_real_final &&
bash RUN_REAL_FINAL.sh speed 42 &&
bash RUN_REAL_FINAL.sh drive
```

42를 바꾸면 일반 구간 상한이 바뀝니다. profile 2는 55km/h 이하입니다.
고속도로는 런치의 `highway_cruise_kph`, `highway_max_speed_kph`가 별도입니다.
속도/차선 튜닝을 초기 real_final 값으로 돌리려면 `bash RUN_REAL_FINAL.sh tune`을 실행합니다.
실행 중 별도 터미널에서 `bash RUN_REAL_FINAL.sh diagnose`로 차선 상태를 확인합니다.
`lane_centering`의 active, reason, correction_rad, observation_window_m가 실제 적용 상태입니다.
영상이 보이는 것과 보정이 활성화된 것은 다릅니다. 품질이 부족하면 reason에 거부 사유가 나옵니다.

## 대회 네트워크 전환

보관된 공식 통신 매뉴얼(2026-10-27 확인)의 LAN은 아래와 같습니다.
Windows `192.168.0.1`, Ubuntu `192.168.0.10`, mask `255.255.255.0`, 직결망 gateway 생략.
근거: https://morai.atlassian.net/wiki/external/Y2Y1YmJhN2EwNDBjNGYwZDgyNmU5OTMyNTQyZTI2MTM
현재 외부 페이지 재접근은 실패했으므로 새 운영 공지가 있으면 그 안내를 우선 대조합니다.

1. 기존 주행을 Ctrl+C로 종료한 뒤 `bash RUN_REAL_FINAL.sh stop`을 실행합니다.
2. Ubuntu 설정 → 네트워크 → 유선 설정 → IPv4 수동에서 192.168.0.10/255.255.255.0을 설정합니다.
   Windows 직결 Ethernet IPv4는 192.168.0.1/255.255.255.0으로 설정합니다.
3. 저장소 루트에서 아래를 실행합니다.

```bash
bash RUN_REAL_FINAL.sh network competition &&
bash RUN_REAL_FINAL.sh install &&
bash RUN_REAL_FINAL.sh show
```

network 명령은 두 IP와 새 CONTAINER_NAME만 저장하고, 기존 highway.env를 백업합니다.
IMAGE_NAME, 센서/Competition/제어 포트, OS 네트워크 설정은 바꾸지 않습니다.
새 컨테이너에 설치하므로 로컬에서 받았던 소스가 반영됩니다. 확인 후 `bash RUN_REAL_FINAL.sh drive`를 실행합니다.
개인 네트워크로 돌아올 때도 먼저 stop 후 `network personal`, `install`을 실행합니다.

직접 수정하는 nano 경로:

```bash
nano ~/AutoVehicle_real_final/morai_ws/docker/final_ws/highway.env
nano ~/AutoVehicle_real_final/morai_ws/docker/final_ws/highway-test.env
```

대회 highway.env의 두 주소는 `UBUNTU_IP=192.168.0.10`, `MORAI_IP=192.168.0.1`입니다.
수동으로 IP를 바꾸면 사용하지 않은 새 CONTAINER_NAME도 지정합니다. IMAGE_NAME은 유지합니다.
nano 저장 Ctrl+O → Enter, 종료 Ctrl+X.

Windows 센서 Host는 192.168.0.1, Destination은 192.168.0.10입니다.
현재 수신 포트는 CAM1 1101, CAM4 1131, GPS 3001, IMU 4001, LiDAR 2001입니다.
UDP 제어는 Ubuntu 9094 → Windows 9093입니다. 실제 센서 JSON/Network Settings와 일치시킵니다.

Competition 수신은 현재 car와 같은 Ubuntu `COMPETITION_STATUS_PORT=9081`을 유지합니다.
공식 표의 Windows loopback `127.0.0.1:9088 → 127.0.0.1:9099`는 판정 프로그램 입력 경로입니다.
이를 Ubuntu 포트 9099로 자동 간주하지 않습니다. 주최측이 Ubuntu로 전달하는 목적지 포트를 확인해
highway-test.env의 COMPETITION_STATUS_PORT와 맞춥니다. 주소 변경만으로 패킷 호환·전체 규정 준수가
확인되는 것은 아닙니다. 일반 Ego 상태를 대신 켜지 않습니다.

## 검증 범위

REAL_FINAL_TEST_RESULTS.json에 오프라인 검증 결과를 보관합니다. 짧은 실제 관측 범위 선택,
곡률 보상, 21km/h 단순 자전거 모델 중앙 복귀, 신호 정지와 제동, Competition 입력,
고속도로/회전교차로 연결과 환경 보존을 확인합니다.
카메라 추론 성능·MORAI 차량 응답·전체 코스 실측 속도·본선 전달 패킷은 아직 실주행 검증 전입니다.
