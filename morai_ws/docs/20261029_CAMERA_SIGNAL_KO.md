후속 수정: 녹색 우회전 우선 처리는 [20261030_RIGHT_TUNNEL_KO.md](20261030_RIGHT_TUNNEL_KO.md)를 따른다.

# CAM1·혼합 신호·4번째 우회전·차선 보정 수정

대상 브랜치: `new_final_ws`. 사용자 사진의 증상을 기준으로 수정했다.
실제 MORAI 영상 수신, 모델 로딩 및 주행은 Ubuntu에서 확인해야 한다.

## 동작

- CAM1: 첫 영상을 15초 기다린 뒤 종료하던 동작을 계속 대기하도록 수정.
  UDP 수신 오류는 로그로 알리고 노드를 3초 뒤 재실행한다. 모델 실행 실패도
  재실행하지만 파일/의존성 오류 자체는 로그에 나온 원인을 해결해야 한다.
- CAM4: 기존의 방향 화살표 조합은 유지. 모순된 검출은 현재 경로에 연결된
  신호 후보만 대상으로 신뢰도 65% + 지도 투영 일치도 35%로 순위를 정한다.
  다른 도로에 더 가까운 후보는 제외하며, 서로 다른 신호의 점수 차가 0.025
  미만이면 UNKNOWN을 유지한다. 기존의 시간 확인도 유지한다.
  `no_projected_targets`는 보이는 신호와 지도 위치를 연결하지 못한 상태이므로
  화면 전체 최고 신뢰도로 우회하지 않는다. 위치·CAM4 장착 설정 확인이 필요하다.
- 4번째 우회전 `mgeo:A2256W000318:0`: 현재 설정된 앞범퍼 정차 목표 근처에서
  속도 0.15m/s 이하를 연속 1초 확인한 후 신호색과 관계없이 우회전을 허용.
  정차 목표에서 0.75m보다 멀리 떨어진 정차는 인정하지 않는다.
  교차로별 상태이므로 다음 교차로의 좌회전에는 적용되지 않는다.
  위치 이상·상태 속도 단절·상위 명령 단절 및 연결된 장애물 정지는 계속 적용된다.
  이 주행 모드 자체에 새 장애물/보행자 인지 기능을 추가한 것은 아니다.
- 차선 보정: 기본 ON. 가중치 0.45, 보정 최대 0.065rad, 필터 0.25초,
  증가율 0.10rad/s. 양쪽 실제 도색 관측·신뢰도·연속 프레임 검사를 유지하고,
  폭 2.4~4.2m의 중앙선을 사용한다. 가짜 유도선·추측 경계는 사용하지 않는다.

## 기존 Docker 설치 업데이트

기존 monitor/drive 터미널을 Ctrl+C로 종료한 뒤 Ubuntu 호스트에서 실행한다.
저장한 IP·Competition 수신 포트·속도 설정은 그대로 사용한다.

```bash
cd ~/AutoVehicle_new_final_ws
git pull --ff-only origin new_final_ws
cd morai_ws/docker/final_ws
bash run_highway.sh stop
bash install_curvature_signal.sh --config
bash run_highway.sh start
bash run_test.sh lane on
bash run_test.sh show 2
bash run_test.sh drive 2
```

`--config`는 저장소의 신호 설정을 설치한다. 기존 설정은 설치기가 백업한다.
`lane on`은 이전 로컬 env의 OFF 및 약한 보정값을 새 프리셋으로 바꾼다.
`show 2`에서 `Lane_centering=true weight=0.45 max_delta_rad=0.065`를 확인한다.
주행 시작 시 `RUNTIME_MATCH`, `SIGNAL_CONFIG_MATCH`가 표시되어야 한다.

CAM1 화면이 여전히 비면 별도 호스트 터미널에서 다음 출력을 확인한다.

```bash
cd ~/AutoVehicle_new_final_ws/morai_ws/docker/final_ws
bash run_test.sh diagnose 2
```

MORAI의 CAM1 송신 목적지는 Ubuntu `192.168.10.21:1101`이어야 한다.
다른 런치의 CAM1 수신기가 같은 포트를 동시에 점유하는지도 확인한다.
카메라 미수신은 코드가 영상을 만들어 해결할 수 있는 문제가 아니다.

## 검증

기존 패킷·위치 추정·제어 검사와 혼합 신호 선택, 우회전 정차 시간/거리,
장애물 정지 유지, 좁은 차선 관측 검사를 수행했다. 실제 MORAI 주행 검증은 별도다.
