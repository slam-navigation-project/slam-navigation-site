# SLAM + NPZ 기반 단일 목적지 A* 관제 시스템

이 수정본은 **기존 웹 관제 시스템**(`amr_server.py` + `index.html`)을 기준으로 통합했습니다. 예전에 사용한 `amr_control_program(7).py`(원격 PC용 Tkinter GUI)는 수정 대상이 아니며, 동일한 ROS/모터를 동시에 제어하지 않도록 주의해야 합니다.

## 주요 변경 사항

- 마우스 왼쪽 클릭 **한 번**으로 목적지 1개 선택. 이전 여러 웨이포인트 누적/왕복 대신 선택한 목적지 하나를 사용합니다.
- 저장된 **NPZ 격자 지도**에서 A* 경로 계산. 초기 로봇 전방을 NPZ의 **+Y(화면 위)**로 정렬합니다.
- NPZ의 `origin_x`, `origin_y`, `origin_yaw`, `resolution`을 이용하여 **실시간 `map → base_footprint` 위치와 지도 셀**을 정확히 대응합니다.
- 클릭 시 서버의 `/api/plan`에서 **사용자가 만든 `astar.py`의 4방향 A***로 즉시 경로를 계산하고 웹 지도에 파란색으로 표시합니다. 실제 주행은 `주행 시작`을 눌러야만 시작됩니다.
- 회색 미탐색 구역, 검은 장애물, 차체 반경에 해당하는 분홍색 팽창 영역은 목적지로 선택할 수 없습니다.
- 주행 버튼을 누르면 현재 로봇 자세에서 **동일한 NPZ / A***로 경로를 다시 검증하고, 기존 `path_follower.py`를 통해 `/cmd_vel`을 발행합니다. 지금은 **단일 목적지 도착 시 정지**합니다.
- 맵 저장 시 `NPZ`, `PGM`, `YAML`, `DATA`, `posegraph` 5개를 저장하고 같은 이름의 기존 파일을 갱신합니다. 새 파일이 누락되면 기존 파일을 덮어쓰지 않습니다.
- 기존 브라우저 기반 실시간 /map 표시, 라이다 레이더/차시점, 수동조종, 로봇 자세·주행 상태 표시 기능을 유지합니다.

## 라즈베리파이 배치

아래 **8개 파일을 모두 같은 폴더**에 복사해야 합니다. 기존 파일은 먼저 백업하세요.

```text
/home/yun/ros2_amr_ws/src/riding_test/
├── amr_server.py             # 웹 서버, SLAM, 지도 저장, 경로 미리보기 API
├── index.html                # 목적지 1회 클릭 + A* 경로 시각화
├── npz_grid.py               # 초기 전방 정렬 NPZ 저장 및 웹 맵 로드 (신규)
├── astar.py                  # 기존 4방향 A* 알고리즘
├── astar_planner.py          # NPZ 좌표 변환·장애물 팽창·A* 호출
├── waypoint_nav_node.py     # 현재 위치 → 단일 목표 경로 추종
├── path_follower.py         # 기존 경로 추종기
└── map_geometry.py          # JSON 명령/상태 도우미
```

`astar_gui.py`, `map_loader.py`, `path_visualization.py`는 이 **웹 관제 화면을 실행하는 데 필요하지 않습니다.** 독립 테스트용으로 별도 보관해도 됩니다.

## 실행

기존 관제/모터 제어 프로세스 중복 실행을 먼저 정리한 다음, Pi에서 ROS2 환경을 설정합니다.

```bash
cd ~/ros2_amr_ws/src/riding_test
source /opt/ros/humble/setup.bash
source ~/ros2_amr_ws/install/setup.bash
python3 -c "import numpy, yaml, rclpy; print('dependencies OK')"
python3 amr_server.py
```

기본 지도 저장 폴더는 `~/ros2_amr_ws/maps/`, 기본 서버 포트는 `8080`입니다. 서버는 Tailscale IP에 바인딩합니다. 원격 PC 브라우저에서 `http://<라즈베리파이-Tailscale-IP>:8080`으로 접속합니다. (만약 Tailscale이 없고 로컬에서 테스트하려면 `python3 amr_server.py --host 127.0.0.1`.)

### 사용 순서

1. 실제 로봇 바퀴를 안전하게 준비한 후 **맵 생성 시작**을 클릭합니다. SLAM이 초기 자세를 캡처할 때까지 로봇을 정지시켜 주세요.
2. 수동 조종으로 Mapping을 마치고 **생성 완료**를 클릭합니다. 기본 저장 이름은 `test`이며, 원한다면 이름을 바꿀 수 있습니다.
3. 저장된 NPZ 지도가 표시되면 **흰색 영역에서 도착지 한 곳을 클릭**합니다. **파란 A* 경로가 즉시 표시되지만 모터는 작동하지 않습니다.**
4. **주행 시작** 클릭 시 경로를 재검증하고 자동 주행합니다. **정지/스페이스**로 중단할 수 있습니다. 실제 AMR에 적용하기 전 바퀴를 띄운 상태에서 모터·안전정지 기능을 충분히 확인하세요.

### 저장 파일

```text
~/ros2_amr_ws/maps/test.npz        # 초기 전방 정렬, A* 4방향 경로용
~/ros2_amr_ws/maps/test.pgm        # 원본 SLAM 이미지
~/ros2_amr_ws/maps/test.yaml       # 원본 지도 메타데이터
~/ros2_amr_ws/maps/test.data       # SLAM 직렬화
~/ros2_amr_ws/maps/test.posegraph  # SLAM 포즈그래프
```

※ `npz`만 방향이 바뀌며 `.pgm`, `.yaml`, `.data`, `.posegraph`는 기존 좌표계/형식을 유지합니다. **실제 로봇의 위치는 `map` 프레임 기준 TF에서 읽고, NPZ 메타데이터로 변환**합니다.

## 유의 사항

- 기존 `test.npz`가 이미 있는 경우에도 **새 관제 시스템에서 Mapping/생성 완료**로 해당 세션의 지도와 초기 로봇 방향을 함께 저장하는 과정을 권장합니다. 웹 관제는 주행 중 SLAM/TF가 살아 있어야 현재 차량 위치를 알 수 있습니다.
- 로봇의 위치 TF가 없으면 목적지 경로를 만들 수 없고, 미탐색/장애물/안전거리 셀은 목적지로 허용하지 않습니다.
- **경로 표시와 실제 주행은 별개**입니다. 회전 제어·오도메트리·장애물 안전거리/센서 정확도를 검증하기 전에는 자율주행 시험에 사용하지 마세요.
- 브라우저에서 `데모 모드`라고 표시되면 Pi 서버에 연결되지 않은 상태입니다. 실제 로봇을 조종하는 화면이 아닙니다.
- 시스템에 ROS2 `base_footprint`가 없고 `base_link`만 있다면 `amr_server.py`, `waypoint_nav_node.py`의 TF 프레임 이름을 실제 환경에 맞게 수정해야 합니다.

## 테스트

- `python3 -m py_compile *.py`: 문법 검사
- `python3 test_integration.py`: ROS2 장비 없이 생성한 샘플 지도로 **0°/90°/임의 각도 정렬, 셀↔월드 좌표 왕복, 4방향 A*, 목적지 거부, 5파일 교체** 검증
- 실제 Pi의 SLAM Toolbox 서비스·모터 주행은 아직 검증되지 않았습니다.


## 현재 로봇 위치를 A* 시작 셀로 사용하는 방식 (정확 셀 버전)

- `amr_server.py`는 `map → base_footprint` TF로 현재 로봇 중심의 `(x, y)` 좌표(미터)를 수신합니다.
- `astar_planner.GridMap.world_to_cell(x, y)`로 **NPZ의 `origin_x`, `origin_y`, `origin_yaw`, `resolution`을 적용해 `(row, col)` 셀**로 변환합니다.
- 이 **동일한 셀을 `astar.py`의 A* 시작점 `(col, row)`**으로 전달합니다. 목적지도 클릭한 셀 자체를 사용합니다.
- 주변의 다른 빈 셀을 시작점으로 고르는 `nearest_free` 보정은 제거했습니다. 실제 셀이 지도 범위 밖이거나 장애물 팽창 영역인 경우 **경로를 생성하지 않고 오류를 표시**합니다. 지도/TF/안전거리 문제를 해결한 뒤 다시 계산해야 합니다.
- 웹 `POST /api/plan`을 통한 **경로 미리보기 시점**과 주행 노드가 실제로 출발하는 **주행 시작 시점**에 각각 최신 위치를 이용해 계산합니다. 두 시점에 차량이 움직였다면 시작 셀도 달라질 수 있습니다.
- 응답 JSON의 `start_cell` 및 `goal_cell`은 `(x=col, y=row)` 순서입니다. 좌상단 웹 화면의 픽셀 좌표와 혼동하지 마세요.
- 이 변경은 A* 경로 탐색 정확성을 위한 것입니다. 모터 제어/실제 충돌 회피까지 검증된 것은 아닙니다.
