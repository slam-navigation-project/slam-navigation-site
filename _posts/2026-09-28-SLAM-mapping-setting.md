---
layout: post
title: "SLAM toolbox 기반 Mapping 및 /map 미수신 문제 해결"
date: 2026-09-28 00:00:00 +0900
categories: [Capstone, ROS2, SLAM]
tags: [SLAM, SLAM-Toolbox, ROS2, RaspberryPi, LiDAR, RViz2, TF]
---

## 1. SLAM Toolbox 실행 및 Map Topic 확인

2D LiDAR 센서로부터 수신한 `/scan` 데이터를 기반으로 지도를 생성하기 위해 ROS2의 `SLAM Toolbox`를 사용하였다.

### (1) SLAM Toolbox 실행

먼저 `SLAM Toolbox`를 실행하여 ROS2 노드에 추가한다.

```bash
ros2 launch slam_toolbox online_async_launch.py
```

실행 후 별도의 터미널을 열어 현재 실행 중인 노드를 확인한다.

```bash
ros2 node list
```

정상적으로 실행되었다면 다음과 같이 `/slam_toolbox` 노드를 확인할 수 있다.

```text
/slam_toolbox
```

### (2) Topic 확인

다음 명령어를 통해 현재 ROS2에서 사용 가능한 토픽을 확인한다.

```bash
ros2 topic list
```

SLAM을 수행하기 위해 주로 확인해야 할 토픽은 다음과 같다.

```text
/scan
/map
```

- `/scan` : LiDAR에서 측정한 거리 데이터
- `/map` : SLAM Toolbox에서 생성한 Occupancy Grid Map

### (3) RViz2에서 Map 추가

RViz2를 실행한다.

```bash
ros2 launch sllidar_ros2 view_sllidar_c1_launch.py
```

이후 좌측 하단의 Add 버튼을 통해 Map을 추가한다. 이후 좌측 메뉴에서 Map의 드롭다운 버튼을 눌러 옵션을 설정할 수 있다. 

```text
Add
→ By topic
→ /map 추가
```

---

## 2. /map tf 관련 문제 발생


문제 1. `Global Options`의 `Fixed Frame` 목록에 `map` 프레임이 나타나지 않음.

문제 2. `Map` 항목의 Status 중 'Message'의 상태가 'No map received'로 뜸.


다음 명령어를 통해 `map` 프레임의 존재 여부를 확인하였다.

```bash
ros2 run tf2_ros tf2_echo map laser
```

그러나 다음과 같이 `map` 프레임이 존재하지 않는다는 오류가 발생하였다.

```text
Invalid frame ID "map"
```

즉 `/map` 토픽 이름 자체는 생성되어 있었지만, 실제 Occupancy Grid Map 데이터와 `map` TF가 정상적으로 생성되지 않고 있었다.

---

## 3. 원인 확인

(현재 테스트 단계에서는 별도의 IMU 센서나 휠 엔코더를 사용하지 않고 LiDAR 센서만을 이용해 SLAM을 테스트하고 있다.)

SLAM Toolbox의 기본 설정값을 확인하기 위해 다음 명령어를 사용하였다.

```bash
ros2 param get /slam_toolbox base_frame
ros2 param get /slam_toolbox odom_frame
ros2 param get /slam_toolbox scan_topic
ros2 param get /slam_toolbox mode
```

초기에는 다음과 같은 값이 적용되어 있었다.

```text
base_frame = base_footprint
odom_frame = odom
scan_topic = /scan
mode = mapping
```

그러나 현재 TF 구조에는 `base_footprint` 및 `odom` 프레임이 존재하지 않았기 때문에 SLAM Toolbox가 정상적으로 LaserScan 데이터를 처리하지 못하였다.

이를 해결하기 위해 별도의 YAML 설정 파일을 생성하고 `base_frame`과 `odom_frame`을 현재 존재하는 `base_link`(기본적으로 존재) 프레임으로 지정하였다.

> 현재 설정은 IMU 및 엔코더가 없는 상태에서 LiDAR-only SLAM 동작을 확인하기 위한 테스트용 설정이다. 추후 장비 추가 시 이를 수정해야할 필요가 있다. 

---

## 4. LiDAR-only SLAM 설정 파일(.yaml) 생성

### (1) config 디렉터리 및 YAML 파일 생성

워크스페이스 내부에 `config` 디렉터리를 생성한다.

```bash
mkdir -p ~/{워크스페이스}/config
```

이후 `nano`를 이용하여 설정 파일을 생성한다.

```bash
nano ~/{워크스페이스}/config/lidar_only_slam.yaml
```


### (2) SLAM Toolbox Parameter 작성

생성한 YAML 파일에 다음 내용을 입력한다.

```yaml
slam_toolbox:
  ros__parameters:

    # Solver
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # Frames
    odom_frame: base_link
    map_frame: map
    base_frame: base_link

    # LiDAR
    scan_topic: /scan

    # Mapping
    mode: mapping
    use_map_saver: true

    # Map
    resolution: 0.05
    map_update_interval: 2.0
    max_laser_range: 12.0
    min_laser_range: 0.15

    # Scan matching
    use_scan_matching: true
    use_scan_barycenter: true

    # 작은 움직임도 처리
    minimum_travel_distance: 0.05
    minimum_travel_heading: 0.05

    # Async
    throttle_scans: 1
    minimum_time_interval: 0.2
    scan_queue_size: 1

    # TF
    transform_publish_period: 0.02
    transform_timeout: 0.5
    tf_buffer_duration: 30.0

    # Loop closure
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # Scan matcher
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0

    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349

    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true

    occupancy_threshold: 0.1
```

현재 가장 중요한 설정은 다음 세 가지이다.

```yaml
odom_frame: base_link
map_frame: map
base_frame: base_link
```

기존에는 `odom_frame`이 `odom`, `base_frame`이 `base_footprint`으로 설정되어 있었지만 해당 프레임이 존재하지 않았기 때문에 두 프레임을 `base_link`로 변경하였다.

---

## 5. LiDAR Static TF 생성

LiDAR의 좌표계와 로봇의 기준 좌표계를 연결하기 위해 ROS2의 `tf2_ros`를 사용하여 Static Transform을 생성한다.

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser
```


현재는 LiDAR가 `base_link`의 원점에 있다고 가정하여 위치와 회전값을 모두 `0`으로 설정하였다.

```text
0 0 0 0 0 0
│ │ │ │ │ │
│ │ │ └───── 회전값
└─────────── 위치값
```

실제 로봇에 LiDAR를 장착할 경우에는 LiDAR의 실제 장착 위치에 맞게 값을 수정해야 한다.

또한 `laser`라는 이름은 `/scan` 토픽에서 사용하는 `frame_id`와 일치해야 한다.

다음 명령어를 통해 확인할 수 있다.

```bash
ros2 topic echo /scan --once
```

예를 들어 다음과 같이 출력된다면,

```yaml
header:
  frame_id: laser
```

Static TF의 자식 프레임 역시 `laser`로 지정하면 된다.

---

## 6. YAML 설정 파일을 적용하여 SLAM Toolbox 실행

기본 설정으로 SLAM Toolbox를 실행하면 작성한 YAML 파일이 자동으로 적용되지 않는다.

따라서 `slam_params_file` 옵션을 이용해 작성한 설정 파일을 직접 지정한다.

```bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/{유저명}/{워크스페이스}/config/lidar_only_slam.yaml use_sim_time:=false
```

---

## 7. SLAM Toolbox Parameter 적용 여부 확인

YAML 파일이 실제로 적용되었는지 다음 명령어를 통해 확인한다.

```bash
ros2 param get /slam_toolbox odom_frame
```

```bash
ros2 param get /slam_toolbox base_frame
```

정상적으로 설정되었다면 두 값 모두 다음과 같이 출력된다.

```text
String value is: base_link
```


따라서 YAML 파일을 생성하는 것뿐만 아니라, SLAM Toolbox 실행 시 해당 YAML 파일이 실제로 적용되었는지를 확인하는 과정이 중요하다.

---

## 8. Map 데이터 수신 확인

SLAM Toolbox를 실행한 상태에서 `/map` 토픽으로 실제 Occupancy Grid 데이터가 발행되는지 확인한다.

```bash
ros2 topic echo /map --once
```

정상적으로 Map이 생성되고 있다면 다음과 같은 데이터가 출력된다.

```yaml
info:
  resolution: 0.05
  width: ...
  height: ...
data:
  ...
```

이후 RViz2에서 `Fixed Frame`을 확인하면 기존에 존재하지 않았던 `map` 프레임을 선택할 수 있다.

```text
Global Options
└── Fixed Frame
    └── map
```

`Map` Display의 Topic을 `/map`으로 지정하면 생성된 Occupancy Grid Map을 RViz2에서 확인할 수 있다.

---

## 9. 문제 해결 과정 정리

이번 문제는 처음에는 RViz2의 설정 문제로 생각하였지만, 실제 원인은 SLAM Toolbox의 Frame 설정이었다.

초기 상태는 다음과 같았다.

```text
LiDAR
  ↓
/scan
  ↓
laser

SLAM Toolbox
  ├── base_frame = base_footprint
  └── odom_frame = odom

→ base_footprint / odom 프레임이 존재하지 않음
→ LaserScan 처리 불가
→ /map 데이터 미생성
→ map TF 미생성
→ RViz Fixed Frame에 map이 나타나지 않음
```

이를 다음과 같이 수정하였다.

```text
base_link
   ↓
 laser
   ↓
 /scan
   ↓
SLAM Toolbox
   ├── base_frame = base_link
   └── odom_frame = base_link
   ↓
 /map 생성
   ↓
RViz2에서 map 표시
```

최종적으로 `lidar_only_slam.yaml` 파일을 생성하고 SLAM Toolbox 실행 시 해당 파일을 명시적으로 지정함으로써 `/map` 데이터를 정상적으로 수신할 수 있었다.

> 향후 Wheel Encoder 또는 IMU 기반 Odometry를 추가할 경우에는 `odom → base_link` TF를 별도로 생성하고, `odom_frame`을 다시 `odom`으로 설정하는 방식으로 시스템을 확장할 필요가 있다.

---
## 실행 시 터미널에 입력할 명령어
### (1) tf 설정
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser
```

### (2) slam_toolbox 활성화(+yaml 파일 적용)
```bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/yun/ros2_amr_ws/config/lidar_only_slam.yaml use_sim_time:=false
```

### (3) 라이다 출력을 Rviz 화면상에서 확인하기
```bash
ros2 launch sllidar_ros2 view_sllidar_c1_launch.py
```



