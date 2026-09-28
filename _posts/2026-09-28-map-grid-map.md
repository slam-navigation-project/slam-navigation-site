---
layout: post
title: "Map 저장 및 Occupancy Grid 데이터 확인"
date: 2026-09-28 22:00:00 +0900
categories: [Capstone, ROS2, SLAM]
tags: [SLAM, ROS2, OccupancyGrid, Map, RViz, Nav2]
---

## 🗺️ SLAM Map 저장 및 데이터 확인

SLAM Toolbox를 통해 생성한 맵을 저장하고, `/map` 토픽을 통해 Occupancy Grid Map의 데이터를 확인한다.

---

### 1. 맵 생성 및 저장

생성된 맵은 `nav2_map_server` 패키지의 `map_saver_cli`를 이용하여 저장할 수 있다.

```bash
ros2 run nav2_map_server map_saver_cli -f ~/{맵 이름}
```

원하는 경로에 맵을 저장하고 싶은 경우 파일 이름 앞에 저장할 경로를 지정한다.

예를 들어 `ros2_amr_ws/maps` 경로에 `test`라는 이름으로 저장하려면 다음과 같이 입력한다.

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_amr_ws/maps/test
```

명령어를 실행하면 지정한 경로에 다음 두 파일이 생성된다.

```text
test.pgm
test.yaml
```

`pgm` 파일에는 실제 Occupancy Grid Map 이미지가 저장되며, `yaml` 파일에는 맵의 해상도와 원점 좌표 등의 정보가 저장된다.

---

### 2. RViz에서 생성된 맵 확인

SLAM을 실행하면 RViz의 `Map` Display를 통해 현재 생성되고 있는 Occupancy Grid Map을 확인할 수 있다.

#### RViz 화면

![ex_screenshot](./img/map사진.png)

#### RViz Display 메뉴

![ex_screenshot](./img/rviz_display.png)

---

### 3. `/map`의 Occupancy Grid 데이터 확인

SLAM Toolbox에서 생성한 지도는 ROS2의 `/map` 토픽을 통해 전달된다.

`/map`의 메시지 형식은 `nav_msgs/msg/OccupancyGrid`이며, 실제 지도 데이터는 `data[]`에 **1차원 배열 형태**로 저장된다.

`data` 배열만 출력하려면 다음 명령어를 사용한다.

```bash
ros2 topic echo /map --field data
```

#### 출력 예시

```text
array('b', [
-1, -1, 100, 100, 100, 100, 100, 100, 100, -1,
-1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
-1, 100, 0, 0, 0, 0, 0, 100, 0, -1,
-1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
100, 0, 0, 0, 0, 0, 0, 0, -1, -1,
...
])
```

Occupancy Grid의 각 값은 다음과 같은 의미를 가진다.

```text
-1   : 아직 탐색되지 않은 영역 (Unknown)
0    : 이동 가능한 영역 (Free)
100  : 장애물이 존재하는 영역 (Occupied)
```

실제 지도는 2차원 형태이지만 `/map`의 `data[]`에서는 이를 1차원 배열로 변환하여 전달한다.

---

### 4. Map 정보 확인

`/map` 토픽에는 지도 데이터뿐만 아니라 맵의 크기, 해상도, 원점 등의 정보도 포함되어 있다.

전체 정보를 한 번 출력하려면 다음 명령어를 사용한다.

```bash
ros2 topic echo /map --once
```

info만 보고싶다면 아래 명령어를 사용한다.
```bash
ros2 topic echo /map --once --field info
```

```yaml
info:
  map_load_time:
    sec: 0
    nanosec: 0
  resolution: 0.05
  width: 24
  height: 19
  origin:
    position:
      x: -0.5
      y: -0.4
      z: 0.0
```

여기서 주요 값의 의미는 다음과 같다.

```text
resolution : Occupancy Grid 한 칸의 실제 크기 [m]
width      : 맵의 가로 셀 개수
height     : 맵의 세로 셀 개수
origin     : 맵 좌표 (0, 0)의 실제 좌표
```

따라서 실제 맵의 크기는 다음과 같이 계산할 수 있다.

```text
실제 가로 길이 = width × resolution
실제 세로 길이 = height × resolution
```

---
## Python에서 맵 데이터 확인하기

```python
## map.txt : .py 파일과 같은 경로에 위치. 맵 데이터가 쉼표 단위로 구분되어있음.
## 맵 데이터를 배열로 받아오는 작업
with open("map.txt", "r", encoding="utf-8") as f:
    map_data = list(map(int, f.read().split(",")))

## 맵의 사이즈
W_map = 19  
H_map = 24  

map = []
## 1차원 배열 -> 2차원 배열
for i in range(0, W_map*H_map, W_map):
    map.append(map_data[i:i+W_map])

map.reverse()
```

#### 출력 결과
```text
[100, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]


[100, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]


[100, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
...
```

