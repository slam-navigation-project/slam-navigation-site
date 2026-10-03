---
layout: post
title: "A* 알고리즘 경로 탐색 테스트"
date: 2026-10-03 18:00:00 +0900
categories: [Capstone, ROS2, AMR]
tags: [A-Star, PathPlanning, Python, Tkinter, OccupancyGrid, SLAM]
---

## 1. A* 경로 탐색 테스트

테스트용으로 생성한 맵 데이터를 기반으로 **A* 알고리즘을 이용한 최단 경로 탐색**을 테스트하였다.

SLAM을 통해 생성되는 `Occupancy Grid Map`과 유사한 형태의 맵 데이터를 `map.txt`에 저장하고, 이를 2차원 배열로 변환하여 A* 알고리즘의 입력으로 사용한다.

탐색된 최단 경로는 `Tkinter`를 이용해 그리드 맵 위에 시각화한다.

---

## 2. 프로그램 구조

프로그램은 기능에 따라 다음과 같이 분리하였다.(파일 경로는 모두 동일 선상에 존재해야 한다.)

```text
main.py
│
├── path_visualization.py
├── map_loader.py
└── astar.py

map.txt
```

각 파일의 역할은 다음과 같다.

```text
map.txt
   │
   ▼
map_loader.py
   │
   │  2차원 맵 배열 생성
   ▼
main.py
   │
   ├──────────────▶ astar.py
   │                    │
   │                    │ 최단 경로 탐색
   │                    ▼
   │                   PATH
   │                    │
   ◀────────────────────┘
   │
   ▼
path_visualization.py
   │
   ▼
경로 시각화
```

---

## 3. 소스파일 설명


### (1) `path_visualization.py`

A* 알고리즘을 통해 생성된 최단 경로를 시각화하는 소스코드이다.

Python의 `Tkinter` 라이브러리를 사용하여 맵 데이터를 그리드 형태로 출력하고, A* 알고리즘을 통해 생성된 이동 경로를 해당 맵 위에 표시한다.

주요 입력 데이터는 다음과 같다.

```python
grid_map
path
start
goal
```

맵의 각 셀은 Occupancy Grid의 값에 따라 서로 다른 형태로 표시하며, 탐색을 통해 생성된 `PATH`를 그리드 위에 표시하여 실제 경로를 확인할 수 있도록 구성하였다.

---

### (2) `map_loader.py`

SLAM을 통해 생성한 Occupancy Grid Map의 배열 데이터를 저장한 텍스트 파일로부터 **2차원 맵 배열을 생성하는 역할**을 한다.

테스트에서는 `/map` 토픽에서 얻을 수 있는 `data[]` 배열과 유사한 형태로 데이터를 `map.txt`에 저장하였다.

맵 데이터는 다음 값을 기준으로 구성하였다.

```text
100 : 장애물
0   : 이동 가능한 공간
-1  : 미탐색 공간
```

텍스트 파일에 저장된 맵 데이터는 처음에는 1차원 배열 형태이므로, 맵의 `width`와 `height` 정보를 이용하여 A* 알고리즘에서 사용할 수 있는 2차원 배열 형태로 변환한다.

```text
1차원 Map Data
      │
      ▼
map_loader.py
      │
      ▼
2차원 Grid Map
```

현재는 테스트를 위해 `map.txt`를 사용하고 있지만, 이후에는 ROS2의 `/map` 토픽으로부터 Occupancy Grid 데이터를 직접 수신하도록 변경할 예정이다.

---

### (3) `astar.py`

`map_loader.py`에서 생성한 2차원 맵 배열을 이용하여 **A* 알고리즘으로 시작 지점에서 목표 지점까지의 최단 경로를 탐색**한다.

A* 알고리즘에서는 각 노드에 대해 다음 세 가지 비용을 사용한다.

```text
G : 시작 지점부터 현재 노드까지의 이동 비용

H : 현재 노드부터 목표 지점까지의 예상 비용
    (Heuristic Cost)

F : G + H
```

현재 구현에서는 상하좌우 **4방향 이동**을 사용하며, 휴리스틱 비용 계산에는 `Manhattan Distance`를 사용한다.

```python
def get_Manhattan(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)
```

탐색 과정에서 각 노드는 다음과 같은 정보를 저장한다.

```python
{
    "G": G,
    "H": H,
    "F": F,
    "parent": (parent_x, parent_y)
}
```

`parent`에는 해당 노드까지 도달하기 직전에 방문한 노드를 저장한다.

목표 지점까지 탐색이 완료되면 목표 지점에서부터 `parent`를 역으로 따라가면서 최종 이동 경로인 `PATH`를 생성한다.

```text
Start
  │
  ▼
Node
  │
  ▼
Node
  │
  ▼
Node
  │
  ▼
Goal
```

최소 `F` 값을 갖는 노드를 빠르게 찾기 위해 Python의 `heapq`를 사용하였다.

---

### (4) `main.py`

`main.py`는 위에서 작성한 각 기능을 하나로 연결하여 실행하는 **상위 실행 코드**이다.

다음 세 개의 Python 파일을 import하여 사용한다.

```python
from map_loader import map_loader
from astar import astar
from path_visualization import path_visualization
```

전체적인 프로그램 실행 순서는 다음과 같다.

```text
1. map_loader.py
        │
        ▼
   Grid Map 생성

2. astar.py
        │
        ▼
   A* 최단 경로 탐색

3. path_visualization.py
        │
        ▼
   탐색 결과 시각화
```

따라서 각 소스코드는 맵 생성, 경로 탐색, 시각화 기능을 독립적으로 담당하며 `main.py`에서 이를 통합하여 실행한다.

---

## 4. 전체 동작 구조

전체 프로그램의 데이터 흐름은 다음과 같다.

```text
              map.txt
                 │
                 ▼
         ┌────────────────┐
         │ map_loader.py  │
         └───────┬────────┘
                 │
                 │ Grid Map
                 ▼
         ┌────────────────┐
         │    astar.py    │
         └───────┬────────┘
                 │
                 │ PATH
                 ▼
      ┌────────────────────────┐
      │ path_visualization.py  │
      └────────────┬───────────┘
                   │
                   ▼
            경로 시각화 결과

       위 과정은 main.py에서 통합 실행
```

기능별로 소스코드를 분리하여 작성함으로써 추후 맵 데이터 입력 방식이나 경로 탐색 알고리즘, 시각화 방식을 변경하더라도 각각의 모듈을 독립적으로 수정할 수 있도록 구성하였다.

---



##  Source Code

- [`main.py`](../assets/code/, main.py)
- [`path_visualization.py`]({{ './code/path_visualization.py' | relative_url }})
- [`map_loader.py`]({{ './code/map_loader.py' | relative_url }})
- [`astar.py`]({{ './code/astar.py' | relative_url }})
- [`map.txt`]({{ .'/code/map.txt' | relative_url }})
