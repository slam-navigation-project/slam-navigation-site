---
layout: post
title: "rosbridge를 foxglove_bridge로 교체"
date: 2026-11-17 18:43:00 +0900
categories: [Capstone, ROS2, AMR]
tags: [ROS2-Humble, Foxglove, foxglove_bridge, rosbridge, TF]
---

## 1. 증상

- Foxglove 맵에서 자동차(`base_footprint`) 위치 표시가 눈에 띄게 늦음.

## 2. 원인

- `slam_wheel_odom.launch.py`에서 Foxglove 연결 통로로 `rosbridge_websocket`(Python, JSON 전송)을 사용.
- `/tf`가 초당 약 80개 발행됨 (SLAM `map→odom` 50Hz + 오도메트리 `odom→base_footprint` 30Hz).
- 여기에 `/scan`, `/map`까지 같은 웹소켓으로 나가서 파이에서 큐가 밀림.

## 3. 수정 내용

- `slam_wheel_odom.launch.py` 5번 블록의 rosbridge include를 지우고 `foxglove_bridge` 노드로 교체.
- `return LaunchDescription([...])`의 마지막 항목을 `rosbridge_launch` → `foxglove_node`로 변경.
- 안 쓰는 import(`IncludeLaunchDescription`, `AnyLaunchDescriptionSource`, `get_package_share_directory`) 정리 가능.

```python
    # 5. Foxglove Bridge (rosbridge 대신, 바이너리 전송이라 훨씬 가벼움)
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',   # 외부(내 PC)에서 접속 가능하게
        }],
        output='screen'
    )

    return LaunchDescription([
        rplidar_node,
        tf_laser_node,
        wheel_odom_proc,
        slam_node,
        foxglove_node
    ])
```

## 4. Foxglove 연결

- Open connection → **Foxglove WebSocket** → `ws://<파이 IP>:8765`

## 5. 결과

- 맵 위 차 위치 딜레이가 거의 없어짐.
