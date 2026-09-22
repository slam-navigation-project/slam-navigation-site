---
layout: post
title: "ROS 2 Nav2와 Slam Toolbox를 연동한 실시간 목적지 자율주행 파이프라인 구축"
date: 2026-09-22 10:00:00 +0900
categories: [Robotics, SLAM]
tags: [ROS2, Nav2, SlamToolbox, LiDAR, Jackal]
---

## 1. 개요
기존의 정적 맵 기반 주행(AMCL)과 달리, 사전 지도 없이 실시간으로 지도를 작성하면서 동시에 목적지로 이동하는 **Online Async SLAM + Nav2 통합 제어 파이프라인**을 구성했습니다.

## 2. 주요 실행 커맨드

LiDAR 드라이버 및 로봇 베이스를 Bringup한 후 아래 명령어로 SLAM과 Nav2 스택을 구동합니다:

```bash
# 1. Slam Toolbox 실행
ros2 launch slam_toolbox online_async_launch.py \
    params_file:=./config/mapper_params_online_async.yaml \
    use_sim_time:=false

# 2. Nav2 Navigation Bringup 실행
ros2 launch nav2_bringup navigation_launch.py \
    params_file:=./config/nav2_params.yaml \
    use_sim_time:=false
```

## 3. Costmap Inflation Radius 튜닝 결과
좁은 복도 주행 시 코스트맵 인플레이션 반경이 너무 클 경우 로봇이 멈추는 데드락(Deadlock)이 발생했습니다. 
이를 해결하기 위해 `inflation_layer`의 `cost_scaling_factor`를 `3.0`에서 `5.0`으로 조정하여 주행 통과율을 **94%**로 개선했습니다.
