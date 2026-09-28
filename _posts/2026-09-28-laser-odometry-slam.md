---
layout: post
title: "2D SLAM 구축 및 Laser Odometry 검증"
date: 2026-09-28 23:39:00 +0900
categories: [Capstone, ROS2, AMR]
tags: [ROS2-Humble, LiDAR, SLAM-Toolbox, Laser-Odometry, Foxglove]
---

## 1. 진행 개요

- 하드웨어 섀시 및 휠 엔코더 부재 상태에서 RPLIDAR C1 단독 2D SLAM 동작 검증.
- 임시 레이저 오도메트리(`laser_odom.py`) 구현을 통한 핸드헬드 매핑 테스트.
- Foxglove WebSocket을 통한 원격 데스크탑 실시간 시각화 구성.

---

## 2. 주요 작업 내용

### (1) 센서 구동 및 시각화
- RPLIDAR C1 시리얼 권한 부여 및 460800 baudrate 환경 구동 (`/scan` 정상 발행).
- `rosbridge_server` 기반 WebSocket(포트 9090) 파이프라인 구축.
- 원격 PC Foxglove Studio Web에서 `/scan` 및 `/map` 실시간 렌더링.

### (2) 가상 레이저 오도메트리 구현 (`laser_odom.py`)
- 엔코더 부재 환경 극복을 위해 포인트클라우드 기반 변위 추정 노드 작성.
- 스캔 점군 중심점(Centroid) 이동량 및 SVD 기반 회전각(Yaw) 역산.
- 동적 `odom -> base_footprint` TF 브로드캐스트.

### (3) SLAM 파라미터 최적화
- `slam_toolbox` 갱신 주기 0.5초 단축 및 이동 임계값 완화.
- 센서 회전 시 글로벌 맵 절대 방위(동서남북) 유지 검증.

---

## 3. 결과 및 현황

### (1) 성과 및 시각화 검증
- 손으로 이동 시 방 윤곽 및 복도 형태 맵 생성 확인.
- Foxglove Studio Web 상에서 전역 좌표계(`map`) 기준 2D 점유 격자 지도 및 `/tf` 정상 연동 확인.

![Foxglove Web 2D SLAM Dashboard]({{ site.baseurl }}/assets/img/posts/2026-09-28-slam-foxglove.png)

### (2) 한계 및 보정 필요성
- 단순 중심점 기반 계산으로 인해 신규 벽면 진입 시 좌표 튐 발생.
- 손 회전 및 미세 흔들림에 따른 맵 왜곡 발생 ("되는 듯하나 불안정함").
- 정밀 매칭 알고리즘 보정 필요.

---