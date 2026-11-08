---
layout: post
title: "런처 실행 및 ssh통신용 GUI 프로그램"
date: 2026-11-08 18:00:00 +0900
categories: [Capstone, ROS2, AMR]
tags: [Python, Tkinter, SSH, ROS2, Foxglove, RaspberryPi]
---

ssh 통신 및 런처 실행 등을 간단하게 실행하기 위한 GUI입니다. 기능은 점점 더 추가할 예정입니다!

---

## 1. GUI 구성 및 주요 기능


개발한 GUI는 다음과 같은 기능을 제공한다.

- **SSH 원격 접속** : Raspberry Pi의 IP 주소와 계정 정보를 입력하여 SSH 연결
- **ROS2 런처 관리** : SLAM 및 센서 관련 노드를 실행하고 종료
- **Foxglove Bridge 관리** : 실시간 ROS2 데이터 시각화를 위한 Bridge 실행 및 종료
- **통합 실행** : SLAM 런처와 Foxglove Bridge를 하나의 버튼으로 실행
- **시각화 프로그램 연동** : Foxglove를 실행하여 지도 및 센서 데이터 확인
- **실행 로그 출력** : 명령 실행 결과와 오류 메시지를 GUI에서 확인

---

## 2. 구현 방법

GUI는 Python의 `Tkinter`를 기반으로 구성하였으며, `Paramiko` 라이브러리를 통해 Raspberry Pi와 SSH 통신을 수행하도록 구현하였다.

사용자가 실행 버튼을 누르면 SSH를 통해 Raspberry Pi에 명령어를 전달하고, ROS2 런처 및 Foxglove Bridge를 백그라운드 프로세스로 실행한다.

또한 `threading`을 사용하여 원격 명령어가 실행되는 동안에도 GUI가 응답하도록 구성하였다.

프로그램 파일(amr_control_program.py)은 윈도우pc에서 실행한다. 

### GUI모습
![AMR Control Manager GUI](/assets/img/posts/2026-11-082026-11-08-GUI.png)

---

## 3. 필요 패키지, 라이브러리


### (1) Paramiko (Windows PC에 설치)

```powershell
py -m pip install paramiko
```

### (2) Foxglove Bridge (Raspberry Pi에 설치)

```bash
sudo apt install ros-humble-foxglove-bridge
```

---

## 4. 코드 수정, 기입할 부분

라즈베리파이의 ip와 유저명을 입력하는 부분이 있습니다. 이 부분을 수정해주세요!

```python
DEFAULT_HOST = "100.88.126.54"   -> tailscale ip주소 입력하면 됩니다
DEFAULT_USER = "yun"             -> 라즈베리파이 유저명 입력하면 됩니다. 

# ROS2 워크스페이스 및 코드 경로
REMOTE_WORKSPACE = "/home/" + DEFAULT_USER + "/ros2_amr_ws" -> 3번째 항에 워크스페이스 이름 입력하시면 됩니다! ~에 바로 위치해있다면 DEFAULT_USER 뒷부분은 지워주세요
REMOTE_CODE_DIR = f"{REMOTE_WORKSPACE}/src/riding_test"     -> 코드들이 위치한 경로 입력하시면 됩니다!
```

---
## 코드경로

##  Source Code

- [`코드 경로`](https://github.com/slam-navigation-project/slam-navigation-site/tree/main/code/2026-11-08)

