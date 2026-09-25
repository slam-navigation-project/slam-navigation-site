---
layout: post
title: "SSH 및 VNC 원격 접속 환경 구축"
date: 2026-09-26 02:42:00 +0900
categories: [Capstone, RaspberryPi, Ubuntu]
tags: [RaspberryPi, Ubuntu22.04, SSH, VNC, TigerVNC, Remote]
---

VNC 환경 설정 과정은 아래 글을 참고하였다.

- 참고: [Ubuntu VNC 설정](https://8trian8.tistory.com/116)

---

## SSH 환경 설정

### (1) OpenSSH Server 설치

Ubuntu에서 SSH 접속을 허용하려면 먼저 `openssh-server` 패키지를 설치한다.

```bash
sudo apt update
sudo apt install openssh-server
```

### (2) SSH 포트 개방

Ubuntu에서 `ufw` 방화벽을 사용하는 경우 SSH 접속에 사용되는 **22번 TCP 포트**를 허용한다.

```bash
sudo ufw allow 22/tcp
```

현재 방화벽 설정은 다음 명령어로 확인할 수 있다.

```bash
sudo ufw status
```

### (3) Raspberry Pi IP 주소 확인

현재 Raspberry Pi의 IP 주소는 다음 명령어를 통해 확인할 수 있다.

```bash
hostname -I
```

예를 들어 Raspberry Pi의 IP 주소가 `192.168.0.107`이고 Ubuntu 사용자명이 `yun`이라면 다음과 같이 접속한다.

```bash
ssh yun@192.168.0.107
```

---

## VNC 원격 GUI 환경 설정

SSH는 터미널 환경만 사용할 수 있기 때문에 `rviz2`와 같이 GUI가 필요한 프로그램을 사용하려면 별도의 원격 데스크톱 환경이 필요하다.

이를 위해 VNC Server를 이용해 Raspberry Pi의 Ubuntu GUI 환경에 원격으로 접속하도록 구성하였다.

VNC 설정 방법은 다음 글을 참고하였다.

- 참고: [Ubuntu VNC 설정](https://8trian8.tistory.com/116)

---

## 4. VNC Server 실행 및 종료

VNC Server를 실행할 때는 사용할 **디스플레이 세션 번호**를 지정한다.

예를 들어 `:1` 세션을 생성하려면 다음과 같이 실행한다.

```bash
vncserver -localhost no :1
```

여기서 `:1`은 VNC에서 사용하는 디스플레이 세션 번호를 의미한다.


예를 들어 Raspberry Pi의 IP 주소가 `192.168.0.107`이라면 다음과 같이 입력한다.

```text
192.168.0.107:1
```

### VNC Server 종료

현재 실행 중인 VNC 세션을 종료하려면 다음 명령어를 사용한다.

```bash
vncserver -kill :1
```

현재 실행되고 있는 VNC 세션은 다음 명령어를 통해 확인할 수 있다.

```bash
vncserver -list
```

---

## GNOME Terminal이 실행되지 않는 경우

VNC 환경에서 Ubuntu GUI에 정상적으로 접속되더라도 **GNOME Terminal이 실행되지 않고 계속 대기 상태로 남는 문제**가 발생할 수 있다.

이 경우 `xterm`을 설치하여 기본 터미널 대신 사용할 수 있다.

### (1) xterm 설치

```bash
sudo apt install -y xterm
```

### (2) xterm 실행

Ubuntu GUI에서 다음 단축키를 입력한다.

```text
Alt + F2
```

실행 창이 나타나면 다음을 입력한다.

```text
xterm
```

Enter를 누르면 `xterm` 터미널이 실행된다.

`xterm`에서도 일반적인 Ubuntu 명령어와 ROS2 명령어를 동일하게 사용할 수 있다.

예를 들어 ROS2 Humble 환경을 불러오려면 다음 명령어를 입력한다.

```bash
source /opt/ros/humble/setup.bash
```

현재 사용 중인 ROS2 Workspace 환경까지 불러오려면 다음 명령어를 추가로 실행한다.

```bash
source ~/ros2_amr_ws/install/setup.bash
```

---

## 6. 자주 사용하는 명령어 정리

| 기능 | 명령어 |
|---|---|
| SSH Server 설치 | `sudo apt install openssh-server` |
| SSH 상태 확인 | `sudo systemctl status ssh` |
| SSH 시작 | `sudo systemctl start ssh` |
| SSH 자동 실행 | `sudo systemctl enable ssh` |
| SSH 포트 허용 | `sudo ufw allow 22/tcp` |
| IP 주소 확인 | `hostname -I` |
| SSH 접속 | `ssh 사용자명@IP주소` |
| xterm 설치 | `sudo apt install -y xterm` |
| VNC 세션 실행 | `vncserver -localhost no :1` |
| VNC 세션 종료 | `vncserver -kill :1` |
| VNC 세션 확인 | `vncserver -list` |

---

## 원격 개발 환경 구성

최종적인 원격 개발 환경은 다음과 같이 구성하였다.

```text
개발 PC
   │
   ├── SSH
   │
   └── VNC
        │
        ▼
Raspberry Pi 4
Ubuntu 22.04
   │
   ├── ROS2 Humble
   ├── SLAM Toolbox
   ├── RViz2
   └── LiDAR Driver
```

SSH는 패키지 설치, ROS2 노드 실행 및 시스템 설정 등 **CLI 기반 작업**에 사용하고, VNC는 `RViz2`와 같이 화면 출력이 필요한 **GUI 기반 작업**에 사용한다.

이를 통해 Raspberry Pi에 별도의 모니터와 키보드를 연결하지 않고도 개발 PC에서 ROS2 및 SLAM 개발 환경을 원격으로 제어할 수 있도록 구성하였다.
````
