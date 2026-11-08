
import shlex
import threading
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import queue
from pathlib import Path
import time
import webbrowser
from urllib.parse import urlencode

try:
    import paramiko
except ImportError:
    paramiko = None


# Raspberry Pi 설정
DEFAULT_HOST = "100.88.126.54"
DEFAULT_USER = "yun"

ROS_SETUP = "/opt/ros/humble/setup.bash"
REMOTE_WORKSPACE = "/home/" + DEFAULT_USER + "/ros2_amr_ws"
REMOTE_CODE_DIR = f"{REMOTE_WORKSPACE}/src/riding_test"

LAUNCH_FILE = "slam_wheel_odom.launch.py"

FOXGLOVE_PORT = 8765


class Manager:
    def __init__(self, root):
        self.root = root
        root.title("AMR Control Manager")
        root.geometry("730x610")

        self.ssh = None
        self.launch_pid = None
        self.bridge_pid = None

        self.events = queue.Queue()
        self.busy = False

        self.host = tk.StringVar(value=DEFAULT_HOST)
        self.user = tk.StringVar(value=DEFAULT_USER)
        self.password = tk.StringVar()
        self.launch = tk.StringVar(value=LAUNCH_FILE)

        self.status = tk.StringVar(value="SSH 연결 안 됨")

        frm = ttk.Frame(root, padding=14)
        frm.pack(fill="both", expand=True)

        fields = [
            ("Raspberry Pi IP / hostname", self.host, False),
            ("SSH username", self.user, False),
            ("SSH password", self.password, True),
            ("Launch filename", self.launch, False),
        ]

        for r, (label, var, secret) in enumerate(fields):
            ttk.Label(frm, text=label).grid(
                row=r, column=0, sticky="w", pady=5
            )

            ttk.Entry(
                frm,
                textvariable=var,
                show="*" if secret else "",
                width=47
            ).grid(
                row=r,
                column=1,
                columnspan=2,
                sticky="ew",
                pady=5
            )

        # SSH 연결
        ttk.Button(
            frm, text="SSH 연결",
            command=self.connect
        ).grid(row=4, column=0, columnspan=3,
               sticky="ew", pady=7)

        # SLAM 관리
        ttk.Button(
            frm, text="SLAM 실행",
            command=self.start
        ).grid(row=5, column=0, sticky="ew", pady=4)

        ttk.Button(
            frm, text="SLAM 종료",
            command=self.stop
        ).grid(row=5, column=1, columnspan=2,
               sticky="ew", pady=4)

        # Foxglove Bridge 관리
        ttk.Button(
            frm, text="Bridge 실행",
            command=self.start_bridge
        ).grid(row=6, column=0, sticky="ew", pady=4)

        ttk.Button(
            frm, text="Bridge 종료",
            command=self.stop_bridge
        ).grid(row=6, column=1, columnspan=2,
               sticky="ew", pady=4)

        # 통합 실행
        ttk.Button(
            frm,
            text="SLAM + Bridge 한번에 실행",
            command=self.start_all
        ).grid(row=7, column=0, columnspan=3,
               sticky="ew", pady=5)

        # Foxglove Desktop 열기
        ttk.Button(
            frm,
            text="Foxglove 열기",
            command=self.open_foxglove
        ).grid(row=8, column=0, columnspan=3,
               sticky="ew", pady=5)

        ttk.Label(
            frm, textvariable=self.status
        ).grid(row=9, column=0, columnspan=3,
               sticky="w", pady=8)

        self.logbox = scrolledtext.ScrolledText(
            frm, height=14, state="disabled"
        )
        self.logbox.grid(
            row=10, column=0, columnspan=3,
            sticky="nsew"
        )

        frm.columnconfigure(1, weight=1)
        frm.rowconfigure(10, weight=1)

        self.root.after(100, self.drain)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    # ----------------------------------------
    # 로그 및 작업 관리
    # ----------------------------------------

    def emit(self, kind, msg):
        self.events.put((kind, msg))

    def drain(self):
        try:
            while True:
                kind, msg = self.events.get_nowait()

                if kind == "status":
                    self.status.set(msg)
                else:
                    self.logbox.configure(state="normal")
                    self.logbox.insert("end", msg + "\n")
                    self.logbox.see("end")
                    self.logbox.configure(state="disabled")

        except queue.Empty:
            pass

        self.root.after(100, self.drain)

    def worker(self, fn):
        if self.busy:
            self.emit("log", "[WARN] 이전 작업 진행 중")
            return

        self.busy = True

        def run():
            try:
                fn()
            except Exception as exc:
                self.emit("log", f"[ERROR] {exc}")
            finally:
                self.busy = False

        threading.Thread(
            target=run, daemon=True
        ).start()

    # ----------------------------------------
    # SSH 연결
    # ----------------------------------------

    def connect(self):
        if paramiko is None:
            messagebox.showerror(
                "모듈 누락",
                "pip install paramiko 를 실행하세요."
            )
            return

        host = self.host.get().strip()
        username = self.user.get().strip()
        password = self.password.get()

        if not host or not username or not password:
            messagebox.showwarning(
                "입력 확인",
                "IP, 사용자명, 비밀번호를 입력하세요."
            )
            return

        def job():
            client = paramiko.SSHClient()
            client.load_system_host_keys()

            known_hosts = Path.home() / ".ssh" / "known_hosts"

            if known_hosts.is_file():
                client.load_host_keys(str(known_hosts))

            client.set_missing_host_key_policy(
                paramiko.RejectPolicy()
            )

            client.connect(
                hostname=host,
                port=22,
                username=username,
                password=password,
                look_for_keys=False,
                allow_agent=False,
                timeout=8,
                auth_timeout=8
            )

            if self.ssh:
                self.ssh.close()

            self.ssh = client

            # 새 SSH 연결에서 기존 PID를 추측하지 않음
            self.launch_pid = None
            self.bridge_pid = None

            self.emit(
                "status",
                f"SSH 연결됨: {username}@{host}"
            )
            self.emit("log", "[OK] SSH 연결 성공")

        self.worker(job)

    def remote(self, command):
        if (
            self.ssh is None
            or self.ssh.get_transport() is None
            or not self.ssh.get_transport().is_active()
        ):
            raise RuntimeError("SSH 연결이 필요합니다.")

        wrapped = "bash -lc " + shlex.quote(command)

        stdin, stdout, stderr = self.ssh.exec_command(
            wrapped, timeout=10
        )

        status = stdout.channel.recv_exit_status()

        output = (
            stdout.read().decode(errors="replace")
            + stderr.read().decode(errors="replace")
        )

        if status != 0:
            raise RuntimeError(
                output.strip()
                or f"Remote exit status: {status}"
            )

        return output

    # ----------------------------------------
    # ROS2 공통 환경
    # ----------------------------------------

    def ros_environment(self):
        return (
            f"source {shlex.quote(ROS_SETUP)} && "
            f"source {shlex.quote(REMOTE_WORKSPACE + '/install/setup.bash')} && "
            "unset CYCLONEDDS_URI RMW_IMPLEMENTATION && "
            "export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0"
        )

    # ----------------------------------------
    # 원격 백그라운드 프로세스 실행
    # ----------------------------------------

    def run_background(self, command, log_file):
        # setsid: 별도 프로세스 그룹
        # exec: 셸을 실제 ROS2 프로세스로 교체
        # 로그 리다이렉션: SSH 종료 후에도 실행 유지

        inner = (
            self.ros_environment()
            + " && exec "
            + command
        )

        background = (
            "setsid bash -lc "
            + shlex.quote(inner)
            + " > "
            + shlex.quote(log_file)
            + " 2>&1 < /dev/null & echo $!"
        )

        result = self.remote(background)
        pid = int(result.strip().splitlines()[-1])

        time.sleep(1.5)

        check = self.remote(
            f"if kill -0 {pid} 2>/dev/null && "
            f"[ \"$(ps -o stat= -p {pid} 2>/dev/null | cut -c1)\" != Z ]; "
            "then echo RUNNING; else echo EXITED; fi"
        )

        if check.strip() != "RUNNING":
            log = self.remote(
                "tail -n 30 "
                + shlex.quote(log_file)
                + " 2>/dev/null || true"
            )
            raise RuntimeError(
                "원격 실행 실패:\n" + log
            )

        return pid

    def stop_process(self, pid):
        # 이 GUI가 생성한 프로세스 그룹만 종료
        self.remote(
            f"kill -INT -- -{pid} 2>/dev/null "
            f"|| kill -INT {pid} 2>/dev/null "
            "|| true"
        )

    # ----------------------------------------
    # SLAM 실행 및 종료
    # ----------------------------------------

    def start_slam_job(self):
        if self.launch_pid is not None:
            self.emit("log", "[INFO] SLAM 이미 실행 요청됨")
            return

        filename = self.launch.get().strip()

        if not filename or Path(filename).name != filename:
            raise ValueError("Launch filename을 확인하세요.")

        launch_path = f"{REMOTE_CODE_DIR}/{filename}"

        command = (
            "ros2 launch "
            + shlex.quote(launch_path)
        )

        pid = self.run_background(
            command,
            "/tmp/amr_launch_gui.log"
        )

        self.launch_pid = pid

        self.emit(
            "log",
            f"[OK] SLAM 런처 실행 (PID={pid})"
        )
        self.emit("status", "SLAM 실행 중")

    def start(self):
        self.worker(self.start_slam_job)

    def stop(self):
        def job():
            if self.launch_pid is None:
                self.emit(
                    "log",
                    "[INFO] GUI에서 관리 중인 SLAM 없음"
                )
                return

            pid = self.launch_pid
            self.stop_process(pid)
            self.launch_pid = None

            self.emit(
                "log",
                f"[OK] SLAM 종료 신호 전송 (PID={pid})"
            )
            self.emit("status", "SLAM 종료 요청됨")

        self.worker(job)

    # ----------------------------------------
    # Foxglove Bridge 실행 및 종료
    # ----------------------------------------

    def start_bridge_job(self):
        if self.bridge_pid is not None:
            self.emit(
                "log",
                "[INFO] Bridge 이미 실행 요청됨"
            )
            return

        # Tailscale 주소에만 바인딩
        # 호스트 입력값이 IP가 아닌 이름이면
        # 라즈베리파이의 Tailscale IPv4 사용
        bridge_ip = self.remote(
            "tailscale ip -4"
        ).strip().splitlines()[0]

        command = (
            "ros2 launch foxglove_bridge "
            "foxglove_bridge_launch.xml "
            f"port:={FOXGLOVE_PORT} "
            f"address:={shlex.quote(bridge_ip)}"
        )

        pid = self.run_background(
            command,
            "/tmp/foxglove_bridge_gui.log"
        )

        # 포트가 실제 LISTEN 상태인지 확인
        listening = self.remote(
            f"ss -lnt | grep -E ':{FOXGLOVE_PORT}[[:space:]]' || true"
        )

        if not listening.strip():
            log = self.remote(
                "tail -n 30 /tmp/foxglove_bridge_gui.log "
                "2>/dev/null || true"
            )
            self.stop_process(pid)
            raise RuntimeError(
                "Bridge 포트가 열리지 않았습니다:\n" + log
            )

        self.bridge_pid = pid

        self.emit(
            "log",
            f"[OK] Foxglove Bridge 실행 (PID={pid})"
        )
        self.emit(
            "log",
            f"[INFO] ws://{bridge_ip}:{FOXGLOVE_PORT}"
        )
        self.emit("status", "Foxglove Bridge 실행 중")

    def start_bridge(self):
        self.worker(self.start_bridge_job)

    def stop_bridge(self):
        def job():
            if self.bridge_pid is None:
                self.emit(
                    "log",
                    "[INFO] GUI에서 관리 중인 Bridge 없음"
                )
                return

            pid = self.bridge_pid
            self.stop_process(pid)
            self.bridge_pid = None

            self.emit(
                "log",
                f"[OK] Bridge 종료 신호 전송 (PID={pid})"
            )
            self.emit("status", "Bridge 종료 요청됨")

        self.worker(job)

    # ----------------------------------------
    # SLAM + Bridge 통합 실행
    # ----------------------------------------

    def start_all(self):
        def job():
            self.emit(
                "log",
                "[INFO] SLAM + Bridge 통합 실행 시작"
            )

            self.start_slam_job()

            # SLAM 시작 직후 브리지 실행
            self.start_bridge_job()

            self.emit(
                "log",
                "[OK] 통합 실행 요청 완료"
            )
            self.emit(
                "status",
                "SLAM + Bridge 실행 중"
            )

        self.worker(job)

    # ----------------------------------------
    # Foxglove Desktop 열기
    # ----------------------------------------

    def open_foxglove(self):
        host = self.host.get().strip()

        if not host:
            messagebox.showwarning(
                "입력 확인",
                "Raspberry Pi IP를 입력하세요."
            )
            return

        websocket_url = (
            f"ws://{host}:{FOXGLOVE_PORT}"
        )

        params = urlencode({
            "ds": "foxglove-websocket",
            "ds.url": websocket_url,
            "openIn": "desktop"
        })

        url = (
            "https://app.foxglove.dev/~/view?"
            + params
        )

        try:
            webbrowser.open(url)
            self.emit(
                "log",
                f"[INFO] Foxglove 연결 요청: {websocket_url}"
            )
        except Exception as exc:
            messagebox.showerror(
                "Foxglove 실행 실패",
                str(exc)
            )

    # ----------------------------------------
    # GUI 종료
    # ----------------------------------------

    def close(self):
        if self.launch_pid is not None or self.bridge_pid is not None:
            answer = messagebox.askyesno(
                "종료 확인",
                "원격 프로그램이 실행 중일 수 있습니다.\n"
                "GUI만 종료할까요?\n\n"
                "SLAM과 Bridge는 유지됩니다."
            )

            if not answer:
                return

        if self.ssh:
            self.ssh.close()

        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    Manager(root)
    root.mainloop()
