import os
import subprocess
import time
import sys


# ============================================================
# tor_r2 D455 Python 启动脚本
# 对应 CMake 目标：getcubeinfo_d455
# 对应可执行文件：build/getcubeinfo_d455
# ============================================================

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
BUILD_DIR = os.path.join(PROJECT_DIR, "build")

TARGET_NAME = "getcubeinfo_d455"

OWN_COLOR = "blue"        # blue 或 red

USE_SERIAL = False
SERIAL_PORT = "/dev/serial/by-path/pci-0000:67:00.4-usb-0:1.4:1.0-port0"
BAUD_RATE = 115200

# D455 彩色通道，需要按 v4l2-ctl --list-devices 的结果修改
CAMERA_DEVICE = "/dev/video6"

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

REBUILD = True


def run_cmd(cmd, cwd=None, check=True):
    print(f"\n执行命令: {cmd}")
    return subprocess.run(cmd, shell=True, cwd=cwd, check=check)


def kill_processes():
    print("关闭可能占用相机/串口的程序...")

    for name in ["getcubeinfo", "getcubeinfo_d455", "getcubeinfo_k4a", "camera_inference"]:
        subprocess.run(
            f"pkill -9 {name}",
            shell=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

    time.sleep(1)


def check_d455_camera():
    print("检查 D455 / V4L2 相机...")

    if not os.path.exists(CAMERA_DEVICE):
        print(f"相机设备不存在: {CAMERA_DEVICE}")
        print("当前视频设备：")
        subprocess.run("ls /dev/video* 2>/dev/null", shell=True)
        print("")
        print("请执行下面命令确认 D455 彩色通道：")
        print("v4l2-ctl --list-devices")
        sys.exit(1)

    print(f"检测到相机设备: {CAMERA_DEVICE}")

    if subprocess.run("which v4l2-ctl >/dev/null 2>&1", shell=True).returncode == 0:
        print("相机格式信息:")
        subprocess.run(f"v4l2-ctl -d {CAMERA_DEVICE} --list-formats-ext | head -80", shell=True)
    else:
        print("未安装 v4l2-ctl，跳过格式检查。可执行：sudo apt install v4l-utils")


def rebuild_project():
    print("重新编译工程...")

    os.makedirs(BUILD_DIR, exist_ok=True)

    run_cmd("cmake ..", cwd=BUILD_DIR)
    run_cmd(f"make -j{os.cpu_count()} {TARGET_NAME}", cwd=BUILD_DIR)


def main():
    print("============================================")
    print("启动 tor_r2 D455 红蓝块识别")
    print(f"PROJECT_DIR  : {PROJECT_DIR}")
    print(f"BUILD_DIR    : {BUILD_DIR}")
    print(f"TARGET_NAME  : {TARGET_NAME}")
    print(f"CAMERA_DEVICE: {CAMERA_DEVICE}")
    print(f"CAMERA_SIZE  : {CAMERA_WIDTH}x{CAMERA_HEIGHT}@{CAMERA_FPS}")
    print(f"OWN_COLOR    : {OWN_COLOR}")
    print(f"USE_SERIAL   : {USE_SERIAL}")
    print(f"SERIAL_PORT  : {SERIAL_PORT}")
    print(f"BAUD_RATE    : {BAUD_RATE}")
    print("============================================")

    if not os.path.isdir(PROJECT_DIR):
        print(f"工程目录不存在: {PROJECT_DIR}")
        sys.exit(1)

    kill_processes()
    check_d455_camera()

    if REBUILD:
        rebuild_project()

    exe_path = os.path.join(BUILD_DIR, TARGET_NAME)

    if not os.path.exists(exe_path):
        print(f"没有找到可执行文件: {exe_path}")
        print(f"请确认 CMakeLists.txt 里有 add_executable({TARGET_NAME} getcubeinfo_d455.cpp)")
        sys.exit(1)

    if USE_SERIAL:
        if not os.path.exists(SERIAL_PORT):
            print(f"串口不存在: {SERIAL_PORT}")
            subprocess.run("ls /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-path/* 2>/dev/null", shell=True)
            sys.exit(1)
        serial_arg = SERIAL_PORT
    else:
        serial_arg = "none"

    cmd = (
        f"./{TARGET_NAME} "
        f"{CAMERA_DEVICE} "
        f"{serial_arg} "
        f"{BAUD_RATE} "
        f"{OWN_COLOR} "
        f"{CAMERA_WIDTH} "
        f"{CAMERA_HEIGHT} "
        f"{CAMERA_FPS}"
    )

    print("============================================")
    print("运行命令:")
    print(cmd)
    print("============================================")

    run_cmd(cmd, cwd=BUILD_DIR)


if __name__ == "__main__":
    main()
