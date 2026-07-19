import os
import subprocess
import time
import shutil
import sys


# ============================================================
# tor_r2 Azure Kinect Python 启动脚本
# 对应 CMake 目标：getcubeinfo_k4a
# 对应可执行文件：build/getcubeinfo_k4a
# ============================================================

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
BUILD_DIR = os.path.join(PROJECT_DIR, "build")

TARGET_NAME = "getcubeinfo_k4a"

OWN_COLOR = "blue"        # blue 或 red
USE_SERIAL = True
SERIAL_PORT = "/dev/serial/by-path/pci-0000:67:00.4-usb-0:1.4:1.0-port0"
BAUD_RATE = 115200
REBUILD = True


def run_cmd(cmd, cwd=None, check=True):
    print(f"\n执行命令: {cmd}")
    return subprocess.run(cmd, shell=True, cwd=cwd, check=check)


def kill_processes():
    print("关闭可能占用 Azure Kinect / 串口的程序...")

    for name in ["k4aviewer", "k4arecorder", "k4a_hsv_debug", "getcubeinfo", "getcubeinfo_k4a", "getcubeinfo_d455"]:
        subprocess.run(
            f"pkill -9 {name}",
            shell=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

    time.sleep(1)


def check_azure_kinect():
    print("检查 Azure Kinect...")

    result = subprocess.run(
        "lsusb | grep -i 045e",
        shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    if result.returncode != 0:
        print("没有检测到 Azure Kinect。")
        print("请检查电源、USB线、USB3.0接口。")
        sys.exit(1)

    print("检测到 Azure Kinect:")
    print(result.stdout)

    if shutil.which("k4arecorder"):
        subprocess.run("k4arecorder --list", shell=True)
    else:
        print("未找到 k4arecorder，跳过 SDK 检查。")


def rebuild_project():
    print("重新编译工程...")

    os.makedirs(BUILD_DIR, exist_ok=True)

    # 如果 k4a 不在默认路径，可以保留这个 -Dk4a_DIR
    run_cmd(
        "cmake .. -Dk4a_DIR=/usr/lib/x86_64-linux-gnu/cmake/k4a",
        cwd=BUILD_DIR
    )

    run_cmd(
        f"make -j{os.cpu_count()} {TARGET_NAME}",
        cwd=BUILD_DIR
    )


def main():
    print("============================================")
    print("启动 tor_r2 Azure Kinect 红蓝块识别")
    print(f"PROJECT_DIR: {PROJECT_DIR}")
    print(f"BUILD_DIR  : {BUILD_DIR}")
    print(f"TARGET_NAME: {TARGET_NAME}")
    print(f"OWN_COLOR  : {OWN_COLOR}")
    print(f"USE_SERIAL : {USE_SERIAL}")
    print(f"SERIAL_PORT: {SERIAL_PORT}")
    print(f"BAUD_RATE  : {BAUD_RATE}")
    print("============================================")

    if not os.path.isdir(PROJECT_DIR):
        print(f"工程目录不存在: {PROJECT_DIR}")
        sys.exit(1)

    kill_processes()
    check_azure_kinect()

    if REBUILD:
        rebuild_project()

    exe_path = os.path.join(BUILD_DIR, TARGET_NAME)

    if not os.path.exists(exe_path):
        print(f"没有找到可执行文件: {exe_path}")
        print(f"请确认 CMakeLists.txt 里有 add_executable({TARGET_NAME} getcubeinfo_k4a.cpp)")
        sys.exit(1)

    if USE_SERIAL:
        if not os.path.exists(SERIAL_PORT):
            print(f"串口不存在: {SERIAL_PORT}")
            subprocess.run("ls /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-path/* 2>/dev/null", shell=True)
            sys.exit(1)

        serial_arg = SERIAL_PORT
    else:
        serial_arg = "none"

    cmd = f"./{TARGET_NAME} k4a {serial_arg} {BAUD_RATE} {OWN_COLOR}"

    print("============================================")
    print("运行命令:")
    print(cmd)
    print("============================================")

    run_cmd(cmd, cwd=BUILD_DIR)


if __name__ == "__main__":
    main()
