# ROS工作空间

## 项目功能

本项目通过激光雷达和双目相机系统实现以下核心功能：

1. **机器人自身全场定位**：实时获取机器人在场地中的精确位置
2. **目标物定位**：识别并定位场地上关键目标物体的位置
3. 串口设备权限配置：`chmod 777 /dev/ttyUSB0`
4. ROS环境加载：`source ./devel/setup.bash`
5. ROS节点启动：`roslaunch serial_sender serial_sender.launch`

注意：需要clone其他所需开源项目补全src/，以及安装必要的硬件驱动库才能运行本项目。

## 自动启动设置

系统启动时将自动运行上述功能，配置方法如下：

```bash
# 1. 确保脚本有执行权限
chmod +x install_service.sh launch.sh

# 2. 运行安装脚本
sudo bash install_service.sh
```

安装脚本会：

- 创建systemd服务`robocon_launch.service`
- 启用开机自启动
- 启动服务

## 服务管理

安装后可使用以下命令管理服务：

| 操作 | 命令 |
|------|------|
| **禁用自启动** | `sudo systemctl disable robocon_launch.service` |
| **手动启动** | `sudo systemctl start robocon_launch.service` |
| **手动停止** | `sudo systemctl stop robocon_launch.service` |
| **重启服务** | `sudo systemctl restart robocon_launch.service` |
| **查看状态** | `sudo systemctl status robocon_launch.service` |
| **查看日志** | `journalctl -u robocon_launch.service -b` |

## 注意事项

1. 确保ROS环境已正确安装
2. 首次运行前编译工作空间：`catkin_make`
3. 串口设备`/dev/ttyUSB0`需存在
