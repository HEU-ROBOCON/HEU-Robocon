#!/bin/bash

# 获取当前用户名和完整工作目录路径
CURRENT_USER=$SUDO_USER
if [ -z "$CURRENT_USER" ]; then
    CURRENT_USER=$USER
fi
WORKING_DIR=$(realpath $(pwd))

# 创建systemd服务文件
sudo tee /etc/systemd/system/robocon_launch.service > /dev/null <<EOF
[Unit]
Description=Robocon ROS Launch Service
After=network.target

[Service]
Type=simple
User=$CURRENT_USER
Group=$CURRENT_USER

# 环境变量设置
Environment=HOME=/home/$CURRENT_USER
Environment=ROS_ROOT=/opt/ros/noetic/share/ros
Environment=ROS_PACKAGE_PATH=/opt/ros/noetic/share
Environment=ROS_MASTER_URI=http://localhost:11311
Environment=ROS_VERSION=1
Environment=ROS_PYTHON_VERSION=3
Environment=ROS_DISTRO=noetic
Environment=PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages
Environment=LD_LIBRARY_PATH=/opt/ros/noetic/lib
Environment=PATH=/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

# 工作目录和启动命令
WorkingDirectory=$WORKING_DIR
ExecStartPre=/bin/sleep 10
ExecStartPre=/bin/bash -c '[ -e /dev/ttyUSB0 ] && /bin/chmod 777 /dev/ttyUSB0 || true'
ExecStart=/bin/bash -c 'source /opt/ros/noetic/setup.bash && source $WORKING_DIR/devel/setup.bash && $WORKING_DIR/launch.sh'

# 重启设置
Restart=on-failure
RestartSec=10

# 日志设置
StandardOutput=append:/var/log/robocon_launch.log
StandardError=append:/var/log/robocon_launch.log

[Install]
WantedBy=multi-user.target
EOF

# 设置权限和文件
sudo touch /var/log/robocon_launch.log
sudo chown $CURRENT_USER:$CURRENT_USER /var/log/robocon_launch.log
chmod +x $WORKING_DIR/launch.sh

# 重载并启动服务
sudo systemctl daemon-reload
sudo systemctl stop robocon_launch.service
sudo systemctl enable robocon_launch.service
sudo systemctl start robocon_launch.service

echo "服务安装完成！"
echo "查看服务状态: sudo systemctl status robocon_launch.service"
echo "查看详细日志: sudo journalctl -u robocon_launch.service -f"
echo "查看输出日志: sudo tail -f /var/log/robocon_launch.log"