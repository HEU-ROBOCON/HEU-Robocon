#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import sys
import os
import math

# 自动将脚本所在目录加入路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from quadrotor_msgs.msg import PositionCommand
    from geometry_msgs.msg import Twist
except ImportError as e:
    print(f"无法加载消息类型: {e}")
    sys.exit(1)

class PosCmdToTwist(Node):
    def __init__(self):
        super().__init__('poscmd_2_twist')
        
        # 订阅 Ego-Planner 话题
        self.sub = self.create_subscription(
            PositionCommand, 
            '/drone_0_planning/pos_cmd', 
            self.callback, 
            10)
            
        # 发布给底盘话题
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("✅ 转换节点已启动")
        self.get_logger().info("转换逻辑: Linear.X = Velocity.X, Angular.Z = Yaw_Dot")

    def callback(self, msg):
        twist = Twist()

        # 1. 线速度控制：将规划的 X 轴速度直接映射
        # 如果你的机器人是麦克纳姆轮或四足，且需要处理侧移，请保留 y 轴
        twist.linear.x = msg.velocity.x
        twist.linear.y = 0.0  # 强制屏蔽 y 轴，确保机器人只走直线
        twist.linear.z = 0.0  # 屏蔽 z 轴（高度）

        # 2. 偏航角控制：直接映射规划的角速度 yaw_dot
        # angular.z 对应机器人的左右旋转
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = msg.yaw_dot 

        # 打印调试信息（可选，如果觉得刷屏可以注释掉）
        # self.get_logger().info(f"VX: {twist.linear.x:.2f}, YawDot: {twist.angular.z:.2f}")

        self.pub.publish(twist)

def main():
    rclpy.init()
    node = PosCmdToTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()