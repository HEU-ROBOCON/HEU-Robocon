import rclpy
from rclpy.node import Node
# 1. 导入里程计消息类型
from nav_msgs.msg import Odometry 
import serial
import json # 建议使用JSON格式发送，方便单片机解析
import math # 导入数学库用于四元数转换

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        
        # 初始化串口
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 921600, timeout=0.1)
            self.get_logger().info('串口 /dev/ttyUSB1 已连接')
        except Exception as e:
            self.get_logger().error(f'无法打开串口: {e}')

        # 2. 修改订阅的消息类型为 Odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/fastlio2/lio_odom',
            self.listener_callback,
            10)
        
        self.get_logger().info('FastLIO2 里程计转发节点已启动')

    def quaternion_to_yaw(self, q):
        """
        将四元数转换为偏航角 (Yaw)
        """
        # 四元数分量
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        # yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # 将弧度转换为角度 (如果单片机需要角度制，取消下面注释)
        yaw = math.degrees(yaw)
        
        return yaw
        
    def listener_callback(self, msg):
        # 3. 提取里程计中的位置数据 (Position)
        # msg.pose.pose.position 包含 x, y, z
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
	
	# 提取并转换偏航角
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q)
        # 4. 格式化数据
        # 方案 A: 简单的字符串格式 (例如: "X:1.23,Y:4.56,Z:7.89\n")
        # send_str = f"X:{x:.2f},Y:{y:.2f},Z:{z:.2f}\n"
        
        # 方案 B: 纯数字逗号分隔 (单片机最容易解析)
        send_str = f"heu{x:.3f},{y:.3f},{z:.3f},{yaw:.3f}"

        try:
            self.ser.write(send_str.encode('utf-8'))
            # 降低日志频率，防止刷屏
            if self.get_clock().now().to_msg().nanosec % 10 == 0:
                self.get_logger().info(f'发送坐标: {send_str.strip()},Yaw:{yaw:.2f}')
        except Exception as e:
            self.get_logger().error(f'串口发送失败: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close() # 退出时关闭串口
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
