import rclpy
from rclpy.node import Node
import tf2_ros
import serial
import math
from geometry_msgs.msg import TransformStamped

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        
        # 1. 初始化串口
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 921600, timeout=0.1)
            self.get_logger().info('✅ 串口 /dev/ttyUSB0 已连接')
        except Exception as e:
            self.get_logger().error(f'❌ 无法打开串口: {e}')

        # 2. 初始化 TF 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 3. 创建定时器，以固定频率获取 TF 并发送 (例如 50Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.get_logger().info('📍 TF 坐标转发节点 (map -> body) 已启动')

    def quaternion_to_yaw(self, q):
        """将四元数转换为偏航角 (Yaw) 角度制"""
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw)
        
    def timer_callback(self):
        try:
            # 4. 查找最新的 map 到 body 的变换
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map',   # 父坐标系
                'base_footprint',  # 子坐标系
                now,
                timeout=rclpy.duration.Duration(seconds=0.01)
            )

            # 5. 提取坐标
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            
            # 6. 提取旋转并转为 Yaw
            q = trans.transform.rotation
            yaw = self.quaternion_to_yaw(q)

            # 7. 格式化并发送
            send_str = f"heu{x:.3f},{y:.3f},{z:.3f},{yaw:.3f}\n"
            self.ser.write(send_str.encode('utf-8'))

            # 日志打印（每秒打印一次）
            self.get_logger().info(f'发送坐标: {send_str.strip()}', throttle_duration_sec=0.1)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # 如果变换还没准备好，静默跳过
            pass

    def __del__(self):
        if hasattr(self, 'ser'):
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
