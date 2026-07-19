import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import math
import serial

class IntegratedPathFollower(Node):
    def __init__(self):
        super().__init__('integrated_path_follower')
        
        # --- 1. 串口初始化 ---
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 921600, timeout=0.1)
            self.get_logger().info('✅ 串口 /dev/ttyUSB0 已连接 (定位TF+控制模式)')
        except Exception as e:
            self.get_logger().error(f'❌ 无法打开串口: {e}')

        # --- 2. 坐标系配置 ---
        self.base_frame = 'body'  # 机器人坐标系
        self.map_frame = 'map'    # 世界坐标系

        # --- 3. ROS2 订阅与发布 ---
        self.path_sub = self.create_subscription(Path, '/pct_path', self.path_callback, 10)
        # 注意：这里保留订阅只是为了触发发送频率，核心坐标从 TF 获取
        self.odom_sub = self.create_subscription(Odometry, '/fastlio2/lio_odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- 4. TF2 监听器 ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # --- 5. 核心控制参数 ---
        self.lookahead_dist = 1.0     
        self.linear_vel = 1.0         
        self.slow_down_vel = 0.3      
        self.max_angular_vel = 0.8    
        self.angle_threshold = 0.35   
        self.slow_down_threshold = 1.5 
        self.stop_threshold = 0.4      

        self.get_logger().info(f'🚀 节点启动：获取 {self.base_frame} 相对于 {self.map_frame} 的位姿')

    def quaternion_to_yaw(self, q):
        """将四元数转换为偏航角 (Yaw) 角度制"""
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def safe_serial_write(self, data_str):
        if hasattr(self, 'ser') and self.ser.is_open:
            try:
                self.ser.write(data_str.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f'串口发送失败: {e}')

    def odom_callback(self, msg):
        """
        修改点：不再直接使用 msg 中的坐标，而是通过 TF 实时获取 body 相对于 map 的位置
        """
        try:
            # 获取最新的 TF 变换
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, 
                self.base_frame, 
                now,
                timeout=rclpy.duration.Duration(seconds=0.01)
            )

            # 提取 TF 坐标
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            
            # 提取旋转并转为 Yaw
            q = trans.transform.rotation
            yaw = self.quaternion_to_yaw(q)

            # 格式化并发送定位数据
            loc_str = f"heu{x:.3f},{y:.3f},{z:.3f},{yaw:.3f}\n"
            self.safe_serial_write(loc_str)
            
            # 定时打印定位日志
            self.get_logger().info(f'[TF-LOC] 发送定位: {loc_str.strip()}', throttle_duration_sec=1.0)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # 如果 TF 还没准备好，跳过此次发送
            pass

    def path_callback(self, msg):
        """路径跟随逻辑：计算 vel 信号"""
        if len(msg.poses) == 0:
            return

        try:
            # 这里的转换依然使用 TF 将路径转换到机器人 body 坐标系
            transform = self.tf_buffer.lookup_transform(self.base_frame, self.map_frame, rclpy.time.Time())
            
            goal_pose = msg.poses[-1].pose
            local_goal = tf2_geometry_msgs.do_transform_pose(goal_pose, transform)
            dist_to_goal = math.sqrt(local_goal.position.x**2 + local_goal.position.y**2)

            cmd = Twist()
            if dist_to_goal <= self.stop_threshold:
                self.get_logger().info(f'🏁 到达终点 ({dist_to_goal:.2f}m), 停止。')
                self.cmd_pub.publish(cmd)
                self.send_vel_to_serial(0.0, 0.0)
                return

            current_base_vel = self.slow_down_vel if dist_to_goal < self.slow_down_threshold else self.linear_vel

            # 寻找预瞄点
            target_pt = local_goal.position
            for pose in msg.poses:
                lp = tf2_geometry_msgs.do_transform_pose(pose.pose, transform)
                if math.sqrt(lp.position.x**2 + lp.position.y**2) > self.lookahead_dist:
                    target_pt = lp.position
                    break

            angle_to_target = math.atan2(target_pt.y, target_pt.x)
            abs_angle = abs(angle_to_target)

            if abs_angle > 0.5:
                cmd.linear.x = 0.0
                cmd.angular.z = np.clip(angle_to_target * 1.2, -self.max_angular_vel, self.max_angular_vel)
                mode = "旋转对准"
            elif abs_angle > self.angle_threshold:
                cmd.linear.x = 0.1
                cmd.angular.z = np.clip(angle_to_target * 1.0, -0.5, 0.5)
                mode = "航向微调"
            else:
                cmd.linear.x = current_base_vel
                cmd.angular.z = angle_to_target * 0.5
                mode = "巡航追踪"

            # 打印控制数据日志
            self.get_logger().info(
                f'🎮 [{mode}] 距离: {dist_to_goal:.2f}m | CMD: vx={cmd.linear.x:.2f}, wz={cmd.angular.z:.2f}',
                throttle_duration_sec=0.5
            )

            self.cmd_pub.publish(cmd)
            self.send_vel_to_serial(cmd.linear.x, cmd.angular.z)

        except Exception as e:
            self.get_logger().warn(f"TF 变换异常: {e}", throttle_duration_sec=2.0)

    def send_vel_to_serial(self, vx, wz):
        vel_str = f"vel{vx:.3f},{wz:.3f}\n"
        self.safe_serial_write(vel_str)

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedPathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser'):
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
