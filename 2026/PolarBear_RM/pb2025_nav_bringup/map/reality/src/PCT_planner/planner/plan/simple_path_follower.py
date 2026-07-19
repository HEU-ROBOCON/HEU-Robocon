import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Path 
import tf2_ros 
import tf2_geometry_msgs 
import numpy as np 
import math 

class SimplePathFollower(Node): 
     def __init__(self): 
         super().__init__('simple_path_follower') 
         # 订阅路径话题
         self.path_sub = self.create_subscription(Path, '/pct_path', self.path_callback, 10) 
         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) 
          
         self.tf_buffer = tf2_ros.Buffer() 
         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self) 
          
         # --- 核心控制参数 --- 
         self.lookahead_dist = 1.0     # 预瞄距离
         self.linear_vel = 1.0         # 正常巡航速度 (m/s)
         self.slow_down_vel = 0.3      # 减速区目标速度 (m/s)
         self.max_angular_vel = 0.8    # 最大角速度限制
         self.angle_threshold = 0.35   # 航向对准阈值
         
         self.slow_down_threshold = 1.5 # 触发减速的距离门槛 (米)
         self.stop_threshold = 0.4      # 停止距离阈值 (米)
         # -------------------- 
          
         self.get_logger().info(f"✅ 减速控制版已启动: 2m内减速至0.4, 0.4m内停止") 
    
     def path_callback(self, msg): 
         # 检查路径是否为空
         if len(msg.poses) == 0: 
             return 

         try: 
             # 1. 获取坐标变换 (imu 为机器人中心, odom 为全局坐标系) 
             transform = self.tf_buffer.lookup_transform('body', 'map', rclpy.time.Time()) 
             
             # 2. 终点距离计算与速度决策
             goal_pose = msg.poses[-1].pose
             local_goal = tf2_geometry_msgs.do_transform_pose(goal_pose, transform)
             dist_to_goal = math.sqrt(local_goal.position.x**2 + local_goal.position.y**2)

             # 判断是否直接停止
             if dist_to_goal <= self.stop_threshold:
                 self.get_logger().info(f"🏁 到达终点 (剩余: {dist_to_goal:.2f}m), 停止。")
                 self.cmd_pub.publish(Twist())
                 return

             # 判断是否需要减速
             if dist_to_goal < self.slow_down_threshold:
                 current_base_vel = self.slow_down_vel
                 self.get_logger().info(f"⚠️ 减速区 (剩余: {dist_to_goal:.2f}m), 速度: {current_base_vel}", throttle_duration_sec=1.0)
             else:
                 current_base_vel = self.linear_vel

             # 3. 寻找预瞄点 (Lookahead Point) 
             target_pt = None 
             for pose in msg.poses: 
                 lp = tf2_geometry_msgs.do_transform_pose(pose.pose, transform) 
                 dist = math.sqrt(lp.position.x**2 + lp.position.y**2) 
                 if dist > self.lookahead_dist: 
                     target_pt = lp.position 
                     break 
              
             if target_pt is None: 
                 target_pt = local_goal.position 

             # 4. 计算角度偏差与生成控制指令
             angle_to_target = math.atan2(target_pt.y, target_pt.x) 
             abs_angle = abs(angle_to_target) 
             cmd = Twist() 

             if abs_angle > 0.5:  # 阶段1: 角度偏差极大，原地旋转
                 cmd.linear.x = 0.0 
                 cmd.angular.z = np.clip(angle_to_target * 1.2, -self.max_angular_vel, self.max_angular_vel)
             
             elif abs_angle > self.angle_threshold: # 阶段2: 微调航向
                 cmd.linear.x = 0.1  # 给极小速度防止死锁
                 cmd.angular.z = np.clip(angle_to_target * 1.0, -0.5, 0.5)

             else: # 阶段3: 航向已对准，执行巡航
                 cmd.linear.x = current_base_vel 
                 cmd.angular.z = angle_to_target * 0.5 
                  
             self.cmd_pub.publish(cmd) 

         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: 
             self.get_logger().warn(f"等待 TF 变换: {e}", throttle_duration_sec=2.0) 

def main(): 
     import sys 
     rclpy.init(args=sys.argv) 
     node = SimplePathFollower() 
     try:
         rclpy.spin(node) 
     except KeyboardInterrupt:
         pass
     node.destroy_node() 
     rclpy.shutdown() 

if __name__ == '__main__': 
     main()
