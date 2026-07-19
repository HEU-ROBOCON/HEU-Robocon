import sys
import argparse
import numpy as np
import tf2_ros
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped, PoseStamped

from utils import traj2ros
from planner_wrapper import TomogramPlanner

sys.path.append('../')
from config import Config

class PCTPlanner(Node):
    def __init__(self, cfg, tomo_file, robot_height):
        super().__init__('pct_planner')
        self.cfg = cfg
        self.tomo_file = tomo_file
        self.robot_height = robot_height

        # QoS 配置：Transient Local 确保路径在发布后，新加入的订阅者也能收到
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # 发布与订阅
        self.path_pub = self.create_publisher(Path, "/pct_path", qos)
        self.click_sub = self.create_subscription(PointStamped, "/clicked_point", self.clicked_point_callback, 10)

        # 初始化底层规划器
        self.planner = TomogramPlanner(cfg)
        self.planner.loadTomogram(self.tomo_file)

        # 规划状态变量
        self.start_pos = None
        self.start_layer = None
        self.end_pos = None
        self.end_layer = None
        
        # TF 监听器：获取机器人实时位姿
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 定时器：控制自动重新规划的频率 (0.5秒/次)
        self.plan_timer = None
        self.replan_period = 0.5 

        self.get_logger().info("✅ PCT Planner 启动成功！")
        self.get_logger().info("💡 请在 RViz 中点击 'Publish Point' 确定终点。")

    def get_current_pose(self):
        """实时获取机器人相对于 map 的坐标"""
        try:
            # 将 'imu' 修改为 'body'，以匹配你 TF 树中的实际坐标系
            trans = self.tf_buffer.lookup_transform('map', 'body', rclpy.time.Time())
            
            curr_pos = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y
            ], dtype=np.float32)
            
            curr_height = trans.transform.translation.z
            return curr_pos, curr_height
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # 更新日志信息，方便调试
            self.get_logger().warn(f"无法获取位姿 (map -> body)，请检查定位节点: {e}")
            return None, None
        
    def clicked_point_callback(self, msg):
        """RViz 点击回调：更新终点并启动定时规划"""
        self.end_pos = np.array([msg.point.x, msg.point.y], dtype=np.float32)
        
        # 修复：增加底层限幅，防止出现 layer=-1
        raw_layer = self.planner.height2layer(msg.point.z)
        self.end_layer = max(0, raw_layer)
        
        self.get_logger().info(f"🚩 目标点已更新: {self.end_pos}, 目标层级: {self.end_layer}")

        if self.plan_timer is None:
            self.plan_timer = self.create_timer(self.replan_period, self.timer_plan_callback)
            self.get_logger().info("🔄 自动刷新模式已开启")
        
        self.timer_plan_callback()

    def timer_plan_callback(self):
        """定时执行函数：根据最新位置重新发布路径"""
        if self.end_pos is None:
            return

        # 获取当前起点
        curr_start, curr_height = self.get_current_pose()
        if curr_start is None:
            return

        self.start_pos = curr_start
        # 修复：起点层级同样增加限幅
        raw_layer = self.planner.height2layer(curr_height)
        self.start_layer = max(0, raw_layer)
        
        # 检查是否已到达
        dist = np.linalg.norm(self.start_pos - self.end_pos)
        if dist < 0.2:
            self.get_logger().info("🏁 距离目标极近，停止刷新路径")
            if self.plan_timer:
                self.plan_timer.cancel()
                self.plan_timer = None
            return

        self.pct_plan()

    def pct_plan(self):
        """调用算法生成并发布轨迹"""
        traj_3d = self.planner.plan(
            self.start_pos, 
            self.end_pos, 
            self.start_layer, 
            self.end_layer, 
            self.robot_height
        )
        
        if traj_3d is not None:
            # 仅发布 ROS2 Path 消息
            self.path_pub.publish(traj2ros(traj_3d))
        else:
            self.get_logger().warn("❌ 局部规划失败")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene', type=str, default='Spiral', help='场景名称')
    parser.add_argument('--robot_height', type=float, default=0.05, help='机器人高度')
    args = parser.parse_args()

    cfg = Config()
    
    scenes = {
        "Spiral": "spiral0.3_2",
        "Building": "building2_9",
        "Plaza": "plaza3_10",
        "Map": "map",
        "Scans": "scans"
    }
    tomo_file = scenes.get(args.scene, "map")

    rclpy.init(args=sys.argv)
    node = PCTPlanner(cfg, tomo_file, args.robot_height)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
