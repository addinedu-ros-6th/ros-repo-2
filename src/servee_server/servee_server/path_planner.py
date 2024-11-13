import math
import rclpy
from rclpy.node import Node
from etc.tools.astar_planning import AStarPlanner
from geometry_msgs.msg import Pose, PoseArray
from servee_interfaces.msg import ReqPath, ResPath
from tf_transformations import quaternion_from_euler
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy


class PathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner_node")
        self.astar_plan = AStarPlanner(resolution=1, rr=0.35, padding=2)
        
        # qos_profile = QoSProfile(depth=1)
        # qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        
        self.create_subscription(
            ReqPath,
            'servee/request_path',
            self.callback_get_path,
            10
        )
        
        self.publisher_= self.create_publisher(
            ResPath,
            "servee/response_path",
            10
        )

        
    def callback_get_path(self, msg):
        """
        경로를 요청받고 Path로 응답
        Returns:
            path(PoseArray): 이동 경로
        """
        self.get_logger().info("요청 받음")
        
        curr = msg.curr_pose.position
        goal = msg.goal_pose.position
        
        rx, ry, tpx, tpy, tvec_x, tvec_y = self.astar_plan.planning(curr.x, curr.y, goal.x, goal.y)
        self.get_logger().info("경로 계산")
        res = ResPath()
        for x, y, vec_x, vec_y in zip(tpx, tpy, tvec_x, tvec_y):
            pose = Pose()
            pose.position.x = x
            pose.position.y = y

            yaw_angle = math.atan2(vec_y, vec_x)
            quaternion = quaternion_from_euler(0, 0, yaw_angle, 'rxyz')
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            
            res.path.poses.append(pose)
            self.get_logger().info(f"경로: x:{pose.position.x:.3f}, y:{pose.position.y:.3f}")
            
        if res.path.poses:
            self.get_logger().info(f"경로 보내기 {res.path.poses[0].position}")
            self.get_logger().info(f"경로 개수: {str(len(res.path.poses))}")
            self.publisher_.publish(res)
        else:
            self.get_logger().warning("경로 계산에 실패하여 경로가 없습니다.")
       

    
def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()  
    
if __name__ == "__main__":
    main()