from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access

import py_trees_ros
import rclpy
import rclpy.parameter
from rclpy.node import Node
from servee_interfaces.srv import PathPlan
from servee_interfaces.msg import ReqPath, ResPath
from geometry_msgs.msg import Pose
from etc.utils.pose_utils import PoseUtils


class RequestPath(Behaviour):
    def __init__(self, name: str):
        super(RequestPath, self).__init__(name)
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="goal_pose", access=Access.READ)
        self.blackboard.register_key(key="curr_pose", access=Access.READ)

        self.blackboard.register_key(key="robot_state", access=Access.WRITE)
        self.blackboard.register_key(key="robot_state", access=Access.READ)
        
    def setup(self, **kwargs):
        self.logger.debug(f"Request path: setup")
        self.blackboard.robot_state = "idle" 
        self.req_path_pub = RequestPathPublisher()

        
    def initialise(self) -> None:   
        if self.are_you_ready() == False:
            return Status.FAILURE
          
        goal_pose = self.blackboard.goal_pose
        curr_pose = self.blackboard.curr_pose.pose.pose
        
        self.req_path_pub.publish(curr_pose, goal_pose)        
        
    def update(self) -> Status:
        
        if self.are_you_ready() == False:
            return Status.FAILURE
        return Status.SUCCESS


    def are_you_ready(self):
        if self.blackboard.exists('goal_pose') == False:
            return False
        
        if self.blackboard.robot_state not in ["idle", "home"]:
            return False
   
   
class RequestPathPublisher(Node):
    def __init__(self):
        super().__init__("request_path_publisher_node")
        self.publisher_ = self.create_publisher(
            ReqPath,
            'servee/request_path',
            py_trees_ros.utilities.qos_profile_latched()
        )
        
    def publish(self, curr_pose, goal_pose):

        # 현재 위치
        curr_x = curr_pose.position.x
        curr_y = curr_pose.position.y
        
        # 목표 위치
        goal_x = goal_pose.goal_pose.position.x
        goal_y = goal_pose.goal_pose.position.y
        
        msg = ReqPath()
        msg.curr_pose = PoseUtils.create_pose(curr_x, curr_y)
        msg.goal_pose = PoseUtils.create_pose(goal_x, goal_y)
        
        self.publisher_.publish(msg)

# class PathServiceClient():
#     def __init__(self, node):
#         self.node = node
#         self.path_client = self.node.create_client(PathPlan, "servee/get_path")
        
#     def request_path(self, curr_pose:Pose, goal_pose: Pose):
#         """
#         PathPlanner에게 경로를 요청합니다.
        
#         Args:
#             goal_pose (Pose): 목표 위치
#         Returns:
#             None
#         """
#         while not self.path_client.wait_for_service(timeout_sec=10):
#             self.node.get_logger().debug("Server opening...")
        
#         request = self.__create_path_request(curr_pose, goal_pose)
#         future = self.path_client.call_async(request)
        
#         return future    
    
    
#     def __create_path_request(self, curr_pose, goal_pose):
#         """
#         private
#         경로 계획을 요청하는 서비스 메시지를 만들기
        
#         Args:
#             goal_x (float): 목표 위치 x 좌표
#             goal_y (float): 목표 위치 y 좌표
            
#         Returns:
#             PathPlan.Request: 현재 위치와 목표위치를 담는 srv
#         """
        
#         # 현재 위치
#         curr_pose_x = curr_pose.position.x
#         curr_pose_y = curr_pose.position.y
        
#         # 목표 위치
#         goal_x = goal_pose.goal_pose.position.x
#         goal_y = goal_pose.goal_pose.position.y
        
#         request = PathPlan.Request()
#         request.curr_pose = PoseUtils.create_pose(curr_pose_x, curr_pose_y)
#         request.goal_pose = PoseUtils.create_pose(goal_x, goal_y)

#         return request