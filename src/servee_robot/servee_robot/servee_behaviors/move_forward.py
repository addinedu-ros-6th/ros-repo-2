import math
from geometry_msgs.msg import PoseArray, Twist
from py_trees.behaviour import Behaviour
from typing import Any
from py_trees.common import Status, Access
from rclpy.node import Node

class MoveForward(Behaviour):
    def __init__(self, name: str):
        super(MoveForward, self).__init__(name)
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="path", access=Access.READ)
        self.blackboard.register_key(key="robot_state", access=Access.READ)
        self.blackboard.register_key(key="target_distance", access=Access.WRITE)
        self.blackboard.register_key(key="odom_pose", access=Access.READ)

        self.blackboard.register_key(key="debug", access=Access.WRITE)
        self.blackboard.debug = 0
        
    def setup(self, **kwargs: Any) -> None:
        self.node: Node = kwargs['node']
        self.cmd_vel_publisher =self.node.create_publisher(
            Twist, 
            '/base_controller/cmd_vel_unstamped',
            10)
        self.prev_position = None  # 이전 위치 저장용 변수
        self.max_linear_speed = 0.08 
        
    def update(self) -> Status:
        if self.are_you_ready() == False:
            return Status.FAILURE
        
        curr_position = self.blackboard.odom_pose.position
        
        # 이전 위치와 현재 위치의 차이를 구합니다.
        if self.prev_position is not None:
            dx = curr_position.x - self.prev_position.x
            dy = curr_position.y - self.prev_position.y
            distance_moved = math.sqrt(dx ** 2 + dy ** 2) 
            self.blackboard.target_distance -= distance_moved

        # 현재 위치를 이전 위치로 업데이트
        self.prev_position = curr_position
        
        cmd_msg = Twist()
        cmd_msg.linear.x = self.max_linear_speed
        self.cmd_vel_publisher.publish(cmd_msg)
        return Status.SUCCESS
    
    
    def are_you_ready(self):
        """
        이동을 할 수 있는 상태인지 체크한다.
        1. 경로를 받아왔는지?
        2. robot_state가 move 인지?
        """               
        if self.blackboard.robot_state not in ["task", "home"]:
            return False
        
        return True