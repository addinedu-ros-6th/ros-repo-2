from typing import Any

import numpy as np
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access

import py_trees_ros
import rclpy
import rclpy.parameter
from rclpy.node import Node
from servee_interfaces.msg import ReqPath, ResPath
from geometry_msgs.msg import Pose
from etc.utils.pose_utils import PoseUtils

class ResponsePath(Behaviour):
    def __init__(self, name: str):
        super(ResponsePath, self).__init__(name)
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="path", access=Access.WRITE)
        self.blackboard.register_key(key="robot_state", access=Access.WRITE)
        self.blackboard.register_key(key="waypoint", access=Access.WRITE)
        self.blackboard.register_key(key="next_pose", access=Access.WRITE)
        self.blackboard.register_key(key="next_pose", access=Access.READ)
        self.blackboard.register_key(key="curr_pose", access=Access.READ)
        self.blackboard.register_key(key="target_distance", access=Access.WRITE)
        
    def setup(self, **kwargs: Any) -> None:
        self.node: Node = kwargs['node']
        self.node.create_subscription(
            ResPath,
            'servee/response_path',
            self.callback,
            10
        )
    
    def update(self) -> Status:
        if self.blackboard.robot_state in ["task", "home", "parking"]:
            return Status.SUCCESS
        
        return Status.FAILURE
    
    def callback(self, msg):
        self.node.get_logger().debug(f"msg {msg}")
        self.blackboard.set("path", None)
        self.blackboard.path = msg.path
        
        self.blackboard.waypoint = 0
        self.blackboard.next_pose = msg.path.poses[self.blackboard.waypoint]
        self.blackboard.target_distance = self.calculate_target_distance_absolute()
        
        
        self.blackboard.robot_state = "task"
        
    def calculate_target_distance_absolute(self):
        """
        목적지와 현재 위치의 절대 좌표를 기준으로 거리를 계산한다.
        """
        goal_position = np.array([self.blackboard.next_pose.position.x, self.blackboard.next_pose.position.y])
        curr_position = np.array([self.blackboard.curr_pose.position.x, self.blackboard.curr_pose.position.y])
        
        distance_vector = goal_position - curr_position
        distance = np.linalg.norm(distance_vector) 
        
        return distance
                

        
