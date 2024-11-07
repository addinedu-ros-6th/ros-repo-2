from typing import Any
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

class ResponsePath(Behaviour):
    def __init__(self, name: str):
        super(ResponsePath, self).__init__(name)
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="path", access=Access.WRITE)
        self.blackboard.register_key(key="robot_state", access=Access.WRITE)
        
    def setup(self, **kwargs: Any) -> None:
        self.node = kwargs['node']
        self.node.create_subscription(
            ResPath,
            'servee/response_path',
            self.callback,
            10
        )
    
    def update(self) -> Status:
        if self.blackboard.robot_state == "task":
            return Status.SUCCESS
        return Status.RUNNING
    
    def callback(self, msg):
        self.node.get_logger().debug(f"msg {msg}")
        self.blackboard.path = msg.path
        self.blackboard.robot_state = "task"
        
                

        
