from typing import Any
import rclpy
from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access
from geometry_msgs.msg import Pose
from std_msgs.msg import String

class RobotDataSender(Behaviour):
    def __init__(self, name:str):
        super(RobotDataSender, self).__init__(name)
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="robot_state", access=Access.READ)
        self.blackboard.register_key(key="curr_pose", access=Access.READ)
        
    def setup(self, **kwargs):
        self.node : Node = kwargs['node']
        
        self.publisher_pose = self.node.create_publisher(
            Pose,
            '/servee/get_pose',
            10
        )
        
        self.publisher_robot_state = self.node.create_publisher(
            String,
            '/servee/get_state',
            10
        )
        
    def update(self):
        robot_state_data = String()
        robot_state_data.data = self.blackboard.robot_state
        
        curr_pose = Pose()
        curr_pose.pose = self.blackboard.curr_pose
        
        self.publisher_pose.publish(curr_pose)
        self.publisher_robot_state.publish(robot_state_data)
        
        return Status.SUCCESS