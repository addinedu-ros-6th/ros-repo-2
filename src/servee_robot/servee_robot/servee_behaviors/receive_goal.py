from typing import Any
import rclpy
from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access
from servee_interfaces.msg import TaskGoalData

class ReceiveGoal(Behaviour):
    
    def __init__(self, name:str):
        super(ReceiveGoal, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="goal_pose", access=Access.WRITE)
        self.blackboard.register_key(key="robot_state", access=Access.READ)
        self.blackboard.register_key(key="robot_state", access=Access.WRITE)
        self.blackboard.register_key(key="aruco_ids", access=Access.WRITE)
        self.blackboard.register_key(key="aruco_id_index", access=Access.WRITE)
        
    def setup(self, **kwargs: Any) -> None:
        self.node: Node = kwargs['node']
        self.node.create_subscription(
            TaskGoalData,
            '/servee/task_goal_data',
            self.callback,
            10
        )
        
    def update(self) -> Status:
        if self.blackboard.robot_state in ["idle", "receive_goal"]:
            return Status.SUCCESS
        
        else:
            return Status.RUNNING
        
    def callback(self, msg):    
        self.blackboard.goal_poses = msg.goal_poses
        self.blackboard.aruco_ids = msg.aruco_id
        self.blackboard.aruco_id_index = 0
        
        self.node.get_logger().info(f"배열로 받은 목적지 {self.blackboard.goal_poses}")
        
        self.blackboard.goal_pose = self.blackboard.goal_poses.poses[0]
        self.blackboard.robot_state = "receive_goal"
        
        # self.blackboard.goal_pose = msg.goal_pose
        # self.blackboard.robot_state = "receive_goal"
        
        