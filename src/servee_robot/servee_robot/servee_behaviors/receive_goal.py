from typing import Any
import rclpy
from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access
from servee_interfaces.msg import TaskGoalData
from geometry_msgs.msg import Pose

class ReceiveGoal(Behaviour):
    
    def __init__(self, name:str):
        super(ReceiveGoal, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="goal_poses", access=Access.WRITE)
        self.blackboard.register_key(key="goal_pose", access=Access.WRITE)
        self.blackboard.register_key(key="robot_state", access=Access.READ)
        self.blackboard.register_key(key="robot_state", access=Access.WRITE)
        
        self.blackboard.register_key(key="aruco_ids", access=Access.WRITE)
        self.blackboard.register_key(key="aruco_id_index", access=Access.WRITE)
        self.blackboard.register_key(key="home_aruco_id", access=Access.READ)
        
        self.blackboard.register_key(key='odom_yaw_error', access=Access.WRITE)
        self.blackboard.register_key(key="home_pose", access=Access.READ)
        
    def setup(self, **kwargs: Any) -> None:
        self.node: Node = kwargs['node']
        self.node.create_subscription(
            TaskGoalData,
            '/robot1/task_goal_data',
            self.callback,
            10
        )
        
    def update(self) -> Status:
        if self.blackboard.robot_state in ["receive_goal"]:
            return Status.SUCCESS
        
        else:
            return Status.RUNNING
        
    def callback(self, msg): 
        # self.node.get_logger().error(f"목표좌표: {msg.data}")
        goal_poses = msg.goal_poses  # PoseArray 타입이라고 가정

        # home_pose를 Pose 객체로 추가
        home_pose = self.blackboard.home_pose

        # PoseArray의 poses 필드에 추가
        goal_poses.poses.append(home_pose)

        
        
        aruco_ids = msg.aruco_id
        aruco_ids.append(self.blackboard.home_aruco_id)
        
        self.blackboard.goal_poses = goal_poses
        self.blackboard.aruco_ids = aruco_ids

        self.blackboard.aruco_id_index = 0
        self.blackboard.odom_yaw_error = 0.0
        self.node.get_logger().error(f"배열로 받은 목적지 {self.blackboard.goal_poses}")
        
        self.blackboard.goal_pose = self.blackboard.goal_poses.poses[0]
        self.blackboard.robot_state = "receive_goal"

        # self.blackboard.goal_pose = msg.goal_pose
        # self.blackboard.robot_state = "receive_goal"
        
        