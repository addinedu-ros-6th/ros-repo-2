import math
from typing import Any
from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access
from geometry_msgs.msg import PoseArray, Twist
from etc.tools.robot_pid_controller import RobotPIDController
import numpy as np


class WaypointArrivalChecker(Behaviour):
    def __init__(self, name: str):
        super(WaypointArrivalChecker, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="next_pose", access=Access.READ)
        self.blackboard.register_key(key="next_pose", access=Access.WRITE)
        self.blackboard.register_key(key="odom_pose", access=Access.READ)
        self.blackboard.register_key(key="curr_pose", access=Access.READ)
        
        self.blackboard.register_key(key="robot_state", access=Access.READ)
        self.blackboard.register_key(key="robot_state", access=Access.WRITE)
        self.blackboard.register_key(key="waypoint", access=Access.WRITE)
        self.blackboard.register_key(key="waypoint", access=Access.READ)
        
        self.blackboard.register_key(key="target_distance", access=Access.READ)
        self.blackboard.register_key(key="target_distance", access=Access.WRITE)
        
        self.blackboard.register_key(key="path", access=Access.READ)
        self.blackboard.register_key(key='odom_yaw_error', access=Access.WRITE)
        
        self.blackboard.target_distance = 0.0
        self.blackboard.waypoint = 0
        self.blackboard.robot_state = "idle"
        
        
    def setup(self, **kwargs: Any) -> None:
        self.node: Node = kwargs['node']
        self.tolerance_distance = 0.2 # 허용 거리
        self.path = []

    def update_next_waypoint_info(self):
        
        if self.blackboard.waypoint>= len(self.path.poses) -1: 
            # Path 완료 
            # 추후에 path가 array로 전달되면 다 돌았는지 체크한 뒤에 parking으로 돌려야 한다.
            self.blackboard.robot_state = "aruco" 
            self.node.get_logger().warn(f"path 완료 {self.blackboard.robot_state}")
            
        else:
            # 다음 웨이포인트로
            
            self.blackboard.waypoint += 1
            self.blackboard.odom_yaw_error = 0.0
            self.blackboard.next_pose = self.path.poses[self.blackboard.waypoint]
            self.blackboard.target_distance = self.calculate_target_distance_absolute()
            self.node.get_logger().warn(f"다음 Waypoint로 {self.blackboard.waypoint}")
  
            
    def update(self) -> Status:
    
        # 주차중 상태라면 빠르게 tree를 건너 뛴다.
        if self.blackboard.robot_state == "aruco":
            return Status.SUCCESS
    
        # 경로가 없는 경우
        if self.blackboard.exists('path') == False:
            return Status.FAILURE
        
        # 로봇이 이동 상태가 아닌 경우
        if self.blackboard.robot_state not in ['task', 'home', 'parking']:
            return Status.FAILURE
        
        # self.node.get_logger().warn(f"way {self.blackboard.waypoint}, next: {self.blackboard.next_pose}")
            
        self.path = self.blackboard.path
        
        # 로컬좌표로 계산된 타겟과의 거리과 오차 허용 범위 안이라면 
        if self.blackboard.target_distance <= self.tolerance_distance:
            
            # 절대 좌표로도 확인해보고 
            distance_error = self.calculate_target_distance_absolute()
            self.node.get_logger().warn(f"허용 범위 체크 {distance_error}")
            
            # 허용 범위 안 이라면. 
            if distance_error <=  self.tolerance_distance:
                # self.node.get_logger().warn(f"허용 범위 체크 {self.distance_error}")
                self.update_next_waypoint_info()
                
            # 허용 범위 밖이라면 더 이동시킨다
            else:
                # self.node.get_logger().warn(f"허용 범위 밖 {self.distance_error}")
                self.blackboard.target_distance = distance_error
        
        return Status.SUCCESS
    
    
    def calculate_target_distance_absolute(self):
        """
        목적지와 현재 위치의 절대 좌표를 기준으로 거리를 계산한다.
        """
        goal_position = np.array([self.blackboard.next_pose.position.x, self.blackboard.next_pose.position.y])
        curr_position = np.array([self.blackboard.curr_pose.position.x, self.blackboard.curr_pose.position.y])
        
        distance_vector = goal_position - curr_position
        distance = np.linalg.norm(distance_vector) 
        
        return distance
        
        
    
    