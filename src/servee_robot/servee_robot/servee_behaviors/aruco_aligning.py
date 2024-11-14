import math
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access
from geometry_msgs.msg import Twist, Pose

class ArucoAligning(Behaviour):
    """
    aruco marker 의 법선와 로봇의 시선으로 작도한 직각삼각형의 내접원의 중심으로 이동
    """
    
    def __init__(self, name:str):
        super(ArucoAligning, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="marker_detected", access=Access.READ)
        self.blackboard.register_key(key="aruco_maker_result", access=Access.READ)
        self.blackboard.register_key(key="yaw", access=Access.READ)
        self.blackboard.register_key(key="path", access=Access.READ)
        self.blackboard.register_key(key="path", access=Access.WRITE)
        self.blackboard.register_key(key="curr_pose", access=Access.READ)
        self.blackboard.register_key(key="odom_pose", access=Access.READ)
        
        self.blackboard.register_key(key="robot_state", access=Access.WRITE)
        self.blackboard.register_key(key='aruco_state', access=Access.WRITE)
        self.blackboard.register_key(key='aruco_state', access=Access.READ)
        
     
    def setup(self, **kwargs: Any) -> None:
        self.node:Node = kwargs['node']   
        self.distance_tolerance = 0.2
        self.yaw_tolerance = 0.3
        self.initial_position = None
        self.initial_yaw = None
        self.ang_vel = 0.3
        self.lin_vel = 0.08
        self.twist_pub = \
            self.node.create_publisher(
                Twist, 
                '/base_controller/cmd_vel_unstamped',
                10)
           
        
    def set_marker_data(self) -> None:
        self.marker_id = self.blackboard.aruco_maker_result['id']
        self.marker_x = self.blackboard.aruco_maker_result['x']
        self.marker_z = self.blackboard.aruco_maker_result['z'] / 2
        self.marker_yaw = self.blackboard.aruco_maker_result['yaw']
        self.marker_centerline_error = self.blackboard.aruco_maker_result['centerline_error']
        self.marker_theta = math.atan2(self.marker_z, self.marker_x)
        
        
    def update(self) -> Status:
        
        if self.blackboard.aruco_state != "align":
            return Status.SUCCESS
        
        if self.blackboard.marker_detected:
            self.set_marker_data()
            
            # 거리
            if math.hypot(self.marker_x, self.marker_z) < self.distance_tolerance:
                self.blackboard.aruco_state = "yawing"
                self.node.get_logger().fatal(f"Yawing 상태로 넘어갑니다. {math.hypot(self.marker_x, self.marker_z)}")
                return Status.FAILURE
                
            # 방향
            elif abs(self.marker_yaw) < self.yaw_tolerance:
                self.blackboard.aruco_state = "approach"
                self.node.get_logger().fatal(f"approach 상태로 넘어갑니다.{self.marker_yaw}")
                return Status.FAILURE
                
            # 마커와 정렬되지 않은 경우 회전해서 각도 조정.
            else:
                
                distance = (self.marker_x * self.marker_z) / (
                        (math.sin(self.marker_theta / 2)) * (self.marker_x + self.marker_z + math.hypot(self.marker_x, self.marker_z)))
                
                curr_x = self.blackboard.curr_pose.position.x
                curr_y = self.blackboard.curr_pose.position.y
                
                goal_pose = Pose()
                goal_pose.position.x = curr_x + distance * math.cos(self.marker_theta)
                goal_pose.position.y = curr_y + distance * math.sin(self.marker_theta)
                self.robot_move(goal_pose.position.x, goal_pose.position.y)
                
                # path = self.blackboard.path
                # path.poses.append(goal_pose)
                # self.blackboard.path = path
                # self.blackboard.aruco_state = "search"
                # self.blackboard.robot_state = "parking"
                
                self.node.get_logger().fatal(f"마커와 정렬되지 않아 다시 이동합니다.{distance}, {math.hypot(self.marker_x, self.marker_z)}, {abs(self.marker_yaw)}")
                    
                    
        else:
            self.blackboard.aruco_state = "search"
            self.node.get_logger().info("Marker not found. Returning to Searching.")
    
        
    def robot_move(self, angle, distance):
        twist = Twist()
        
        if self.initial_yaw is None:
            self.initial_yaw = self.blackboard.yaw
            
        yaw_error = angle - (self.blackboard.yaw - self.initial_yaw)
        
        if abs(yaw_error) > 0.1:
            twist.angular.z = self.ang_vel if yaw_error > 0 else -self.ang_vel
        else:
            twist.angular.z = 0.0
            self.initial_yaw = None  # Reset for next rotation
            
        curr_x = self.blackboard.odom_pose.position.x
        curr_y = self.blackboard.odom_pose.position.y
        
        if self.initial_position is None:
            self.initial_position = (curr_x, curr_y)
            
            
        # 현재 위치와 초기 위치 간의 변위를 계산합니다.
        dx = curr_x - self.initial_position[0]
        dy = curr_y - self.initial_position[1]
        dist_moved = math.hypot(dx, dy)
        
        # 목표 거리(target_distance)까지 이동이 완료되지 않은 경우 전진합니다.
        if dist_moved < distance:
            twist.linear.x = self.lin_vel
                
        # 목표 거리에 도달한 경우 이동을 멈추고 초기 위치를 재설정합니다.
        else:
            twist.linear.x = 0.0
            self.initial_position = None  # 다음 이동을 위해 초기 위치 초기화
            
        self.twist_pub.publish(twist)