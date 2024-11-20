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
        # self.blackboard.register_key(key="odom_pose", access=Access.READ)
        
        self.blackboard.register_key(key="aruco_id_index", access=Access.READ)
        self.blackboard.register_key(key="aruco_ids", access=Access.READ)
        
        self.blackboard.register_key(key='aruco_state', access=Access.WRITE)
        self.blackboard.register_key(key='aruco_state', access=Access.READ)
        
        
     
     
    def setup(self, **kwargs: Any) -> None:
        self.node:Node = kwargs['node']   

        self.yaw_tolerance = 30
        self.initial_position = None
        self.initial_yaw = None
        self.ang_vel = 0.3
        self.lin_vel = 0.08
        self.twist_pub = \
            self.node.create_publisher(
                Twist, 
                '/base_controller/cmd_vel_unstamped',
                10)
            
        self.node.declare_parameter(
            'distance_tolerance_aligning', [0.35, 0.35, 0.35, 0.35, 0.70, 0.70, 0.70, 0.70, 0.80, 0.80, 0.80]
        )
        
        self.distance_tolerance_array = self.node.get_parameter('distance_tolerance_aligning').value
        
        # self.node.get_logger().error(f"aligning self.distance_tolerance : {self.distance_tolerance_array}")

        
    def set_marker_data(self) -> None:
        self.marker_id = self.blackboard.aruco_maker_result['id']
        self.marker_x = self.blackboard.aruco_maker_result['x']
        self.marker_z = self.blackboard.aruco_maker_result['z'] / 2
        self.marker_yaw = self.blackboard.aruco_maker_result['yaw']
        self.marker_centerline_error = self.blackboard.aruco_maker_result['centerline_error']
        self.marker_theta = math.atan2(self.marker_z, self.marker_x)
        
        
    def update(self) -> Status:
        
        if self.blackboard.aruco_state != "align":
            return Status.FAILURE
        
        if self.blackboard.marker_detected:
            self.set_marker_data()
            
            self.node.get_logger().error(f"index{self.blackboard.aruco_id_index}")
            self.node.get_logger().error(f"aruco_ids: {self.blackboard.aruco_ids}")
            
            aruco_id = 0
            aruco_id = self.blackboard.aruco_ids[self.blackboard.aruco_id_index] -5
            self.node.get_logger().error(f"aruco_id: {aruco_id}")
            self.node.get_logger().error(f"distance_tolerance_array {self.distance_tolerance_array}")
            self.distance_tolerance = self.distance_tolerance_array[aruco_id]
            
            
            
            # 거리
            if math.hypot(self.marker_x, self.marker_z) < self.distance_tolerance:
                self.blackboard.aruco_state = "yawing"
                self.node.get_logger().fatal(f"Yawing 상태로 넘어갑니다. {math.hypot(self.marker_x, self.marker_z)}")
                return Status.FAILURE
                
            # # # 방향
            # elif abs(self.marker_yaw) < self.yaw_tolerance:
            #     self.blackboard.aruco_state = "approach"
            #     self.node.get_logger().fatal(f"approach 상태로 넘어갑니다.{self.marker_yaw}")
            #     return Status.FAILURE
                
            # 마커와 정렬되지 않은 경우 회전해서 각도 조정.
            else:
                
                # self.rotate_by_angle(-self.marker_theta / 2)
                distance = (self.marker_x * self.marker_z) / (
                    (math.sin(self.marker_theta / 2)) * (self.marker_x + self.marker_z + math.hypot(self.marker_x, self.marker_z)))
                
                self.move_by_distance(distance*1.5)
                # self.rotate_by_angle(np.sign(self.marker_theta) * math.pi / 4)
                self.node.get_logger().fatal(f"마커와 정렬되지 않아 다시 이동합니다. 거리: {distance} 좌표:{self.marker_x}, {self.marker_z}, yaw {abs(self.marker_yaw)}")
                return Status.SUCCESS
                    
        else:
            self.blackboard.aruco_state = "search"
            self.node.get_logger().info("Marker not found. Returning to Searching.")
            return Status.SUCCESS
    
    
    def rotate_by_angle(self, target_angle):
        
        twist = Twist()
        """
        odometry를 이용해서 target_angle 만큼 회전한다.
        """
        if self.initial_yaw is None:
            self.initial_yaw = self.blackboard.yaw
        yaw_error = target_angle - (self.blackboard.yaw- self.initial_yaw)
        
        if abs(yaw_error) > 0.1:
            twist.angular.z = self.ang_vel if yaw_error > 0 else -self.ang_vel
            self.twist_pub.publish(twist)

            
        else:
            twist.angular.z = 0.0
            self.twist_pub.publish(twist)
            self.initial_yaw = None 
           
    
            
    def move_by_distance(self, target_distance):
        """
        odometry를 이용해서 target_distance 만큼 직진한다. 
        """
        twist = Twist()
        
        curr_x = self.blackboard.curr_pose.position.x
        curr_y = self.blackboard.curr_pose.position.y
        
        # 초기 위치를 설정
        if self.initial_position is None:
            self.initial_position = (curr_x, curr_y)
           
        
            
        # 현재 위치와 초기 위치 간의 변위를 계산
        dx = curr_x - self.initial_position[0]
        dy = curr_y - self.initial_position[1]
        
        dist_moved = math.hypot(dx, dy)
        
        # 목표 거리(target_distance)까지 이동이 완료되지 않은 경우 전진
        if dist_moved < target_distance:
            twist.linear.x = self.lin_vel
            self.twist_pub.publish(twist)
            self.node.get_logger().fatal(f"속도: {self.lin_vel}")  


        # 목표 거리에 도달한 경우
        else:
            twist.linear.x = 0.0
            self.twist_pub.publish(twist)
            self.initial_position = None  # 다음 이동을 위해 초기 위치 초기화
        