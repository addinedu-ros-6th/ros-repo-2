import math
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access
from geometry_msgs.msg import Twist, Pose

class ArucoApproaching(Behaviour):
    def __init__(self, name:str):
        super(ArucoApproaching, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="yaw", access=Access.READ)
        self.blackboard.register_key(key="curr_pose", access=Access.READ)
        
        self.blackboard.register_key(key='aruco_state', access=Access.WRITE)
        self.blackboard.register_key(key='aruco_state', access=Access.READ)
        self.blackboard.register_key(key="robot_state", access=Access.WRITE)
        self.blackboard.register_key(key="marker_detected", access=Access.READ)
        self.blackboard.register_key(key="marker_detected", access=Access.WRITE)
        self.blackboard.register_key(key="aruco_maker_result", access=Access.READ)
          
        self.blackboard.register_key(key="aruco_id_index", access=Access.READ)
        self.blackboard.register_key(key="aruco_ids", access=Access.READ)
        
    def setup(self, **kwargs: Any) -> None:
        self.node:Node = kwargs['node'] 
        self.twist_pub = \
            self.node.create_publisher(
                Twist, 
                '/base_controller/cmd_vel_unstamped',
                10)
        
        self.node.declare_parameter(
            'centerline_error_tolerance_approaching', 130  
        )
        
        self.node.declare_parameter(
            'distance_tolerance_approach', [0.30, 0.30, 0.30, 0.30, 0.65, 0.65, 0.65, 0.65, 0.80, 0.80, 0.80]
        )
        
        self.distance_tolerance_array = self.node.get_parameter(
            'distance_tolerance_approach'
        ).value
        
        self.centerline_error_tolerance = self.node.get_parameter(
            'centerline_error_tolerance_approaching'
        ).value
        
        self.node.get_logger().warn(f"centerline_error_tolerance_approaching: {self.centerline_error_tolerance}")

        self.ang_vel = 0.3
        self.lin_vel = 0.05
        # self.node.get_logger().warn(f"approach self.distance_tolerance : {self.distance_tolerance}")

     
    def set_marker_data(self) -> None:
        self.marker_id = self.blackboard.aruco_maker_result['id']
        self.marker_x = self.blackboard.aruco_maker_result['x']
        self.marker_z = self.blackboard.aruco_maker_result['z'] / 2
        self.marker_yaw = self.blackboard.aruco_maker_result['yaw']
        self.marker_centerline_error = self.blackboard.aruco_maker_result['centerline_error']
        self.marker_theta = math.atan2(self.marker_z, self.marker_x)   
        
        
    def update(self) -> Status:
        if self.blackboard.aruco_state != "approach":
            return Status.FAILURE
        
        self.node.get_logger().fatal("approach")
        self.set_marker_data()
        
        aruco_id = 0
        aruco_id = self.blackboard.aruco_ids[self.blackboard.aruco_id_index] -5
        self.distance_tolerance = self.distance_tolerance_array[aruco_id]
                
        twist = Twist()
        
        # 허용 오처 범위 밖이라면 
        if math.hypot(self.marker_x, self.marker_z) > self.distance_tolerance:
            # self.node.get_logger().fatal(f"거리 체크 {math.hypot(self.marker_x, self.marker_z)}")
            
            
            
            
            twist.linear.x = self.lin_vel
            # # 중심점에서 벗어났다면
            # if abs(self.marker_centerline_error) > self.centerline_error_tolerance:
            #     twist.linear.x = 0.0
            #     twist.angular.z = -self.ang_vel * np.sign(self.marker_centerline_error)
            
            # # 전진
            # else:
            #     twist.linear.x = self.lin_vel
            #     twist.angular.z = 0.0
                
                
            self.twist_pub.publish(twist)
            return Status.SUCCESS

    
        # self.node.get_logger().info("Approaching Aruco marker...")
        
        # 마커와의 거리가 허용 오차 이내인 경우 상태를 'YAWING'으로 전환
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.twist_pub.publish(twist)
            self.node.get_logger().fatal("주차 완료")
            
            self.blackboard.aruco_state = 'search'
            self.blackboard.robot_state = 'standy'
            
            
                
            return Status.FAILURE
        

