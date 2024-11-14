import math
from typing import Any
import rclpy
from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access
from geometry_msgs.msg import Twist

class ArucoSearch(Behaviour):
    """
    아르코 마커를 찾고 로봇과 마커를 정렬시키는 액션
    1. 회전하며 아루코 마커를 탐색
    2. 탐색 후 정렬 좌표를 블랙보드에 전달.
    """
    
    def __init__(self, name:str):
        super(ArucoSearch, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        
        self.blackboard.register_key(key="marker_detected", access=Access.READ)
        self.blackboard.register_key(key="aruco_maker_result", access=Access.READ)
        self.blackboard.register_key(key='max_angular_speed', access=Access.READ)
        
        self.blackboard.register_key(key="robot_state", access=Access.WRITE)
        
        self.blackboard.register_key(key='aruco_state', access=Access.WRITE)
        self.blackboard.register_key(key='aruco_state', access=Access.READ)
        
        self.blackboard.aruco_state = "search"
        
        
    def setup(self, **kwargs: Any) -> None:
        
        self.node:Node = kwargs['node']
        self.ang_vel = (self.blackboard.max_angular_speed - 0.35)
        self.rotation_count = 0
        self.centerline_error_tolerance = 50

        self.twist_pub = \
            self.node.create_publisher(
                Twist, 
                '/base_controller/cmd_vel_unstamped',
                10)

   
    def set_marker_data(self):
        self.marker_id = self.blackboard.aruco_maker_result['id']
        self.marker_x = self.blackboard.aruco_maker_result['x']
        self.marker_z = self.blackboard.aruco_maker_result['z'] / 2
        self.marker_yaw = self.blackboard.aruco_maker_result['yaw']
        self.marker_centerline_error = self.blackboard.aruco_maker_result['centerline_error']
        self.marker_theta = math.atan2(self.marker_z, self.marker_x)
   
    
    def update(self) -> Status:
        
        if self.blackboard.aruco_state != "search":
            return Status.FAILURE
        
        self.node.get_logger().warn(f"아루코 마커를 찾아서! {self.blackboard.marker_detected}")
        
        # 마커가 검출된 동안 반복하여 로봇을 회전시켜 정렬을 시도합니다.
        if  self.blackboard.marker_detected == True:
            self.set_marker_data()
            # self.node.get_logger().warn(f"마커: {self.marker_id }")
            
            # 마커와 로봇이 정면으로 마주보도록 회전한다.
            if abs(self.marker_centerline_error) > self.centerline_error_tolerance:
                twist = Twist()
                twist.angular.z = -self.ang_vel if self.marker_centerline_error > 0 else self.ang_vel
                self.twist_pub.publish(twist)
                # self.node.get_logger().warn(f"마커를 정면에서 보게 한다. {self.rotation_count}")
                return Status.SUCCESS

            else:
                self.blackboard.aruco_state = "align"
                self.rotation_count = 0
                twist = Twist()
                twist.angular.z = 0.0
                self.twist_pub.publish(twist)
                self.node.get_logger().fatal(f"마커를 정면에서 보는 중 {self.rotation_count}")
                return Status.FAILURE
            
        # 마커가 안 보인다면 회전하며 마커를 찾는다.  
        else:
            self.node.get_logger().warn("마커 탐색 중")
            twist = Twist()
            twist.angular.z = self.ang_vel
            self.twist_pub.publish(twist)
            self.rotation_count += 1
            # self.node.get_logger().warn(f"마커를 찾는 중 {self.rotation_count}")
            # 시도를 했지만 결국 마커를 못 찾는 경우
            
            if self.rotation_count >= 50:  
                twist.angular.z = 0.0
                self.twist_pub.publish(twist)
                self.rotation_count = 0
                
                self.blackboard.robot_state = "task"
                self.node.get_logger().fatal("결국 마커를 찾지 못 했습니다.")
                
                
            return Status.SUCCESS
        
        
            