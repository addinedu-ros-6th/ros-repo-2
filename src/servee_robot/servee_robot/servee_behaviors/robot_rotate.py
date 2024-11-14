import math
from typing import Any
from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access
from geometry_msgs.msg import PoseArray, Twist
from etc.tools.robot_pid_controller import RobotPIDController
from etc.utils.pose_utils import PoseUtils
import numpy as np
from rclpy.time import Time


class RobotRotate(Behaviour):
    def __init__(self, name: str):
        super(RobotRotate, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="robot_state", access=Access.READ)
        self.blackboard.register_key(key="next_pose", access=Access.READ)
        self.blackboard.register_key(key="curr_pose", access=Access.READ)
        self.blackboard.register_key(key="odom_pose", access=Access.READ)
        self.blackboard.register_key(key='path', access=Access.READ)
        self.blackboard.register_key(key='yaw', access=Access.READ)
        
        self.blackboard.register_key(key='waypoint', access=Access.READ) # 디버깅을 위한 용도
        
        self.blackboard.register_key(key='odom_yaw_error', access=Access.READ)
        self.blackboard.register_key(key='odom_yaw_error', access=Access.WRITE)
        
        self.blackboard.register_key(key='max_angular_speed', access=Access.WRITE)
        self.blackboard.register_key(key='max_angular_speed', access=Access.READ)
        self.blackboard.max_angular_speed = 0.8
        
    def setup(self, **kwargs: Any) -> None:
        self.node: Node = kwargs['node']
        self.pid = RobotPIDController(1, 0.3, 0.05)
        
        
        self.max_angular_speed = self.blackboard.max_angular_speed
        self.blackboard.odom_yaw_error = 0.0
        self.absolute_yaw_threshold = math.radians(10) 
        self.smooth_turn_tolerance = math.radians(5)
        self.cmd_vel_publisher = self.node.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        
        self.prev_time = None 
        self.prev_yaw = None  
     
    def reset_values(self):
        self.prev_yaw = None
        self.prev_time = None
                
                
    def update(self) -> Status:

        if self.are_you_ready() == False:
            return Status.FAILURE
               
        
        # waypoin인덱스가 변경될때만 절대좌표로 위치 확인
        if abs(self.blackboard.odom_yaw_error) == 0.0:
            # self.node.get_logger().warn(f"waypoint:  {self.blackboard.waypoint}, next: {self.blackboard.next_pose}, curr: {self.blackboard.curr_pose}")

            # 목표 방향으로 회전해야 하는 각도 계산
            target_yaw = self.calculate_target_angle()
            yaw_error = target_yaw - self.blackboard.yaw
            yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi
             
            # 오차가 허용범위보다 큰 경우 
            if abs(yaw_error) >= self.absolute_yaw_threshold:
                self.blackboard.odom_yaw_error = yaw_error
                # self.node.get_logger().warn(f"odom_yaw_error 갱신: {self.blackboard.odom_yaw_error:.3f} way {self.blackboard.waypoint}, next {self.blackboard.next_pose}")
                return Status.FAILURE

            else:
                # yaw 오차 없음. 
                self.node.get_logger().warn(f"yaw 오차가 없다구? {self.blackboard.odom_yaw_error:.3f} way {self.blackboard.waypoint}")
                return Status.SUCCESS   
        
        
        # 상대좌표로 계산한 값이 허용범위보다 크다면.        
        else:            
            # 회전하며 self.odom_yaw_error값 줄이기
            current_time = self.node.get_clock().now()

            # prev_time이 None이면 초기화 후 첫 프레임에서의 dt를 0으로 설정
            if self.prev_time is None:
                dt = 0.1
                self.prev_time = current_time
            else:
                dt = (current_time - self.prev_time).nanoseconds * 1e-9  # 초 단위로 변환
            
            if dt > 2:
                dt = 0.1
                
            # 현재 odom yaw 값 계산
            curr_yaw = PoseUtils.get_yaw_from_quaternion(self.blackboard.odom_pose.orientation)
            
            if self.prev_yaw is not None:
                yaw_difference = curr_yaw - self.prev_yaw
                yaw_difference = (yaw_difference + math.pi) % (2 * math.pi) - math.pi
                self.blackboard.odom_yaw_error -= yaw_difference
                
                # self.node.get_logger().warn(f"현재 odom yaw 오차 {self.blackboard.odom_yaw_error}")
         
            angular_speed = max(min(self.pid.compute(self.blackboard.odom_yaw_error, dt), self.max_angular_speed), -self.max_angular_speed)
            
                        
            self.prev_yaw = curr_yaw
            self.prev_time = current_time
            
            
            # 제자리 회전
            if abs(self.blackboard.odom_yaw_error) > self.smooth_turn_tolerance:
                # self.node.get_logger().warn(f"너무 큰 오차 {self.blackboard.odom_yaw_error:.3f} way {self.blackboard.waypoint}")
                self.twist_publish(angular_speed)
                return Status.FAILURE
            
            # 선회하며 회전
            else:
                # self.node.get_logger().warn(f"작은 오차 {self.blackboard.odom_yaw_error:.3f} way {self.blackboard.waypoint}")
                self.twist_publish(angular_speed, 0.05)
                return Status.SUCCESS
                
    
    def twist_publish(self, angular_speed, linear_speed = 0.0):
        cmd_msg = Twist()
        cmd_msg.linear.x = linear_speed
        cmd_msg.angular.z = angular_speed
        self.cmd_vel_publisher.publish(cmd_msg)
    
    
    def calculate_target_angle(self):
        curr_pose_x = self.blackboard.curr_pose.position.x
        curr_pose_y = self.blackboard.curr_pose.position.y
        next_pose_x = self.blackboard.next_pose.position.x
        next_pose_y = self.blackboard.next_pose.position.y
        
        error = math.atan2(next_pose_y - curr_pose_y, next_pose_x - curr_pose_x)
        
        # self.node.get_logger().info(f"현재 위치: {self.blackboard.curr_pose} 목표위치: {self.blackboard.next_pose} 바라봐야 하는 방향(ra): {error}")
        
        return error
        
        
    def are_you_ready(self):
        # 경로가 없는 경우
        if self.blackboard.exists('path') == False:
            self.node.get_logger().error(f"경로가 없습니다.: rotate")
            return False
        
        # 로봇이 이동 상태가 아닌 경우
        if self.blackboard.robot_state not in ['task', 'home', 'aruco']:
            return False