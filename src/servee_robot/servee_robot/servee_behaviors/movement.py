import math
from typing import Any
from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access
from geometry_msgs.msg import PoseArray, Twist
from etc.tools.robot_pid_controller import RobotPIDController
import numpy as np


class Movement(Behaviour):
    def __init__(self, name: str):
        super(Movement, self).__init__(name)
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="robot_state", access=Access.WRITE)
        self.blackboard.register_key(key="robot_state", access=Access.READ)
        self.blackboard.register_key(key="curr_pose", access=Access.READ)
        self.blackboard.register_key(key="path", access=Access.READ)
        self.blackboard.register_key(key="yaw", access=Access.READ)
        self.blackboard.register_key(key="goal_pose", access=Access.WRITE)
        
    def setup(self, **kwargs: Any) -> None:
        self.node:Node = kwargs['node']
        self.pid = RobotPIDController(kp=1.0, ki=0.3, kd=0.05)
        self.cmd_vel_publisher = self.node.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.robot_state = self.blackboard.robot_state
        self.distance_tolerance = 0.2 # 목표 허용 오차. 추후에 파라미터로 뺄 것
        self.max_angular_speed = 0.25   # 추후에 파라미터로 뺄 것
        self.max_linear_speed = 0.1    # 추후에 파라미터로 뺄 것
        self.curr_position = np.array([0.0, 0.0])
        self.target_position = np.array([0.0, 0.0])
        self.current_yaw = 0.0
        self.angle_threshold = math.radians(50)
        self.reset_values()

    def reset_values(self):
        self.path = PoseArray()
        self.waypoint_index = 0

        
    def initialise(self) -> None:
        
        if self.are_you_ready() == False:
            return 
                
        if self.blackboard.exists('path'):    
            self.path = self.blackboard.path
              
 
    def update(self) -> Status:
       
        if self.are_you_ready() == False:
            return Status.FAILURE
        
        self.node.get_logger().error(f"curr: {self.curr_position}, goal: {self.target_position}")
        
        goal_pose = self.path.poses[self.waypoint_index]
        curr_pose = self.blackboard.curr_pose
        self.target_position = np.array([goal_pose.position.x, goal_pose.position.y])
        self.curr_position = np.array([curr_pose.position.x, curr_pose.position.y])
        self.current_yaw = self.blackboard.yaw
        
        # 완료 여부 체크
        self.waypoint_reached()
        if self.is_complete_path():
            self.blackboard.robot_state = "idle"
            self.go_to_pose(0.0, 0.0)
            self.reset_values()

            # del self.blackboard.goal_pose
            # del self.blackboard.path
            return Status.SUCCESS
        
        dt = 0.2 # 임시 테스트 후 변경.
        
        # 목표 좌표와의 방향 오차 계산.
        angle_error = self.get_target_errors()
        angular_speed = max(min(self.pid.compute(angle_error, dt), self.max_angular_speed), -self.max_angular_speed)
        
        
        # 급 커브인지 확인하고 제자리 회전 적용
        if abs(angle_error) > self.angle_threshold:
            # 급 커브의 경우 제자리 회전
            linear_speed = 0.0
        else:
            # 급 커브가 아닌 경우 일반 이동
            linear_speed = self.max_linear_speed
        
        self.go_to_pose(linear_speed, angular_speed)
        return Status.FAILURE
        



        






    
    def go_to_pose(self, linear_speed, angular_speed):
        cmd_msg = Twist()
        cmd_msg.linear.x = linear_speed
        cmd_msg.angular.z = angular_speed
        self.cmd_vel_publisher.publish(cmd_msg)
    
    
    def get_distance_error(self):
        # 목표 방향 벡터 계산
        target_vector = self.target_position - self.curr_position # 목표 방향 계산
        distance_error = np.linalg.norm(target_vector)  # 벡터크기 계산
        return distance_error

    
    def get_target_errors(self):
        target_vector = self.target_position - self.curr_position # 목표 방향 계산
        
        # 각도 오차 계산
        target_yaw = math.atan2(target_vector[1], target_vector[0])
        angle_error = target_yaw - self.current_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        return angle_error
    
    
    def waypoint_reached(self):
        distance_error = self.get_distance_error()
        if distance_error <= self.distance_tolerance:
            self.node.get_logger().error(f"인덱스 업")
            self.waypoint_index += 1
            
    def is_complete_path(self):
        """
        waypoint_index가 path의 마지막 인덱스인지 확인한다.
        Returns:
            Path(Bool): Path 완료 여부
        """
        if self.waypoint_index >= len(self.path.poses): 
            self.node.get_logger().error(f"총 {len(self.path.poses)} 목적지 도달 {self.waypoint_index}")
            return True
        else:
            return False


    def are_you_ready(self):
        """
        이동을 할 수 있는 상태인지 체크한다.
        1. 경로를 받아왔는지?
        2. robot_state가 move 인지?
        """
        if self.blackboard.exists('path') == False:
            return False
                
        if self.blackboard.robot_state not in ["task", "home"]:
            return False
        
        return True

    
    
        
        

    
    
    
    
    # def go_to_pose(self):
    #     pose = self.path.poses[self.waypoint_index]
    #     yaw = PoseUtils.get_yaw_from_quaternion(pose.orientation)
    #     timeStemp = self.node.get_clock().now().to_msg()
    #     goal_pose = PoseUtils.create_pose_stamped(pose.position.x, pose.position.y, yaw, timeStemp)
    #     self.navigator.goToPose(goal_pose)

    # 이동이 완료된 상태
    # if self.navigator.isTaskComplete():
        
    #     # Path 완료 여부 체크
    #     if self.is_complete_path():
    #         self.blackboard.robot_state = "idle"
    #         self.reset_values()
    #         return Status.SUCCESS
        
    #     # 다음 웨이포인트로 이동.
    #     else:
    #         self.waypoint_index += 1
    #         self.go_to_pose()
    
    # # 이동 중인 상태 
    # else:             
    #     return Status.FAILURE