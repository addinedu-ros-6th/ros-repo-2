import math
from typing import Any

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access
from geometry_msgs.msg import Twist, Pose

class ArucoYawing(Behaviour):
    def __init__(self, name:str):
        super(ArucoYawing, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key='aruco_state', access=Access.WRITE)
        self.blackboard.register_key(key='aruco_state', access=Access.READ)
        self.blackboard.register_key(key="robot_state", access=Access.WRITE)
        
        self.blackboard.register_key(key='scan', access=Access.READ)
        
        
    def setup(self, **kwargs):
        self.node:Node = kwargs['node'] 
        self.twist_pub = \
            self.node.create_publisher(
                Twist, 
                '/base_controller/cmd_vel_unstamped',
                10)
            
        self.node.declare_parameter('closest_line_angle_tolerance', 0.02)
        self.closest_line_angle_tolerance = self.node.get_parameter('closest_line_angle_tolerance').get_parameter_value().double_value
        self.closest_line_distance =100
        self.closest_line_angle = 100
        self.scale = 700  # 기본값 설정
        self.hough_threshold = 60
        self.ang_vel = 0.3
        self.lin_vel = 0.08
        
    def update(self):
        
        self.detect_lines()
        self.yawing()
    
    def detect_lines(self):
        msg = self.blackboard.scan
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        
        # 유효한 거리 값 필터링
        valid_indices = np.isfinite(ranges)
        angles = angles[valid_indices]
        ranges = ranges[valid_indices]
        
        # 극좌표를 직교좌표로 변환
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        
        # 라인 검색
        img_size = 500
        img = np.zeros((img_size, img_size), dtype=np.uint8)
        offset = img_size // 2

        # (x, y) 좌표들을 수직으로 쌓아서 포인트 배열을 생성
        points = np.vstack((x, y)).T
        points = np.int32(points * self.scale + offset)  # 스케일 조정 및 오프셋 적용
        for point in points:
            if 0 <= point[0] < img_size and 0 <= point[1] < img_size:
                cv2.circle(img, tuple(point), 1, 255, -1)  # 해당 위치에 흰색 점을 그림
        
        # 허프 변환을 사용하여 직선을 검출합니다.
        lines = cv2.HoughLines(img, 1, np.pi / 180, self.hough_threshold)

        if lines is not None:
            adjusted_lines = []
            
            # 검출된 각 직선에 대해 처리
            for line in lines:
                rho, theta = line[0]
                
                # 각도 범위를 -π/2 ~ π/2로 조정
                if theta > np.pi / 2:
                    theta -= np.pi
                elif theta < -np.pi / 2:
                    theta += np.pi
                adjusted_lines.append([rho, theta])

            # 특정 각도 범위(-1 ~ 1 라디안) 내의 직선만 필터링
            filtered_lines = [line for line in adjusted_lines if -1 <= line[1] <= 1]

            if filtered_lines:
                # 가장 가까운 직선을 찾습니다.
                closest_line = min(filtered_lines, key=lambda line: abs(line[0]))
                self.closest_line_distance = closest_line[0] # 직선까지의 거리 저장
                self.closest_line_angle = closest_line[1]    # 직선의 각도 저장
                self.node.get_logger().fatal(f"closest_line_angle: {self.closest_line_angle}, closest_line_distance: {self.closest_line_distance}")
        
    
    def yawing(self):
        """Stop at the target distance and finish the yawing."""

        twist = Twist()
        
         # 라이다로 검출한 벽의 각도가 허용 오차를 초과하는 경우, 각도를 조정합니다.
        if abs(self.closest_line_angle) > self.closest_line_angle_tolerance:
            twist.angular.z = self.ang_vel * np.sign(self.closest_line_angle)
            self.node.get_logger().fatal(f"yawing 이동: {twist.angular.z}")
            self.twist_pub.publish(twist)
            return Status.FAILURE
            
        else:
            # 각도가 허용 오차 이내라면 회전을 멈추고 상태를 'STANDBY'로 전환합니다.
            twist.angular.z = 0.0
            self.twist_pub.publish(twist)
            self.node.get_logger().fatal("주차 완료")
            self.blackboard.aruco_state = 'search'
            self.blackboard.robot_state = 'idle'
            return Status.SUCCESS
            
    