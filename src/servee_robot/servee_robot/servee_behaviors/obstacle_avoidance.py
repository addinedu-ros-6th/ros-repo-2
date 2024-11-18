from typing import Any

import numpy as np
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access

from rclpy.node import Node
from geometry_msgs.msg import Twist



class ObstacleAvoidanceMover(Behaviour):
    def __init__(self, name: str):
        super(ObstacleAvoidanceMover, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="scan", access=Access.READ)
        self.blackboard.register_key(key="robot_state", access=Access.READ)
        # self.blackboard.register_key(key="next_pose", access=Access.WRITE)
        
    def setup(self, **kwargs: Any) -> None:
        self.node: Node = kwargs['node']

        # 파라미터 이름 및 기본값 정의
        parameters_to_use = {
            "avoidance_lieaner": 0.1,
            "avoidance_angular": 0.5,
            "wall_threshold": 0.13
        }

        # 파라미터 선언 및 재활용
        for param_name, default_value in parameters_to_use.items():
            if self.node.has_parameter(param_name):
                # 이미 선언된 파라미터를 재활용
                param_value = self.node.get_parameter(param_name).value
                self.node.get_logger().info(f"Reusing parameter '{param_name}' with value: {param_value}")
            else:
                # 파라미터를 선언하고 기본값 설정
                self.node.declare_parameter(param_name, default_value)
                param_value = default_value
                self.node.get_logger().info(f"Declared parameter '{param_name}' with default value: {param_value}")

            # 파라미터 값을 인스턴스 변수로 저장
            setattr(self, param_name, param_value)

        # 최종 파라미터 값 확인
        self.node.get_logger().info(f"lieaner: {self.avoidance_lieaner}, angular: {self.avoidance_angular}, wall_threshold: {self.wall_threshold}")

        # Twist 메시지 퍼블리셔 생성
        self.cmd_vel = self.node.create_publisher(
            Twist,
            '/base_controller/cmd_vel_unstamped',
            10
        )
     
    def update(self) -> Status:
        
        if self.blackboard.robot_state in ['idle', 'aruco']:
            return Status.SUCCESS
        
        self.scan = self.blackboard.scan
        ranges = self.scan.ranges.tolist()
        num_readings = len(ranges)
        
         # 세그먼트 수와 각 세그먼트별 최소 거리 리스트
        num_segments = 4
        min_distances = [float('inf')] * num_segments

        start_angle = -45
        angle_step = 360 // num_segments

        for i in range(num_segments):
            # 각 세그먼트의 시작 각도와 인덱스 계산
            angle = start_angle + i * angle_step
            start_index = int((angle / 360) * num_readings)
            end_index = int(((angle + angle_step) / 360) * num_readings)
            
            # 해당 세그먼트 범위 가져오기
            if i == 0:
                segment = ranges[start_index:] + ranges[:end_index]
            else:
                segment = ranges[start_index:end_index]
            
            # 0.0을 제외한 유효한 거리 값만 필터링
            valid_distances = [dist for dist in segment if dist > 0.0]
            
            # 유효한 거리가 있으면 최소 거리 저장
            if valid_distances:
                min_distances[i] = min(valid_distances)

        # 회피 동작 수행
        if min(min_distances) < self.wall_threshold:
            closest_direction = min_distances.index(min(min_distances))
            # self.node.get_logger().info(f"가까운 벽의 방향 인덱스: {closest_direction}")
            self.avoid(closest_direction)  # 인덱스를 전달하여 회피 동작 수행
            # self.node.get_logger().info(f"가장 가까운 range 인덱스: {valid_distances.index(min(valid_distances))}, 값: {min(valid_distances)}")

            return Status.FAILURE # 회피 해야 함. 
        
        else:
            return Status.SUCCESS # 회피 안해도 됨. 
    
    
    def avoid(self, direction):
        """
        너무 근접한 벽으로부터 도망친다.
        0: 뒤쪽
        1: 오른쪽
        2: 정면
        3: 왼쪽
        """
        # next_pose = self.blackboard.next_pose

        # self.node.get_logger().info(f"회피 힘 앞뒤: {self.avoidance_lieaner}, 좌우 {self.avoidance_angular}, dir: {direction}") 
        void_twist = Twist()
        if direction % 2 == 0:
            # 앞뒤 회피
            # void_twist.linear.x = 0
            # void_twist.linear.x = (self.avoidance_lieaner-0.01) if direction == 0 else -(self.avoidance_lieaner-0.01)
            # self.node.get_logger().info(f"앞 뒤: {direction} : {twist.linear.x}")
            void_twist.angular.z = self.avoidance_angular
            void_twist.linear.x = (self.avoidance_lieaner-0.01) if direction == 0 else -(self.avoidance_lieaner-0.01)
        else:
            # 좌우 회피
            void_twist.angular.z = self.avoidance_angular if direction == 1 else -self.avoidance_angular
            void_twist.linear.x = self.avoidance_lieaner
            
            # self.blackboard.next_pose = next_pose
            
            # self.node.get_logger().info(f"좌 우: {direction} : {twist.angular.z}")
            
        self.node.get_logger().warn(f"{direction}, {void_twist.angular.z}, {self.blackboard.robot_state}") 
        self.cmd_vel.publish(void_twist)
        
                
        # self.node.get_logger().info(f"scan {self.scan.ranges}")
        # min_distances = [0, 0, 0, 0]
        
        # num_segments = 4
        # segment_size = len(self.scan.ranges) // num_segments
        # min_distances = [float('inf')] * num_segments
        
        # for i in range(num_segments):
        #     start_index = i * segment_size
        #     end_index = start_index + segment_size
        #     segment = self.scan.ranges[start_index:end_index]
        
        #     # 0.0을 제외한 유효한 거리 값만 필터링
        #     valid_distances = [dist for dist in segment if dist > 0.0]
            
        #     # 유효한 거리가 있으면 min() 계산
        #     if valid_distances:
        #         min_distances[i] = min(valid_distances)
        
        
        
        # for i in range(num_segments):
        #     # 라이다 한 조각의 거리 최소값 저장
        #     min_distances[i] = min(self.scan.ranges[int(32.5 * (2*i-1)) : int(32.5 * (2*i+1))])
        
        # # 거리 최소값이 임계값보다 작으면 회피동작 수행
        # if min(min_distances) < self.wall_threshold:
        #     self.node.get_logger().info(f"너무 붙은 벽의 인덱스 {min_distances.index(min(min_distances))}")
        #     self.avoid(min_distances.index(min(min_distances)))  # 여기 테스트할때 인덱스 제대로 들어오는지 확인 주의
        
        # return Status.SUCCESS