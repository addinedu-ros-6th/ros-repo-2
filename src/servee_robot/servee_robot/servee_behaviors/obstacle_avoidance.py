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
    
        
    def setup(self, **kwargs: Any) -> None:
        self.node: Node = kwargs['node']
    
        self.node.declare_parameters(
            namespace='',
            parameters=[
                ('avoidance_lieaner', 0.8),
                ('avoidance_angular', 1.2),
                ('wall_threshold', 0.3),
            ]
        )
        
        self.avoidance_lieaner = self.node.get_parameter('avoidance_lieaner').value
        self.avoidance_angular = self.node.get_parameter('avoidance_angular').value
        self.wall_threshold = self.node.get_parameter('wall_threshold').value
        
        self.node.get_logger().info(f"lieaner: {self.avoidance_lieaner}, angular: {self.avoidance_angular}")
        self.cmd_vel = self.node.create_publisher(
            Twist, 
            '/base_controller/cmd_vel_unstamped', 
            10
        )
     
             
    def update(self) -> Status:
        self.scan = self.blackboard.scan
        ranges = self.scan.ranges
        num_readings = len(ranges)
        
        
         # 세그먼트 수와 각 세그먼트별 최소 거리 리스트
        num_segments = 4
        min_distances = [float('inf')] * num_segments

        start_angle = 162  # 97
        angle_step =  num_readings // num_segments
        
       

        for i in range(num_segments):
            # 각 세그먼트의 시작 각도와 인덱스 계산
            angle = start_angle + i * angle_step
            start_index = int((angle / 360) * num_readings)
            end_index = int(((angle + angle_step) / 360) * num_readings)
            
            # 해당 세그먼트 범위 가져오기
            segment = ranges[start_index:end_index]
            
            # 0.0을 제외한 유효한 거리 값만 필터링
            valid_distances = [dist for dist in segment if dist > 0.0]
            
            # 유효한 거리가 있으면 최소 거리 저장
            if valid_distances:
                min_distances[i] = min(valid_distances)

        # 회피 동작 수행
        if min(min_distances) < self.wall_threshold:
            closest_direction = min_distances.index(min(min_distances))
            self.node.get_logger().info(f"가까운 벽의 방향 인덱스: {closest_direction}")
            self.avoid(closest_direction)  # 인덱스를 전달하여 회피 동작 수행
            self.node.get_logger().info(f"가장 가까운 인덱스: {ranges.index(min(ranges))}")

        return Status.SUCCESS

    
    
    def avoid(self, direction):
        """
        너무 근접한 벽으로부터 도망친다.
        0: 정면
        1: 왼쪽
        2: 뒤쪽
        3: 오른쪽
        """
        self.node.get_logger().info(f"회피 힘 앞뒤: {self.avoidance_lieaner}, 좌우 {self.avoidance_angular}, dir: {direction}") 
        void_twist = Twist()
        if direction % 2 == 0:
            # 앞뒤 회피
            void_twist.linear.x = -self.avoidance_lieaner if direction == 0 else self.avoidance_lieaner
            # self.node.get_logger().info(f"앞 뒤: {direction} : {twist.linear.x}")
        else:
            # 좌우 회피
            void_twist.angular.z = -self.avoidance_angular if direction == 1 else self.avoidance_angular
            # self.node.get_logger().info(f"좌 우: {direction} : {twist.angular.z}")
            
        self.node.get_logger().info(f"좌 우: {direction}, {void_twist.angular.z}") 
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