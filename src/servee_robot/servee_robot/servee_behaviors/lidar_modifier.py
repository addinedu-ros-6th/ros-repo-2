from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Access
from tf_transformations import euler_from_quaternion

from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan

import math


class LidarModifier(Behaviour):
    """
    lidar의 Scan데이터를 블랙보드의 key= scan에 저장한다. 
    """
    def __init__(self, name:str):
        super(LidarModifier, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="scan", access=Access.READ)
        self.blackboard.register_key(key="scan", access=Access.WRITE)
        self.blackboard.register_key(key="curr_pose", access=Access.READ)
        
    def setup(self, **kwargs):
        self.node: Node = kwargs['node']
        
        self.robot2_pose = Pose()
        self.robot2_pose_subscriber = self.node.create_subscription(Pose, '/robot2/pose', self.pose_callback_robot2, 10)

        self.robot3_pose = Pose()
        self.robot3_pose_subscriber = self.node.create_subscription(Pose, '/robot3/pose', self.pose_callback_robot3, 10)
        
    def update(self):
        original_scan: LaserScan = self.blackboard.scan
        robot1_pose: Pose = self.blackboard.curr_pose
        
        # 다른 로봇의 위치 정보를 original_scan에 추가해서 
        # 아래와 같이 수정해서 넣기

        try:
            num_readings = 260

            # 로봇1 방향
            robot1_yaw = euler_from_quaternion([0, 
                                                0, 
                                                robot1_pose.orientation.z, 
                                                robot1_pose.orientation.w])[2]
            scan_index_from_yaw = int((robot1_yaw - original_scan.angle_min) / original_scan.angle_increment)  # 회전 보상을 위한 라이다 인덱스 값

            # 로봇1 기준 로봇2의 거리와 각도
            dx_1 = self.robot2_pose.position.x - robot1_pose.position.x
            dy_1 = self.robot2_pose.position.y - robot1_pose.position.y

            distance_1 = math.hypot(dx_1, dy_1)
            angle_1 = math.atan2(dy_1, dx_1)

            # 로봇2 위치에 대한 라이다 값 인덱스
            index_1 = (int((angle_1 - original_scan.angle_min) / original_scan.angle_increment) - scan_index_from_yaw - 130) % num_readings

            # 장애물과의 거리보다 로봇과의 거리가 더 짧을 경우 라이다 값을 조작
            if distance_1 < original_scan.ranges[index_1]:
                original_scan.ranges[index_1] = distance_1
                original_scan.ranges[(index_1+1)%num_readings] = distance_1 / math.cos(original_scan.angle_increment)
                original_scan.ranges[(index_1+2)%num_readings] = distance_1 / math.cos(original_scan.angle_increment*2)
                original_scan.ranges[index_1-1] = distance_1 / math.cos(original_scan.angle_increment)
                original_scan.ranges[index_1-2] = distance_1 / math.cos(original_scan.angle_increment*2)

                # print("center index:", index_1)
                # print("ranges[index_1]:", original_scan.ranges[index_1])
                # print("ranges[index_1+1]:", original_scan.ranges[(index_1+1)%num_readings])
                # print("ranges[index_1+2]:", original_scan.ranges[(index_1+2)%num_readings])
                # print("ranges[index_1-1]:", original_scan.ranges[index_1-1])
                # print("ranges[index_1-2]:", original_scan.ranges[index_1-2])
        
        except Exception as e:
            print(e)

        self.blackboard.scan = original_scan
        
    def pose_callback_robot2(self, msg: Pose):
        self.robot2_pose = msg

    def pose_callback_robot3(self, msg: Pose):
        self.robot3_pose = msg
