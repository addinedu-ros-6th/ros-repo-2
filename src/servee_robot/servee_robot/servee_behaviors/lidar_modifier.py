from typing import Any
import rclpy
from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access
from tf2_ros import TransformListener, Buffer
from etc.utils.pose_utils import PoseUtils 

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy




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
        self.node = kwargs['node']
        
        
        
    def update(self):
        original_scan = self.blackboard.scan
        
        # 다른 로봇의 위치 정보를 original_scan에 추가해서 
        # 아래와 같이 수정해서 넣기
        
        self.blackboard.scan = original_scan
        