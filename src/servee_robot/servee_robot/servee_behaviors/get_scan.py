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




class GetScan(Behaviour):
    """
    lidar의 Scan데이터를 블랙보드의 key= scan에 저장한다. 
    """
    def __init__(self, name:str):
        super(GetScan, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="scan", access=Access.WRITE)
        self.blackboard.scan = None
        
    def setup(self, **kwargs: Any) -> None:
        self.node: Node = kwargs['node']
        
        
        qos_profile_sensor_data = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # RELIABILITY를 BEST_EFFORT로 설정
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.node.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
      
    def update(self) -> Status:
        
        if self.blackboard.exists('scan'):
            return Status.FAILURE
        else:
            return Status.SUCCESS
        
    def scan_callback(self, msg):
        self.blackboard.scan = msg
        