from typing import Any
import rclpy
from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access
from tf2_ros import TransformListener, Buffer
from etc.utils.pose_utils import PoseUtils 

class GetCurrPose(Behaviour):
    
    def __init__(self, name:str):
        super(GetCurrPose, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="curr_pose", access=Access.WRITE)
        self.blackboard.register_key(key="yaw", access=Access.WRITE)
        
    def setup(self, **kwargs: Any) -> None:
        self.node: Node = kwargs['node']
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        
    def update(self) -> Status:
        
        try:
            if not self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
                return Status.RUNNING
            
            # self.robot1_pose = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            
            self.blackboard.curr_pose = PoseUtils.create_pose(trans.transform.translation.x, trans.transform.translation.y)
            self.blackboard.yaw = PoseUtils.get_yaw_from_quaternion(trans.transform.rotation)

        except Exception as e:
            self.node.get_logger().warn(f"변환을 조회할 수 없습니다: {e}")
        
        
        return Status.FAILURE