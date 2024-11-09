from typing import Any
import rclpy
from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access
from tf2_ros import TransformListener, Buffer
from etc.utils.pose_utils import PoseUtils 

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class GetCurrPose(Behaviour):
    
    def __init__(self, name:str):
        super(GetCurrPose, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="curr_pose", access=Access.WRITE)
        self.blackboard.register_key(key="yaw", access=Access.WRITE)
        self.blackboard.register_key(key="odom_pose", access=Access.WRITE)
        
    def setup(self, **kwargs: Any) -> None:
        self.node: Node = kwargs['node']
        self.subscription = self.node.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',  # AMCL 노드에서 위치 정보를 받는 경우 일반적으로 /amcl_pose 토픽을 사용
            self.pose_callback,
            10
        )
        
        self.odom_sub = self.node.create_subscription(
            Odometry,
            '/base_controller/odom',
            self.odom_callback,
            10
        )
        
    def update(self) -> Status:
        if not self.blackboard.exists('curr_pose'):
            return Status.RUNNING  
        return Status.FAILURE
    
    
    def pose_callback(self, msg):
        self.blackboard.curr_pose = msg.pose.pose
        self.blackboard.yaw = PoseUtils.get_yaw_from_quaternion(msg.pose.pose.orientation)

    def odom_callback(self, msg):
        self.blackboard.odom_pose = msg.pose.pose

    
    

# try:

                
#     내일 시도.
#     경로 계획은 odom 으로 하고 경로 이동은 odom으로 하는 것이 좋다고 함. AMCL 쓰는게 좋을듯. 
#     trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
    
    
#     2안 지수 이동 평균 (변동성이 큰 노이즈에 취약하다.)
#     curr_position = np.array([curr_pose.position.x, curr_pose.position.y])
#     self.curr_position_filtered = self.alpha * curr_position + (1 - self.alpha) * self.curr_position_filtered
#     self.curr_position = self.curr_position_filtered
    
#     MA필터는 급격한 변화에 취약하다. 
    
#     칼만필터 : 이동 공간이 충분해야한다.
    
#     trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
#     self.blackboard.curr_pose = PoseUtils.create_pose(trans.transform.translation.x, trans.transform.translation.y)
#     self.blackboard.yaw = PoseUtils.get_yaw_from_quaternion(trans.transform.rotation)

# except Exception as e:
#     self.node.get_logger().warn(f"변환을 조회할 수 없습니다: {e}")