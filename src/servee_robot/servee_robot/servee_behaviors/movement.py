import math
from typing import Any
from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access


from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseArray

from etc.utils.pose_utils import PoseUtils

# from servee_robot.servee_robot.etc.utils import PoseUtils

class Movement(Behaviour):
    def __init__(self, name: str):
        super(Movement, self).__init__(name)
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="robot_state", access=Access.WRITE)
        self.blackboard.register_key(key="robot_state", access=Access.READ)
        self.blackboard.register_key(key="curr_pose", access=Access.READ)
        self.blackboard.register_key(key="path", access=Access.READ)
        
    def setup(self, **kwargs: Any) -> None:
        self.reset_values()
        self.navigator:BasicNavigator = BasicNavigator()
        self.node: Node = kwargs['node']
        
    def reset_values(self):
        self.path = PoseArray()
        self.waypoint_index = 0
        
    def initialise(self) -> None:
        self.robot_state = self.blackboard.robot_state
        if len(self.path.poses) == 0 and self.blackboard.exists('path'):    
            self.path = self.blackboard.path
            self.go_to_pose()
            
    def update(self) -> Status:
        # 이동이 더는 불가능한 상태라면 SUCCESS를 반환해서
        # 다른 노드가 실행될 수 있도록 한다. FAILURE를 반환하면 movement가 계속 실행된다.
        if self.are_you_ready() == False:
            return Status.SUCCESS
        
        
        # 이동이 완료된 상태
        if self.navigator.isTaskComplete():
            
            # Path 완료 여부 체크
            if self.is_complete_path():
                self.blackboard.robot_state = "idle"
                self.reset_values()
                return Status.SUCCESS
            
            
            # 다음 웨이포인트로 이동.
            else:
                self.waypoint_index += 1
                self.go_to_pose()
        
        # 이동 중인 상태 
        else:             
            return Status.FAILURE
    
    def go_to_pose(self):
        pose = self.path.poses[self.waypoint_index]
        yaw = PoseUtils.get_yaw_from_quaternion(pose.orientation)
        timeStemp = self.node.get_clock().now().to_msg()
        goal_pose = PoseUtils.create_pose_stamped(pose.position.x, pose.position.y, yaw, timeStemp)
        self.navigator.goToPose(goal_pose)


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
    
    def is_complete_path(self):
        """
        waypoint_index가 path의 마지막 인덱스인지 확인한다.
        Returns:
            Path(Bool): Path 완료 여부
        """
        if self.waypoint_index == len(self.path.poses): 
            return True
        else:
            return False
        
        
    def calculate_distance(self, last_position, current_position):
        """
        이전 위치와 현재 위치를 계산해서 이동한 거리를 반환한다.
        Args:
            last_position(Pose.position): 이전 위치
            current_posistion(Pose.position): 현재위치
        Returns:
            distance (Float): 이동한 거리
        """
        dx = last_position.x - current_position.x
        dy = last_position.y - current_position.y
        return math.sqrt(dx ** 2 + dy ** 2)