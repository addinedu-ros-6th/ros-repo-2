from typing import Any
import rclpy
from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

import time

class RobotDataSender(Behaviour):
    def __init__(self, name:str):
        super(RobotDataSender, self).__init__(name)
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="robot_state", access=Access.READ)
        self.blackboard.register_key(key="curr_pose", access=Access.READ)
        self.blackboard.register_key(key="curr_pose", access=Access.WRITE)
        
    def setup(self, **kwargs):
        self.node : Node = kwargs['node']
        
        self.node.declare_parameter('home_position_x', 0.025)
        self.node.declare_parameter('home_position_y', 0.200)
        
        self.node.declare_parameter('home_orientation_x', 0.0)
        self.node.declare_parameter('home_orientation_y', 0.0)
        self.node.declare_parameter('home_orientation_z', -0.00416502)
        self.node.declare_parameter('home_orientation_w', 1.000)

        self.init_p_x = self.node.get_parameter('home_position_x').get_parameter_value().double_value
        self.init_p_y = self.node.get_parameter('home_position_y').get_parameter_value().double_value
        
        self.init_o_x = self.node.get_parameter('home_orientation_x').get_parameter_value().double_value
        self.init_o_y = self.node.get_parameter('home_orientation_y').get_parameter_value().double_value
        self.init_o_z = self.node.get_parameter('home_orientation_z').get_parameter_value().double_value
        self.init_o_w = self.node.get_parameter('home_orientation_w').get_parameter_value().double_value
        self.blackboard.curr_pose = None
        
        self.publisher_init = self.node.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10)
        
        self.publisher_pose = self.node.create_publisher(
            Pose,
            '/servee/get_pose',
            10
        )
        
        self.publisher_robot_state = self.node.create_publisher(
            String,
            '/servee/get_state',
            10
        )
        
    def update(self):
        if self.blackboard.curr_pose is None: 
            self.init_pose()
            return Status.SUCCESS
        
        curr_pose = Pose()
        curr_pose.position = self.blackboard.curr_pose.position
        
        self.publisher_pose.publish(curr_pose)
        self.send_robot_state()
        
        return Status.SUCCESS
    
    
    def init_pose(self):
        self.node.get_logger().warn("init_pose")
        time.sleep(1)
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.header.frame_id = "map"

        # 위치 설정
        pose.pose.pose.position.x = self.init_p_x
        pose.pose.pose.position.y = self.init_p_y
        pose.pose.pose.position.z = 0.0
        
        pose.pose.pose.orientation.x = self.init_o_x
        pose.pose.pose.orientation.y = self.init_o_y
        pose.pose.pose.orientation.z = self.init_o_z
        pose.pose.pose.orientation.w = self.init_o_w
        
        # self.blackboard.curr_pose = pose.pose.pose
        self.publisher_init.publish(pose)
    
    
    
    def send_robot_state(self):
        
        # 로봇의 상태
        # idle - 임무를 받을 수 있는 상태
        # receive_goal - 목적지를 받은 상태 
        # request_path - 경로를 요청 중인 상태.
        # low_battery - 배터리가 부족한 상태
        # task - 임무를 수행 중인 상태
        # home - 임무를 마치고 돌아가는 상태
        # aruco - 아루커 마커 검색
        robot_state = String()
        
        if self.blackboard.robot_state == "idle":
            robot_state.data = "idle"
            
        elif self.blackboard.robot_state in ["receive_goal", "request_path", "task", "home"]:
            if self.blackboard.aruco_id_index == 0:
                robot_state.data = "running1"
            
            elif self.blackboard.aruco_id_index == 1:
                robot_state.data = "running2"
                
            elif self.blackboard.aruco_id_index == 2:
                robot_state.data = "returning_home"    
                
        
        # "idle"
        # "running1"
        # "standby1"
        # "running2"
        # "standy2"
        # "returning_home"
        # "low_battery"
        

        self.blackboard.register_key(key="aruco_id_index", access=Access.READ)
        
        
        self.publisher_robot_state.publish(robot_state)
        