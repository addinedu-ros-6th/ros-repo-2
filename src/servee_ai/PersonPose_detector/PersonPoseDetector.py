import base64
import cv2
import numpy as np
import sys
import time
import math
import os

from sympy import false
#from ultralytics import YOLO
import socket
import struct
import pickle
import threading


import rclpy
from rclpy.node import Node
from std_msgs.msg import String

current_dir = os.path.dirname(os.path.abspath(__file__)) # 현재 스크립트의 디렉토리를 가져오고, 프로젝트 루트로 이동하는 상대 경로를 추가
relative_path = os.path.join(current_dir, '../../..')  # 상위 폴더로 이동
sys.path.append(relative_path)

from scripts.motion_detector_ROS_client_format import ClientFormat


class ROS2SocketNode(Node):
    def __init__(self):
        super().__init__('ros2_pose_detector_node')
       #변수 선언
        
        self.result=""
        self.robot_1_motion_client = ClientFormat(host="localhost",port=9998,topic="robot1",nodeName="robot1_pub_node")
       
        self.robot_2_motion_client = ClientFormat(host="localhost",port=9996,topic="robot2",nodeName="robot2_pub_node")

        self.robot_3_motion_client = ClientFormat(host="localhost",port=9994,topic="robot3",nodeName="robot3_pub_node")


        # 소켓 서버 스레드 시작
        robot1_thread = threading.Thread(target=self.robot_1_motion_client.client_start)
        robot1_thread.start()
        
        robot2_thread = threading.Thread(target=self.robot_2_motion_client.client_start)
        robot2_thread.start()

        robot3_thread = threading.Thread(target=self.robot_3_motion_client.client_start)
        robot3_thread.start()


def main(args=None):
    rclpy.init(args=args)
    node = ROS2SocketNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("ros error")
    finally:
        node.destroy_node()
        rclpy.shutdown()    
if __name__ == "__main__":
    main()

