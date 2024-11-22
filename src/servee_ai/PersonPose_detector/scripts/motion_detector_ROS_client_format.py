import os
import sys
import socket
import cv2
import pickle
import struct
import threading
import time
import queue
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

current_dir = os.path.dirname(os.path.abspath(__file__)) # 현재 스크립트의 디렉토리를 가져오고, 프로젝트 루트로 이동하는 상대 경로를 추가
relative_path = os.path.join(current_dir, '../../..')  # 상위 폴더로 이동
sys.path.append(relative_path)

from scripts.pose_estimate_final_ver import YoloPose


class ClientFormat(Node):
    def __init__(self,host="localhost", port=9999,shared_queue=None,topic="pose_check",nodeName="motion_node"):
        super().__init__(nodeName)
        self.host = host
        self.port = port
        self.client_socket = None
        self.shared_queue = shared_queue
        self.msg = String()
        self.yolo_pose = YoloPose()
        self.topic = topic
        self.publisher = self.create_publisher(String, self.topic, 10)
        

    def client_start(self):
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
        self.client_socket.bind((self.host, self.port))
        # 서버에 연결
        #client_socket.connect((self.host, self.port))
        #data = b""
        #payload_size = struct.calcsize("Q")
        # print("test up")
        while True:
            # print("test down")
            packet, addr = self.client_socket.recvfrom(65536)  # 수신할 최대 크기 설정
            # print("test frame")
            # 수신한 데이터를 OpenCV 이미지로 변환
            frame = cv2.imdecode(np.frombuffer(packet, np.uint8), cv2.IMREAD_COLOR)
            self.result= self.yolo_pose.predict_pose(frame)
            
            if(self.result ==None):
                self.msg.data="noData"
            else:    
                self.msg.data = self.result
            print(self.result)
            # print("ssss")
            self.publisher.publish(self.msg)
            self.get_logger().info(f'Publishing: {self.msg.data} topic: {self.topic}')
            


        



