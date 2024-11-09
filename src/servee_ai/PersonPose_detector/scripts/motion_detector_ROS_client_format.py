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

from scripts.pose_estimate_ver01 import YoloPose


class ClientFormat(Node):
    def __init__(self,host="localhost", port=9999,shared_queue=None,topic="chatter",nodeName="motion_node"):
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
        data = b""
        payload_size = struct.calcsize("Q")
        
        while True:
        #    # 패킷 수신
        #    print("수신중")
        #    while len(data) < payload_size:
        #        packet, addr = self.client_socket.recvfrom(4*1024)  # 수신 버퍼 크기 설정
        #        if not packet:
        #            break
        #        data += packet
#
        #    # 데이터 크기 추출
        #    packed_msg_size = data[:payload_size]
        #    data = data[payload_size:]
        #    msg_size = struct.unpack("Q", packed_msg_size)[0]
#
        #    # 실제 프레임 데이터 수신
        #    while len(data) < msg_size:
        #        packet, addr = self.client_socket.recvfrom(4*1024)
        #        if not packet:
        #            break
        #        data += packet
#
        #    frame_data = data[:msg_size]
        #    data = data[msg_size:]
#
        #    # 받은 데이터를 복원하여 JPG 형식에서 OpenCV 이미지로 변환
        #    frame_encoded = pickle.loads(frame_data)
        #    frame = cv2.imdecode(frame_encoded, cv2.IMREAD_COLOR)
        #    
            packet, addr = self.client_socket.recvfrom(65536)  # 수신할 최대 크기 설정

            # 수신한 데이터를 OpenCV 이미지로 변환
            frame = cv2.imdecode(np.frombuffer(packet, np.uint8), cv2.IMREAD_COLOR)
            self.result= self.yolo_pose.predict_pose(frame)
            
            if(self.result ==None):
                self.msg.data="noData"
            else:    
                self.msg.data = self.result
            print(self.result)
            self.publisher.publish(self.msg)
            self.get_logger().info(f'Publishing: {self.msg.data} topic: {self.topic}')
            


        



