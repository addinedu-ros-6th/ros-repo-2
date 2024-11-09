import os
import sys
import socket
import cv2
import pickle
import struct
import threading
import time
import queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ServerFormat():
    def __init__(self,host="localhost", port=9999, shared_queue=None):
       
        self.host = host
        self.port = port
        self.server_socket = None
        self.shared_queue = shared_queue
        

    def server_start(self):
        
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
        # 소켓 바인딩 및 수신 대기 설정
        self.server_socket.bind((self.host, self.port))
        #self.server_socket.listen(1)
        print(f"Listening on {self.host}:{self.port}...")

        # 클라이언트 연결 수락
        #self.client_socket, addr = self.server_socket.accept()
        #print(f"Connection from {addr} established.")

        #데이터 수집
        while True:
            try:

                frame = self.shared_queue.get()

                # 압축된 이미지를 직렬화하여 전송
                #data = pickle.dumps(frame_encoded)
                #message = struct.pack("Q", len(frame)) + frame
                #print(len(message))
                #self.server_socket.sendto(message, (self.host, self.port))
                self.server_socket.sendto(frame, (self.host, self.port))
            except queue.Empty:
                time.sleep(0.03) 
            except Exception as e:
                # 연결 종료
                self.server_socket.close()


