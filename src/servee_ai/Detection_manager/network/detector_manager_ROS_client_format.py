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



class ClientFormat:
    def __init__(self,host="localhost", port=9999,shared_queue=None):
        self.host = host
        self.port = port
        self.client_socket = None
        self.shared_queue = shared_queue

    def client_start(self):
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
        #self.client_socket.connect((self.host, self.port))
        self.client_socket.bind((self.host, self.port))
        # 데이터 수신 루프
        data = b""
        payload_size = struct.calcsize("Q")
        
        while True:
            packet, addr = self.client_socket.recvfrom(65536)
            self.shared_queue.put(packet)
            ## 패킷 수신
#
            #while len(data) < payload_size:
            #    packet , addr = self.client_socket.recvfrom(4*1024)  # 수신 버퍼 크기 설정
            #    if not packet:
            #        break
            #    data += packet
#
#
            ## 데이터 크기 추출
            #packed_msg_size = data[:payload_size]
            #data = data[payload_size:]
#
            #msg_size = struct.unpack("Q", packed_msg_size)[0]
#
            ## 실제 프레임 데이터 수신
            #while len(data) < msg_size:
            #    packet, addr= self.client_socket.recvfrom(4*1024)
            #    if not packet:
            #        break
            #    data += packet
            #
            #frame_data = data[:msg_size]
           #
            #data = data[msg_size:]
            #self.shared_queue.put(frame_data)

  

            


        



