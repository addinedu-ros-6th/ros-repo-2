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

from network.detector_manager_ROS_client_format import ClientFormat
from network.detector_manager_ROS_server_format import ServerFormat
data_queue_robot1 = queue.Queue()
data_queue_robot2 = queue.Queue()

class ROS2SocketNode(Node):
    def __init__(self):
        super().__init__('ros2_detector_manager_node')
        self.robot1_topic = "robot1"
        self.robot2_topic = "robot2"
        self.client_robot1 = ClientFormat("localhost", 9999, data_queue_robot1)
        self.server_robot1 = ServerFormat("localhost", 9998, data_queue_robot1)
        
        self.client_robot2 = ClientFormat("localhost", 9996, data_queue_robot2)
        self.server_robot2 = ServerFormat("localhost", 9997, data_queue_robot2)

        self.subscription_robot1= self.create_subscription(
            String,
            self.robot1_topic,
            self.listener_callback_robot1,
            10 # 큐 크기
        )
        self.subscription_robot2= self.create_subscription(
            String,
            self.robot2_topic,
            self.listener_callback_robot2,
            10 # 큐 크기
        )
        self.subscription_robot1
        self.subscription_robot2  

        # 소켓 서버 스레드 시작
        self.socket_thread = threading.Thread(target=self.server_test)
        self.socket_thread.start()

        time.sleep(1)
        self.socket_thread_1 = threading.Thread(target=self.video_test)
        self.socket_thread_1.start()

        #로봇1번의 쓰레드
        time.sleep(1)
        self.robot1_client_thread = threading.Thread(target=self.client_robot1.client_start)
        self.robot1_client_thread.start()

        time.sleep(1)
        self.robot1_server_thread = threading.Thread(target=self.server_robot1.server_start)
        self.robot1_server_thread.start()

        #로봇2번의 쓰레드
        self.robot2_client_thread = threading.Thread(target=self.client_robot2.client_start)
        self.robot2_client_thread.start()

        time.sleep(1)
        self.robot2_server_thread = threading.Thread(target=self.server_robot2.server_start)
        self.robot2_server_thread.start()


    def listener_callback_robot1(self, msg):
        # 수신한 메시지 출력
        self.get_logger().info(f'Received: "{msg.data}" topic: {self.robot1_topic}')

    def listener_callback_robot2(self, msg):
        # 수신한 메시지 출력
        self.get_logger().info(f'Received: "{msg.data}" topic: {self.robot2_topic}')          

    def server_test(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        host = 'localhost'  # Raspberry Pi의 IP 주소
        port = 9999

        # 소켓 바인딩 및 수신 대기 설정
        server_socket.bind((host, port))
        #server_socket.listen(1)
        print(f"Listening on {host}:{port}...")

        # 클라이언트 연결 수락
        #client_socket, addr = server_socket.accept()
        #print(f"Connection from {addr} established.")

        # 카메라 연결
        cap = cv2.VideoCapture(0)

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            
            # 이미지 크기 조정 (50% 크기로 축소)
            frame_resized = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
            
            # JPG 형식으로 이미지 압축 (품질 설정: 90%)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            result, frame_encoded = cv2.imencode('.jpg', frame_resized, encode_param)
            
            # 압축된 이미지를 직렬화하여 전송
            #data = pickle.dumps(frame_encoded)
            #message = struct.pack("Q", len(data)) + data
            #server_socket.sendto(message,(host, port))
            server_socket.sendto(frame_encoded,(host, port))
            #frame = cv2.imdecode(frame_encoded, cv2.IMREAD_COLOR)
#
            #if frame is not None:
            #    # Display the frame on the client side
            #    cv2.imshow('Client - Receiving Video', frame)
            #    
            #    # Exit on 'q' key press
            #    if cv2.waitKey(1) & 0xFF == ord('q'):
            #        break

        # 연결 종료
        cap.release()
        server_socket.close()

    def video_test(self):
        video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        video_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        host = 'localhost'  # Raspberry Pi의 IP 주소
        port = 9996

        # 소켓 바인딩 및 수신 대기 설정
        video_socket.bind((host, port))
        #video_socket.listen(1)
        print(f"Listening on {host}:{port}...")

        # 클라이언트 연결 수락
        #v_client_socket, addr = video_socket.accept()
        #print(f"Connection from {addr} established.")

        # 카메라 연결
        cap = cv2.VideoCapture(2)

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            
            # 이미지 크기 조정 (50% 크기로 축소)
            frame_resized = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
            
            # JPG 형식으로 이미지 압축 (품질 설정: 90%)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            result, frame_encoded = cv2.imencode('.jpg', frame_resized, encode_param)
            
            # 압축된 이미지를 직렬화하여 전송
            #data = pickle.dumps(frame_encoded)
            #message = struct.pack("Q", len(data)) + data
           
            video_socket.sendto(frame_encoded.tobytes(),(host, port))

        # 연결 종료
        cap.release()
        #v_client_socket.close()
        video_socket.close()    

def main(args=None):
    rclpy.init(args=args)
    node = ROS2SocketNode()

    try:

        rclpy.spin(node)
       
    except KeyboardInterrupt:
        pass
    finally:
        node.socket_thread.join()
        node.socket_thread_1.join()   # 올바른 쓰레드 종료 대기
        node.robot1_client_thread.join()
        node.robot1_server_thread.join()
        node.robot2_client_thread.join()
        node.robot2_server_thread.join()
        node.destroy_node()
        rclpy.shutdown()       


if __name__ == "__main__":
    main()