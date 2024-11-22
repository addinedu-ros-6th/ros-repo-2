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
data_queue_robot3 = queue.Queue()


class ROS2SocketNode(Node):
    def __init__(self):
        super().__init__('ros2_detector_manager_node')
        #토픽 설정
        self.robot1_topic = "robot1" # 서빙로봇1 의 토픽
        self.robot2_topic = "robot2" # 서빙로봇2 의 토픽
        self.robot3_topic = "robot3" # 회수로봇1 의 토픽

        # 로봇 클라이언트 및 서버 인스턴스 추가
        # 각 로봇은 ClientFormat을 통해 비디오 데이터를 받고, ServerFormat을 통해 데이터를 전송
        # 로봇마다 고유한 포트가 할당
        # self.client_robot1 = ClientFormat("192.168.0.130", 9999, data_queue_robot1) #9999 포트로 데이터를 받고
        self.client_robot1 = ClientFormat("192.168.0.49", 9999, data_queue_robot1)
        # self.server_robot1 = ServerFormat("192.168.0.130", 9998, data_queue_robot1) #9998 포트로 데이터를 보냄
        self.server_robot1 = ServerFormat("192.168.0.49", 9998, data_queue_robot1)

        self.client_robot2 = ClientFormat("localhost", 9997, data_queue_robot2)
        self.server_robot2 = ServerFormat("localhost", 9996, data_queue_robot2)

        self.client_robot3 = ClientFormat("localhost", 9995, data_queue_robot3)
        self.server_robot3 = ServerFormat("localhost", 9994, data_queue_robot3)

        #토픽 구독 설정
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

        self.robot3_topic = "robot3"
        self.subscription_robot3 = self.create_subscription(
            String,
            self.robot3_topic,
            self.listener_callback_robot3,
            10
        )


        self.subscription_robot1
        self.subscription_robot2  
        self.subscription_robot3  

        ## 소켓 서버 스레드 시작
        # self.socket_thread_1 = threading.Thread(target=self.robot1_send_video)
        # self.socket_thread_1.start()

        #time.sleep(1)
        #self.socket_thread_2 = threading.Thread(target=self.robot2_send_video)
        #self.socket_thread_2.start()
#
        #time.sleep(1)
        #self.socket_thread_3 = threading.Thread(target=self.robot3_send_video)
        #self.socket_thread_3.start()


        #로봇1번의 쓰레드
        time.sleep(1)
        self.robot1_client_thread = threading.Thread(target=self.client_robot1.client_start)
        self.robot1_client_thread.start()
#
        time.sleep(1)
        self.robot1_server_thread = threading.Thread(target=self.server_robot1.server_start)
        self.robot1_server_thread.start()

        ##로봇2번의 쓰레드
        #self.robot2_client_thread = threading.Thread(target=self.client_robot2.client_start)
        #self.robot2_client_thread.start()
#
        #time.sleep(1)
        #self.robot2_server_thread = threading.Thread(target=self.server_robot2.server_start)
        #self.robot2_server_thread.start()
#
        ##로봇3번의 쓰레드
        #self.robot3_client_thread = threading.Thread(target=self.client_robot3.client_start)
        #self.robot3_client_thread.start()
#
        #time.sleep(1)
        #self.robot3_server_thread = threading.Thread(target=self.server_robot3.server_start)
        #self.robot3_server_thread.start()



    #콜백 함수 추가
    #각 로봇의 토픽을 구독하고, 수신된 메시지는 콜백 함수를 통해 출력
    # --> 이 부분 로봇에다가 구현
    def listener_callback_robot1(self, msg):
        # 수신한 메시지 출력
        self.get_logger().info(f'Received: "{msg.data}" topic: {self.robot1_topic}')

    def listener_callback_robot2(self, msg):
        # 수신한 메시지 출력
        self.get_logger().info(f'Received: "{msg.data}" topic: {self.robot2_topic}')       

    def listener_callback_robot3(self, msg):
        # 수신한 메시지 출력
        self.get_logger().info(f'Received: "{msg.data}" topic: {self.robot3_topic}')
   # --> 이 부분 로봇에다가 구현

    def robot1_send_video(self):
        robot1_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        robot1_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        host = '192.168.0.49'  # Raspberry Pi의 IP 주소
        port = 9999

        # 소켓 바인딩 및 수신 대기 설정
        robot1_socket.bind((host, port))
        print(f"Listening on {host}:{port}...")

        # 카메라 연결 로봇 1
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

            robot1_socket.sendto(frame_encoded,(host, port))

        # 연결 종료
        cap.release()
        robot1_socket.close()

    def robot2_send_video(self):
        robot2_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        robot2_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        host = 'localhost'  # Raspberry Pi의 IP 주소
        port = 9997

        # 소켓 바인딩 및 수신 대기 설정
        robot2_socket.bind((host, port))
        print(f"Listening on {host}:{port}...")

        # 카메라 연결 로봇 2
        cap = cv2.VideoCapture(1)

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            
            # 이미지 크기 조정 (50% 크기로 축소)
            frame_resized = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
            
            # JPG 형식으로 이미지 압축 (품질 설정: 90%)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            result, frame_encoded = cv2.imencode('.jpg', frame_resized, encode_param)
           
            robot2_socket.sendto(frame_encoded.tobytes(),(host, port))

        # 연결 종료
        cap.release()
        #v_client_socket.close()
        robot2_socket.close()    

    def robot3_send_video(self):
        robot3_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        robot3_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        host = 'localhost'  # Raspberry Pi의 IP 주소
        port = 9995  # 로봇3의 비디오 전송 포트

        # 소켓 바인딩 및 수신 대기 설정
        robot3_socket.bind((host, port))
        print(f"Listening on {host}:{port}...")

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
            robot3_socket.sendto(frame_encoded.tobytes(), (host, port))

        # 연결 종료
        cap.release()
        robot3_socket.close()


def main(args=None):
    rclpy.init(args=args)
    node = ROS2SocketNode()

    try:
        rclpy.spin(node)
       
    except KeyboardInterrupt:
        pass
    finally:
        node.socket_thread_1.join()
        node.socket_thread_2.join()   
        node.socket_thread_3.join()# 올바른 쓰레드 종료 대기
        node.robot1_client_thread.join()
        node.robot1_server_thread.join()
        node.robot2_client_thread.join()
        node.robot2_server_thread.join()
        node.robot3_client_thread.join()
        node.robot3_server_thread.join()
        node.destroy_node()
        rclpy.shutdown()       

if __name__ == "__main__":
    main()

# 실행 흐름
# ROS2SocketNode 객체가 생성되면서 토픽 구독, 클라이언트 및 서버 설정, 비디오 전송 스레드가 동시에 시작됩니다.
# 각 로봇의 비디오 전송 스레드는 독립적으로 비디오 데이터를 소켓을 통해 전송하며, ROS2 노드에서 구독하는 메시지를 로깅합니다.
# main 함수는 ROS2 노드가 종료될 때 모든 스레드가 종료되도록 관리합니다.
# 이 코드로 세 대의 로봇이 서로 독립적으로 데이터를 송수신하며 비디오 스트림을 전송할 수 있습니다.