import socket
from typing import Any
import rclpy
from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access

import cv2

class ImageSender(Behaviour):
    def __init__(self, name:str):
        super(ImageSender, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="picam_raw_image", access=Access.READ)
        
    def setup(self, **kwargs: Any) -> None:
        self.node: Node = kwargs['node']
        
        self.node.declare_parameter('client_ip', '192.168.0.130')
        self.node.declare_parameter('client_port', 9999)
        
        client_ip = self.node.get_parameter('client_ip').get_parameter_value().string_value
        client_port = self.node.get_parameter('client_port').get_parameter_value().integer_value
        
        # UDP 소켓 초기화
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_address = (client_ip, client_port)  # 클라이언트 IP 주소로 설정
        
    def update(self) -> Status:
        frame = self.blackboard.picam_raw_image
        
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30] 
        
        # 전송을 위해 프레임을 JPEG로 인코딩
        _, buffer = cv2.imencode('.jpg', frame, encode_param)
        frame_data = buffer.tobytes()
        
        # UDP를 통해 인코딩된 프레임 전송
        self.udp_socket.sendto(frame_data, self.client_address)
        
    def shutdown(self) -> None:
        self.udp_socket.close()
        super().shutdown()