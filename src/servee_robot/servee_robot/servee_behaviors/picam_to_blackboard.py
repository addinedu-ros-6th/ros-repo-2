from typing import Any
import rclpy
from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access

import cv2

class PicamToBlackboard(Behaviour):
    """
    Picam의 raw_image를 블랙보드(picam_raw_image)에 저장하고
    AI 서버에게 udp 통신으로 보내는 클랙스
    """
    def __init__(self, name:str):
        super(PicamToBlackboard, self).__init__(name)
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="picam_raw_image", access=Access.WRITE)
        
    def setup(self, **kwargs: Any) -> None:
        self.node: Node = kwargs['node']
        self.cap = cv2.VideoCapture(0)
        if not self.cap or not self.cap.isOpened():
            self.node.get_logger().error("Camera is not opened")


    def update(self) -> Status:
        ret, frame = self.cap.read()
        if not ret:
            self.node.get_logger().error("Failed to grab frame")
            return Status.FAILURE
        
        self.blackboard.picam_raw_image = frame
        return Status.SUCCESS
        
        
    def shutdown(self) -> None:
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.cap = None
            self.node.get_logger().info("Camera released")
        super().shutdown()