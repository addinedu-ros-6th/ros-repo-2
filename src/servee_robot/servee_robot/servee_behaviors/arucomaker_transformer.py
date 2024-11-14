import math
import os
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access

import cv2

class ArucoMakerTransformer(Behaviour):
    """
    아루커 마커로부터 3D 좌표를 얻습니다. 
    """
    def __init__(self, name:str):
        super(ArucoMakerTransformer, self).__init__(name)
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="picam_raw_image", access=Access.READ)
        self.blackboard.register_key(key="aruco_maker_result", access=Access.WRITE)
        self.blackboard.register_key(key="marker_detected_time", access=Access.WRITE)
        self.blackboard.register_key(key='marker_detected', access=Access.WRITE)
        self.blackboard.register_key(key="robot_state", access=Access.READ)
        self.blackboard.marker_detected  = False
        
    def setup(self, **kwargs: Any) -> None:
        self.node: Node = kwargs['node']
        
        # 현재는 19번 로봇으로만 테스트 가능
        self.node.declare_parameter('data_path', "~/Downloads/ArucoMarker/calibration_data.npz")
        self.node.declare_parameter('marker_length', 0.15)
        
        data_path = self.node.get_parameter('data_path').get_parameter_value().string_value
        self.marker_length = self.node.get_parameter('marker_length').get_parameter_value().double_value
        self.node.get_logger().info(f"marker length: {self.marker_length}")
        
        # 보정 데이터 로드
        data_path = os.path.expanduser(data_path)
        data = np.load(data_path)
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs = data['dist_coeffs']
        
        # Aruco 검출기 초기화
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.detector = cv2.aruco.ArucoDetector(aruco_dict)
    
    def update(self) -> Status:
        
        if self.blackboard.robot_state not in ["aruco", "parking"]:
            # self.node.get_logger().warn(f"{self.blackboard.robot_state}")
            return Status.SUCCESS
        
        
        frame = self.blackboard.picam_raw_image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray) # Aruco 마커 검출
        
        # cv2.imshow("test", frame)
        
        marker_data = []  # 검출된 각 마커의 데이터를 저장할 임시 테이블
        # 카메라 중심 정의
        _, frame_width = frame.shape[:2]
        camera_center_x = frame_width / 2
        
        if ids is not None:  # 마커가 검출된 경우
            for i, corner in enumerate(corners):
                half_length = self.marker_length / 2  # 마커 변의 절반 길이
                obj_points = np.array([
                    [-half_length, -half_length, 0],
                    [half_length, -half_length, 0],
                    [half_length, half_length, 0],
                    [-half_length, half_length, 0]
                ], dtype=np.float32)
                
                # 마커의 자세 추정 (카메라 이미지의 2D 마커 좌표와 마커의 실제 3D 크기를 기반으로 마커의 위치와 방향을 계산)
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, corner[0], self.camera_matrix, self.dist_coeffs
                )
                
                if success:
                    
                    # 변환 벡터(tvec)는 x, y, z 거리 제공
                    x = tvec[0][0]
                    y = tvec[1][0]
                    z = tvec[2][0]
                    distance = np.sqrt(x**2 + y**2 + z**2)

                    marker_center_x = np.mean(corner[0][:, 0])  # 코너의 x좌표 평균 계산
                    centerline_error = marker_center_x - camera_center_x

                    # 회전 벡터를 회전 행렬로 변환
                    R, _ = cv2.Rodrigues(rvec)  

                    # 회전 행렬로부터 오일러 각도(roll, pitch, yaw) 계산
                    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

                    singular = sy < 1e-6

                    if not singular:
                        roll = math.atan2(R[2, 1], R[2, 2])
                        pitch = math.atan2(-R[2, 0], sy)
                        yaw = math.atan2(R[1, 0], R[0, 0])
                    else:
                        roll = math.atan2(-R[1, 2], R[1, 1])
                        pitch = math.atan2(-R[2, 0], sy)
                        yaw = 0

                    # 각도를 라디안에서 도(degree)로 변환
                    roll_deg = math.degrees(roll)
                    pitch_deg = math.degrees(pitch)
                    yaw_deg = math.degrees(yaw)

                    # 마커 데이터에 추가 (yaw는 평활화 처리)
                    marker_data.append({
                        "id": int(ids[i][0]),
                        "distance": distance,
                        "x": x,
                        "y": y,
                        "z": z,
                        "yaw": yaw_deg,
                        "pitch": pitch_deg,
                        "roll": roll_deg,
                        'centerline_error': centerline_error})
                    
        # 아루코마커가 검출됨. 
        if marker_data:
            closest_marker = min(marker_data, key=lambda x: x["distance"])
            self.blackboard.aruco_maker_result = closest_marker
            self.blackboard.marker_detected_time = self.node.get_clock().now()
            
            self.node.get_logger().info(
                f"Published Marker ID: {closest_marker['id']:.2f}, "
                f"Distance: {closest_marker['distance']:.2f}m, "
                f"Centerline Error: {closest_marker['centerline_error']:.2f}, "
                f"X: {closest_marker['x']:.2f}m, Y: {closest_marker['y']:.2f}m, Z: {closest_marker['z']:.2f}m, "
                f"Yaw: {closest_marker['yaw']:.2f}°, Pitch: {closest_marker['pitch']:.2f}°, Roll: {closest_marker['roll']:.2f}°"
            )  
            self.blackboard.marker_detected  = True
            # self.node.get_logger().warn("아루코마커 검출 완료")
            return Status.FAILURE
        
        else:
            self.blackboard.marker_detected  = False
            # self.node.get_logger().warn("아루코마커 검출 실패")
            return Status.FAILURE