import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import os
from servee_interfaces.msg import ArucoMarkerResult
import socket
import math

class PicamProcessor(Node):
    def __init__(self):
        super().__init__('picam_processor')
        
        # Declare parameters to be loaded from YAML
        self.declare_parameter('data_path', "~/Downloads/ArucoMarker/calibration_data.npz")
        self.declare_parameter('client_ip', '192.168.0.130')
        self.declare_parameter('client_port', 9999)
        self.declare_parameter('marker_length', 0.15)

        # Load parameters
        data_path = self.get_parameter('data_path').get_parameter_value().string_value
        client_ip = self.get_parameter('client_ip').get_parameter_value().string_value
        client_port = self.get_parameter('client_port').get_parameter_value().integer_value
        self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value
        
        print("marker length: ", self.marker_length)

        # Load calibration data
        data_path = os.path.expanduser(data_path)
        data = np.load(data_path)
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs = data['dist_coeffs']

        # Create a publisher to publish marker data
        self.publisher = self.create_publisher(ArucoMarkerResult, 'aruco_marker_data', 10)

        # Initialize Aruco detector
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.detector = cv2.aruco.ArucoDetector(aruco_dict)
        
        # Open webcam
        self.cap = cv2.VideoCapture(0)

        # Initialize UDP socket
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_address = (client_ip, client_port)  # Replace <client_ip> with the client's IP address

        # Start timer to periodically process and publish data
        self.timer = self.create_timer(0.1, self.detect_and_publish)

    def detect_and_publish(self):
        ret, frame = self.cap.read()  # Read frame from webcam
        if not ret:
            self.get_logger().error("Failed to grab frame")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
        corners, ids, _ = self.detector.detectMarkers(gray)  # Detect Aruco markers

        marker_data = []  # Temporary table to store data for each detected marker
        # Define the camera center
        _, frame_width = frame.shape[:2]
        camera_center_x = frame_width / 2

        if ids is not None:  # If markers are detected
            for i, corner in enumerate(corners):
                half_length = self.marker_length / 2  # Half of marker's edge length
                obj_points = np.array([
                    [-half_length, -half_length, 0],
                    [half_length, -half_length, 0],
                    [half_length, half_length, 0],
                    [-half_length, half_length, 0]
                ], dtype=np.float32)

                # Estimate the pose of the marker
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, corner[0], self.camera_matrix, self.dist_coeffs
                )

                if success:

                    # Translation vector (tvec) gives x, y, z distances
                    x = tvec[0][0]
                    y = tvec[1][0]
                    z = tvec[2][0]
                    distance = np.sqrt(x**2 + y**2 + z**2)

                    marker_center_x = np.mean(corner[0][:, 0])  # Average x-coordinates of corners
                    centerline_error = marker_center_x - camera_center_x

                    # Convert rotation vector to rotation matrix
                    R, _ = cv2.Rodrigues(rvec)  

                    # Calculate Euler angles (roll, pitch, yaw) from the rotation matrix
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

                    # Convert angles from radians to degrees
                    roll_deg = math.degrees(roll)
                    pitch_deg = math.degrees(pitch)
                    yaw_deg = math.degrees(yaw)

                    # Append marker data with smoothed yaw
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

            if marker_data:
                closest_marker = min(marker_data, key=lambda x: x["distance"])
                marker_msg = ArucoMarkerResult()
                marker_msg.id = closest_marker["id"]
                marker_msg.distance = closest_marker["distance"]
                marker_msg.x = closest_marker["x"]
                marker_msg.y = closest_marker["y"]
                marker_msg.z = closest_marker["z"]
                marker_msg.centerline_error = closest_marker["centerline_error"]
                marker_msg.yaw = closest_marker["yaw"]
                marker_msg.pitch = closest_marker["pitch"]
                marker_msg.roll = closest_marker["roll"]

                # Publish the message
                self.publisher.publish(marker_msg)
                self.get_logger().info(
                    f"Published Marker ID: {marker_msg.id}, "
                    f"Distance: {marker_msg.distance:.2f}m, "
                    f"Centerline Error: {marker_msg.centerline_error:.2f}, "
                    f"X: {marker_msg.x:.2f}m, Y: {marker_msg.y:.2f}m, Z: {marker_msg.z:.2f}m, "
                    f"Yaw: {marker_msg.yaw:.2f}°, Pitch: {marker_msg.pitch:.2f}°, Roll: {marker_msg.roll:.2f}°"
                )
        
        #Set encoding parameters to reduce image quality (0 to 100, where 100 is highest quality)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]  # Adjust quality; 30 is lower quality

        # Encode the frame as JPEG for transmission
        _, buffer = cv2.imencode('.jpg', frame, encode_param)
        frame_data = buffer.tobytes()
        # Send the encoded frame via UDP
        self.udp_socket.sendto(frame_data, self.client_address)

        # Check for quit condition
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        # Release resources when the node is destroyed
        self.cap.release()
        self.udp_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PicamProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()