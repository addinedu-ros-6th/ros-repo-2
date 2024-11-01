import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import os
from test_package_msgs.msg import ArucoMarker
import socket

class ArucoPublisher(Node):
    def __init__(self):
        super().__init__('aruco_marker_publisher')
        
        # Create a publisher to publish marker data
        self.publisher = self.create_publisher(ArucoMarker, 'aruco_marker_data', 10)
        
        # Initialize Aruco detector
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.detector = cv2.aruco.ArucoDetector(aruco_dict)

        # Marker length (in meters)
        self.marker_length = 0.15  # Example: 15 cm
        
        # Load calibration data
        data_path = os.path.expanduser("~/Downloads/ArucoMarker/calibration_data.npz")
        data = np.load(data_path)
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs = data['dist_coeffs']
        
        # Open webcam
        self.cap = cv2.VideoCapture(0)
        
        # Start timer to periodically process and publish data
        self.timer = self.create_timer(0.1, self.detect_and_publish)

        # Initialize UDP socket
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_address = ('192.168.0.155', 9999)  # Replace <client_ip> with the client's IP address

    def detect_and_publish(self):
        ret, frame = self.cap.read()  # Read frame from webcam
        if not ret:
            self.get_logger().error("Failed to grab frame")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
        corners, ids, _ = self.detector.detectMarkers(gray)  # Detect Aruco markers

        marker_data = []  # Temporary table to store data for each detected marker

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
                    # Convert rotation vector to rotation matrix
                    rotation_matrix, _ = cv2.Rodrigues(rvec)

                    # Calculate Euler angles from the rotation matrix
                    sy = np.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)
                    singular = sy < 1e-6

                    if not singular:
                        yaw = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
                        pitch = np.arctan2(-rotation_matrix[2, 0], sy)
                        roll = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
                    else:
                        yaw = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
                        pitch = np.arctan2(-rotation_matrix[2, 0], sy)
                        roll = 0

                    # Transform tvec to the world coordinate system
                    transformed_tvec = -np.matmul(rotation_matrix.T, tvec)

                    # Get x, y, z coordinates, calculate distance
                    x = transformed_tvec[0][0]
                    y = transformed_tvec[1][0]
                    z = transformed_tvec[2][0]
                    distance = np.sqrt(x**2 + y**2 + z**2)

                    # Append marker data to the list
                    marker_data.append({
                        "id": int(ids[i][0]),
                        "distance": distance,
                        "x": x,
                        "y": y,
                        "z": z,
                        "yaw": np.degrees(yaw),
                        "pitch": np.degrees(pitch),
                        "roll": np.degrees(roll)
                    })

            if marker_data:
                closest_marker = min(marker_data, key=lambda x: x["distance"])
                marker_msg = ArucoMarker()
                marker_msg.id = closest_marker["id"]
                marker_msg.distance = closest_marker["distance"]
                marker_msg.x = closest_marker["x"]
                marker_msg.y = closest_marker["y"]
                marker_msg.z = closest_marker["z"]
                marker_msg.yaw = closest_marker["yaw"]
                marker_msg.pitch = closest_marker["pitch"]
                marker_msg.roll = closest_marker["roll"]

                # Publish the message
                self.publisher.publish(marker_msg)
                self.get_logger().info(
                    f"Published Marker ID: {marker_msg.id}, "
                    f"Distance: {marker_msg.distance:.2f}m, "
                    f"X: {marker_msg.x:.2f}m, Y: {marker_msg.y:.2f}m, Z: {marker_msg.z:.2f}m, "
                    f"Yaw: {marker_msg.yaw:.2f}°, Pitch: {marker_msg.pitch:.2f}°, Roll: {marker_msg.roll:.2f}°"
                )
            
                # Draw the axes on the frame
                # cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)
        
        # Display the frame (optional)
        # cv2.imshow("Aruco Marker Detection", frame)
        
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
        # cv2.destroyAllWindows()
        self.udp_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

