import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera_topic', 10)
        self.cap = cv2.VideoCapture(0)  # 카메라 연결
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.publish_frame)  # 10fps로 전송

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # 크기 조정 (원하는 크기로 변경 가능)
            frame_resized = cv2.resize(frame, (640, 480))

            # JPG로 압축 (품질 설정: 90)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            result, frame_encoded = cv2.imencode('.jpg', frame_resized, encode_param)

            # 압축된 이미지를 byte array로 변환
            jpg_as_text = np.array(frame_encoded).tobytes()

            # ROS2 이미지 메시지에 압축된 데이터를 담아 전송
            msg = Image()
            msg.height = frame_resized.shape[0]
            msg.width = frame_resized.shape[1]
            msg.encoding = 'jpeg'  # encoding을 jpeg로 설정
            msg.is_bigendian = 0
            msg.step = len(jpg_as_text) // frame_resized.shape[0]
            msg.data = jpg_as_text

            # 메시지 전송
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing compressed video frame...')

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

