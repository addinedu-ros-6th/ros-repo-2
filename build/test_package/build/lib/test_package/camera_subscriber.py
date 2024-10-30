import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera_topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # 수신한 데이터를 numpy 배열로 변환
        jpg_as_np = np.frombuffer(msg.data, dtype=np.uint8)

        # JPG 압축을 해제하여 OpenCV 이미지로 변환
        frame = cv2.imdecode(jpg_as_np, cv2.IMREAD_COLOR)

        # 영상 출력
        if frame is not None:
            cv2.imshow("Received Video", frame)
        if cv2.waitKey(1) == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()