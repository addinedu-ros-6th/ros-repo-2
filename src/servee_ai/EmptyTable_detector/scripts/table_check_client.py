import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
from tensorflow.keras.models import load_model
from collections import deque

class TableCheckClient(Node):
    def __init__(self):
        super().__init__('table_check_client')
        self.publisher_ = self.create_publisher(String, 'table_status', 10)
        self.model = load_model('/home/heechun/final_ws/TableCheck/best_model_1031_093.keras')
        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.1, self.detect_table_status)
        self.table_rois = {
            1: (50, 50, 200, 200),
            2: (300, 50, 200, 200),
            3: (50, 300, 200, 200),
            4: (300, 300, 200, 200)
        }
        self.recent_predictions = {table_num: deque(maxlen=10) for table_num in self.table_rois.keys()}

    def predict_table_status(self, roi):
        roi_resized = cv2.resize(roi, (224, 224))
        roi_array = np.expand_dims(roi_resized / 255.0, axis=0)
        prediction = self.model.predict(roi_array)[0][0]
        return prediction

    def detect_table_status(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return

        for table_num, (x, y, w, h) in self.table_rois.items():
            roi = frame[y:y+h, x:x+w]
            prediction = self.predict_table_status(roi)
            self.recent_predictions[table_num].append(prediction)
            avg_prediction = np.mean(self.recent_predictions[table_num])
            status = "Not Empty" if avg_prediction > 0.5 else "Empty"

            msg = String()
            msg.data = f"Table {table_num} Status: {status}"
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    client = TableCheckClient()

    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.cap.release()
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
