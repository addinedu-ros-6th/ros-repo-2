# table_check_ver01.py 파일을 실행하여 로컬 웹캠에서 실시간 테이블 상태를 읽고,
# 그 상태를 table_status 토픽으로 게시. 이 노드가 테이블 상태를 감지하고 전송하는 메인 파일

import cv2
import numpy as np
import tensorflow as tf
from collections import deque
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 학습된 모델 로드
# model = tf.keras.models.load_model('./servee_ai/EmptyTable_detector/scripts/table_check_model_final_ver.keras')
model = tf.keras.models.load_model('/home/heechun/dev_ws/ros-repo-2/src/servee_ai/EmptyTable_detector/scripts/table_check_model_final_ver.keras')

# 테이블 ROI 설정: (x, y, width, height) 형식으로 4개의 테이블 지정
# table_rois = {
#     1: (50, 50, 200, 200),    # 테이블 1
#     2: (300, 50, 200, 200),   # 테이블 2
#     3: (50, 300, 200, 200),   # 테이블 3
#     4: (300, 300, 200, 200)   # 테이블 4
# }

table_rois = {
    1: (110, 70, 150, 90),    # 테이블 1 (x, y, x2 = x1+w, y2 = y1+h)
    2: (410, 70, 140, 90),   # 테이블 2
    3: (40, 270, 190, 140),   # 테이블 3
    4: (430, 280, 190, 130)   # 테이블 4
}

# 최근 예측 결과를 저장할 큐
recent_predictions = {table_num: deque(maxlen=10) for table_num in table_rois.keys()}

# 테이블 상태 예측 함수
def predict_table_status(roi):
    roi_resized = cv2.resize(roi, (224, 224))
    roi_array = np.expand_dims(roi_resized / 255.0, axis=0)  # 정규화
    prediction = model.predict(roi_array)[0][0]
    return prediction

class TableCheckNode(Node):
    def __init__(self):
        super().__init__('table_check_node')
        self.publisher_ = self.create_publisher(String, 'table_status', 10)
        self.cap = cv2.VideoCapture(2)  # 로컬 웹캠 연결
        self.timer = self.create_timer(0.1, self.timer_callback)  # 주기적 콜백

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Failed to capture frame from webcam')
            return

        for table_num, (x, y, w, h) in table_rois.items():
            roi = frame[y:y+h, x:x+w]  # 테이블 영역 추출
            prediction = predict_table_status(roi)

            recent_predictions[table_num].append(prediction)
            avg_prediction = np.mean(recent_predictions[table_num])
            status = "Not Empty" if avg_prediction > 0.5 else "Empty"
            color = (0, 255, 0) if status == "Empty" else (0, 0, 255)

            # 테이블 상태 텍스트 표시
            cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
            cv2.putText(frame, f"Table {table_num}: {status}", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # ROS2 메시지 전송
            msg = String()
            msg.data = f'Table {table_num} Status: {status}'
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')

        cv2.imshow('Table Status', frame)
        if cv2.waitKey(1) & 0xFF == 27:
            self.cap.release()
            cv2.destroyAllWindows()
            self.get_logger().info('Shutting down Table Check Node')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TableCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# ROS2 노드 설정: TableCheckNode 클래스를 추가하여 테이블 상태를 주기적으로 감지하고, 결과를 table_status라는 ROS2 토픽으로 전송
# 타이머 콜백: timer_callback 함수를 통해 주기적으로 각 테이블 상태를 예측하고, 해당 상태를 ROS2 메시지로 게시
# 웹캠 연결 및 프레임 표시: 개별 노트북의 웹캠을 통해 실시간으로 프레임을 가져오며, 상태를 GUI에 표시하고 ESC 키를 통해 종료할 수 있도록 구현

# [INFO] [1731299198.417671473] [table_check_node]: Published: Table 1 Status: Not Empty
# 1/1 ━━━━━━━━━━━━━━━━━━━━ 0s 14ms/step
# [INFO] [1731299198.456117654] [table_check_node]: Published: Table 2 Status: Not Empty
# 1/1 ━━━━━━━━━━━━━━━━━━━━ 0s 13ms/step
# [INFO] [1731299198.494262766] [table_check_node]: Published: Table 3 Status: Not Empty
# 1/1 ━━━━━━━━━━━━━━━━━━━━ 0s 12ms/step
# [INFO] [1731299198.533956621] [table_check_node]: Published: Table 4 Status: Not Empty
