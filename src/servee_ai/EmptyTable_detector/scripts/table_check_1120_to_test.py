# table_check_ver01.py 파일을 실행하여 로컬 웹캠에서 실시간 테이블 상태를 읽고,
# 그 상태를 table_status 토픽으로 게시. 이 노드가 테이블 상태를 감지하고 전송하는 메인 파일

import cv2
import numpy as np
import tensorflow as tf
from collections import deque
import rclpy
from rclpy.node import Node
from servee_interfaces.msg import TableState

# 학습된 모델 로드
# model = tf.keras.models.load_model('./servee_ai/EmptyTable_detector/scripts/table_check_model_final_ver.keras')
model = tf.keras.models.load_model('/home/heechun/dev_ws/ros-repo-2/src/servee_ai/EmptyTable_detector/scripts/1118_model_01.keras')

# 테이블 ROI 설정: (x, y, width, height) 형식으로 4개의 테이블 지정

# table_rois = {
#     1: (125, 65, 120, 100),    # 테이블 1 
#     2: (420, 70, 125, 100),   # 테이블 2
#     3: (70, 270, 160, 135),   # 테이블 3
#     4: (440, 280, 180, 130)   # 테이블 4
# }

table_rois = {
    1: (125, 65, 120, 80),    # 테이블 1 
    2: (420, 75, 130, 70),   # 테이블 2
    3: (70, 270, 160, 125),   # 테이블 3
    4: (450, 280, 170, 110)   # 테이블 4
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
        self.publisher_ = self.create_publisher(TableState, 'table_check', 10)
        self.cap = cv2.VideoCapture(0)  # 로컬 웹캠 연결
        # self.cap = cv2.VideoCapture('/dev/920_cam')  # 로컬 웹캠 연결
        self.timer = self.create_timer(0.1, self.timer_callback)  # 주기적 콜백

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Failed to capture frame from webcam')
            return

        table_statuses = []  # 테이블 상태를 저장할 리스트
        for table_num, (x, y, w, h) in table_rois.items():
            roi = frame[y:y+h, x:x+w]  # 테이블 영역 추출
            prediction = predict_table_status(roi)

            recent_predictions[table_num].append(prediction)
            avg_prediction = np.mean(recent_predictions[table_num])
            is_empty = avg_prediction <= 0.5  # True: Empty, False: Not Empty
            table_statuses.append(is_empty)

            # 테이블 상태 텍스트 및 박스 표시
            status_text = "Empty" if is_empty else "Not Empty"
            color = (0, 255, 0) if is_empty else (0, 0, 255)
            cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
            cv2.putText(frame, f"Table {table_num}: {status_text}", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # ROS2 메시지 전송
        msg = TableState()

        table_statuses = [bool(status) for status in table_statuses]


        msg.status = table_statuses
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.status}')

        # 프레임 표시
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


# 3. 주요 변경 요약
# TableState 메시지 사용
# 기존 String 대신 bool[] 필드를 가지는 TableState 메시지 사용.
# 상태를 리스트로 저장
# 각 테이블 상태를 bool 리스트로 저장 (True: Empty, False: Not Empty).
# 메시지 퍼블리싱
# TableState 메시지 객체를 생성하고 status 필드에 리스트 데이터를 할당 후 퍼블리싱.

# Published: [True, False, True, False]