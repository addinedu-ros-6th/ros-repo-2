import cv2
import numpy as np
from collections import deque, Counter
from ultralytics import YOLO
import tensorflow as tf
from sklearn.preprocessing import LabelEncoder


class YoloPose:
    def __init__(self):
    # YOLOv8 Pose 모델 로드
        # self.yolo_model = YOLO('./src/servee_ai/PersonPose_detector/scripts/yolov8n-pose.pt')
        self.yolo_model = YOLO('/home/heechun/dev_ws/ros-repo-2/src/servee_ai/PersonPose_detector/scripts/yolov8n-pose.pt')
        
        
        # 학습된 LSTM 모델 로드
        # self.lstm_model = tf.keras.models.load_model('./src/servee_ai/PersonPose_detector/scripts/pose_check_model_final_ver.keras')
        self.lstm_model = tf.keras.models.load_model('/home/heechun/dev_ws/ros-repo-2/src/servee_ai/PersonPose_detector/scripts/pose_check_model_final_ver.keras')
        
    
        # LabelEncoder 객체 생성 및 클래스에 맞게 fit
        self.label_encoder = LabelEncoder()
        self.label_encoder.fit(["standing", "running", "walking", "sitting"])
        # self.label_encoder.fit(["running", "running", "running", "running"])

    
        # 고정된 시퀀스 길이 설정 (예: 30 프레임)
        self.fixed_time_steps = 15
        self.sequence = deque(maxlen=self.fixed_time_steps)  # 고정된 길이의 deque로 시퀀스 관리
    
        # 예측 결과의 일관성을 위한 예측 결과 저장 deque
        self.prediction_history = deque(maxlen=10)  # 최근 5개 예측을 저장하여 보정
        self.direction_history = deque(maxlen=10)  # 최근 5개 방향을 저장하여 보정
    
        # 관절 표시 여부를 결정하는 변수 (True: 표시, False: 비표시)
        self.show_keypoints = True

    # 웹캠 열기
    #cap = cv2.VideoCapture(0)  # 0은 기본 웹캠, 다른 번호는 추가 웹캠 - 920_cam -> 희천 컴에서 설정한 이름임
    def predict_pose(self, frame):
        # 프레임 크기 가져오기
        height, width, _ = frame.shape

        # YOLOv8 Pose 모델을 이용해 관절 좌표 추출
        results = self.yolo_model(frame)
        keypoints = results[0].keypoints.xy.cpu().numpy() if results and results[0].keypoints else []

        # 좌표를 하나의 벡터로 변환 (예: [x1, y1, x2, y2, ..., x17, y17])
        keypoints_flat = []
        facing_forward = True  # 기본값을 정면(True)으로 설정

        # 검출된 관절이 충분히 있는지 확인 (10개 이상)
        if len(keypoints) > 0 and len(keypoints[0]) >= 10:
            # 관절 좌표를 정규화하여 벡터에 저장
            detected_points = 0
            for kp in keypoints[0]:
                x = kp[0] / width  # x 좌표 정규화
                y = kp[1] / height  # y 좌표 정규화
                keypoints_flat.extend([x, y])

                # 검출된 좌표 중 유효한 좌표 개수 확인
                if x > 0 and y > 0:
                    detected_points += 1

                # [0,0]이 아닌 좌표를 프레임에 표시
                if x > 0 and y > 0 and self.show_keypoints:
                    cv2.circle(frame, (int(kp[0]), int(kp[1])), 3, (0, 0, 255), -1)

            # 만약 유효 좌표(화면에 표시된 좌표)가 10개 미만이면 검출 실패로 간주
            if detected_points < 10:
                cv2.putText(frame, "No Pose detected", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                pose = "go"
                cv2.imshow("Pose Estimation", frame)

                # 'q' 키를 누르면 종료
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    raise KeyboardInterrupt

                return pose

        else:
            # 검출된 좌표가 없거나 10개 미만일 경우
            cv2.putText(frame, "No Pose detected", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            pose = "go"
            cv2.imshow("Pose Estimation", frame)

            # 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                raise KeyboardInterrupt

            return pose

        # 시퀀스에 추가 (고정된 길이 유지)
        if keypoints_flat:
            self.sequence.append(keypoints_flat)

        # 시퀀스 길이가 고정된 길이에 도달하면 예측 수행
        if len(self.sequence) == self.fixed_time_steps:
            input_sequence = np.array(self.sequence).reshape(1, self.fixed_time_steps, 34)  # (1, 15, 34) 형태로 입력

            # 예측
            prediction = self.lstm_model.predict(input_sequence)
            predicted_class = np.argmax(prediction, axis=1)[0]
            predicted_class_name = self.label_encoder.inverse_transform([predicted_class])[0]

            # 예측 결과를 deque에 추가
            self.prediction_history.append(predicted_class_name)

            # 예측 결과의 모드(최빈값)를 사용하여 결과 필터링
            final_class_name = Counter(self.prediction_history).most_common(1)[0][0]

            # 방향 정보도 보정하여 사용
            self.direction_history.append(facing_forward)
            final_direction = Counter(self.direction_history).most_common(1)[0][0]

            # `running` 또는 `walking`일 때만 방향 정보 표시
            if final_class_name in ["running", "walking"]:
                direction_text = "Facing Forward" if final_direction else "Not Facing Forward"
                cv2.putText(frame, f"Predicted: {final_class_name}, {direction_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                pose = "stop" if final_direction else "go"
            else:
                cv2.putText(frame, f"Predicted: {final_class_name}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                pose = "go"
        else:
            pose = "go"

        # 실시간으로 처리 결과를 화면에 표시
        cv2.imshow("Pose Estimation", frame)

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            raise KeyboardInterrupt

        # 결과 pose 반환
        return pose

    # 프레임을 읽어 관절 좌표 추출 및 전처리
    # def predict_pose(self,frame):

    #     # 프레임 크기 가져오기
    #     height, width, _ = frame.shape

    #     # YOLOv8 Pose 모델을 이용해 관절 좌표 추출
    #     results = self.yolo_model(frame)
    #     keypoints = results[0].keypoints.xy.cpu().numpy() if results and results[0].keypoints else []

    #     # 좌표를 하나의 벡터로 변환 (예: [x1, y1, x2, y2, ..., x17, y17])
    #     keypoints_flat = []
    #     facing_forward = True  # 기본값을 정면(True)으로 설정
    #     if len(keypoints) > 0:
    #         for kp in keypoints[0]:
    #             x = kp[0] / width  # x 좌표 정규화
    #             y = kp[1] / height  # y 좌표 정규화
    #             keypoints_flat.extend([x, y])
    #             if x == 0 and y == 0:  # [0,0] 좌표가 있으면 다른 방향으로 간주
    #                 facing_forward = False

    #         # 관절 표시가 켜져 있는 경우 프레임에 표시
    #         if self.show_keypoints:
    #             for kp in keypoints[0]:
    #                 cv2.circle(frame, (int(kp[0]), int(kp[1])), 3, (0, 0, 255), -1)

    #     # 시퀀스에 추가 (고정된 길이 유지)
    #     if keypoints_flat:
    #         self.sequence.append(keypoints_flat)

    #     # 시퀀스 길이가 고정된 길이에 도달하면 예측 수행
    #     if len(self.sequence) == self.fixed_time_steps:
    #         input_sequence = np.array(self.sequence).reshape(1, self.fixed_time_steps, 34)  # (1, 15, 34) 형태로 입력

    #         # 예측
    #         prediction = self.lstm_model.predict(input_sequence)
    #         predicted_class = np.argmax(prediction, axis=1)[0]
    #         predicted_class_name = self.label_encoder.inverse_transform([predicted_class])[0]

    #         # 예측 결과를 deque에 추가
    #         self.prediction_history.append(predicted_class_name)

    #         # 예측 결과의 모드(최빈값)를 사용하여 결과 필터링
    #         final_class_name = Counter(self.prediction_history).most_common(1)[0][0]

    #         # 방향 정보도 보정하여 사용
    #         self.direction_history.append(facing_forward)
    #         final_direction = Counter(self.direction_history).most_common(1)[0][0]

    #         # `running` 또는 `walking`일 때만 방향 정보 표시
    #          # `running` 또는 `walking`일 때만 방향 정보 표시
    #         if final_class_name in ["running", "walking"]:
    #             direction_text = "Facing Forward" if final_direction else "Not Facing Forward"
    #             cv2.putText(frame, f"Predicted: {final_class_name}, {direction_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    #             # cv2.putText(frame, f"Predicted: running, {direction_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    #             pose = "stop" if final_direction else "go"
    #         else:
    #             cv2.putText(frame, f"Predicted: {final_class_name}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    #             # cv2.putText(frame, f"Predicted: running", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
               
    #             pose = "go"
    #     else:
    #         pose = "go"

    #     # 실시간으로 처리 결과를 화면에 표시
    #     cv2.imshow("Pose Estimation", frame)

    #     # 'q' 키를 누르면 종료
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         # 모든 창 닫기
    #         cv2.destroyAllWindows()
    #         raise KeyboardInterrupt  # 루프 종료를 위해 예외 발생

    #     # 결과 pose 반환
    #     return pose
            




