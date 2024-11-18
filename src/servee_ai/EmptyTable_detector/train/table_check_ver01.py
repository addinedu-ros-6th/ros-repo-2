import cv2
import numpy as np
import tensorflow as tf

# 학습된 모델 로드
model = tf.keras.models.load_model('/home/heechun/final_ws/TableCheck/table_check_model_ver01.keras')

# 테이블 ROI 설정: (x, y, width, height) 형식으로 4개의 테이블 지정
table_rois = {
    1: (50, 50, 200, 200),    # 테이블 1
    2: (300, 50, 200, 200),   # 테이블 2
    3: (50, 300, 200, 200),   # 테이블 3
    4: (300, 300, 200, 200)   # 테이블 4
}

# 테이블 상태 예측 함수
def predict_table_status(roi):
    """ROI 이미지를 입력으로 받아 물체 유무를 예측하는 함수."""
    roi_resized = cv2.resize(roi, (224, 224))
    roi_array = np.expand_dims(roi_resized / 255.0, axis=0)  # 정규화
    prediction = model.predict(roi_array)[0][0]
    return "Not Empty" if prediction > 0.5 else "Empty"

# 카메라에서 실시간 영상 가져오기
cap = cv2.VideoCapture('/dev/920_cam')  # 0번 카메라 (Raspberry Pi 카메라 또는 USB 카메라)
# cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 각 테이블 ROI에서 상태 예측 및 표시
    for table_num, (x, y, w, h) in table_rois.items():
        roi = frame[y:y+h, x:x+w]  # 테이블 영역 추출
        status = predict_table_status(roi)  # 물체 유무 예측

        # 테이블 상태에 따라 색상 설정 (초록색=Empty, 빨간색=Not Empty)
        color = (0, 255, 0) if status == "Empty" else (0, 0, 255)

        # 테이블 ROI와 상태 텍스트를 화면에 표시
        cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
        cv2.putText(frame, f"Table {table_num}: {status}", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    # 결과 화면 표시
    cv2.imshow('Table Status', frame)

    # ESC 키로 종료
    if cv2.waitKey(1) & 0xFF == 27:
        break

# 리소스 해제
cap.release()
cv2.destroyAllWindows()
