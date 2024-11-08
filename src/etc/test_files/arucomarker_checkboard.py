# import cv2
# import os

# # 체스보드 크기 (8x6 내부 코너)
# CHECKERBOARD = (8, 6)

# # 이미지 저장 경로 생성
# save_dir = os.path.expanduser("~/Downloads/ArucoMarker/calibration_images")
# os.makedirs(save_dir, exist_ok=True)

# cap = cv2.VideoCapture(0)  # 0번 웹캠 사용

# if not cap.isOpened():
#     print("[ERROR] 웹캠을 열 수 없습니다.")
#     exit()

# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
# print("[INFO] 's' 키를 눌러 이미지를 저장하고, 'q' 키를 눌러 종료하세요.")
# img_count = 0

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         print("[ERROR] 프레임을 읽어오지 못했습니다.")
#         break

#     # 체스보드 코너 검출
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
#                                                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

#     if found:
#         cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, found)

#     # 이미지 표시
#     cv2.imshow("Calibration", frame)
#     key = cv2.waitKey(1) & 0xFF

#     if key == ord('s'):
#         # 이미지 저장
#         img_path = os.path.join(save_dir, f"calib_{img_count}.jpg")
#         cv2.imwrite(img_path, frame)
#         print(f"[INFO] 이미지 저장: {img_path}")
#         img_count += 1
#     elif key == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()