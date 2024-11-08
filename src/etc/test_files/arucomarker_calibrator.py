# import cv2
# import numpy as np
# import glob
# import os

# # 체스보드 크기 (8x6 내부 코너)
# CHECKERBOARD = (8, 6)

# # 체스보드의 실제 3D 좌표 생성
# objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
# objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# # 3D와 2D 포인트를 저장할 리스트
# objpoints = []  # 3D 포인트 (체스보드 코너의 실제 좌표)
# imgpoints = []  # 2D 포인트 (이미지에서 검출된 코너 좌표)

# # 캡처한 체스보드 이미지 경로 불러오기
# images = glob.glob(os.path.expanduser("~/Downloads/ArucoMarker/calibration_images/*.jpg"))


# # 이미지 처리 및 코너 검출
# for fname in images:
#     img = cv2.imread(fname)
#     if img is None:
#         print(f"[ERROR] '{fname}' 이미지를 읽을 수 없습니다.")
#         continue

#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#     # 체스보드 코너 검출
#     found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

#     if found:
#         print(f"[INFO] 코너 감지 성공: {fname}")
#         objpoints.append(objp)
#         imgpoints.append(corners)

#         # 코너 그리기
#         cv2.drawChessboardCorners(img, CHECKERBOARD, corners, found)
#         cv2.imshow("Corners", img)
#         cv2.waitKey(500)
#     else:
#         print(f"[WARNING] 코너 감지 실패: {fname}")

# cv2.destroyAllWindows()

# # 코너가 검출된 이미지가 있는지 확인
# if len(objpoints) == 0 or len(imgpoints) == 0:
#     print("[ERROR] 코너가 검출된 이미지가 없습니다. 캘리브레이션을 진행할 수 없습니다.")
#     exit()

# # 카메라 매트릭스와 왜곡 계수 계산
# ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# # 결과 출력
# print("[INFO] 카메라 매트릭스:\n", camera_matrix)
# print("[INFO] 왜곡 계수:\n", dist_coeffs)

# # 매트릭스와 왜곡 계수를 파일로 저장
# np.savez("/home/minibot/Downloads/ArucoMarker/calibration_data.npz", camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)