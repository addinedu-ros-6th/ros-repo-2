##=================== < simple camera tester > ===================#
# import cv2
# import numpy as np
# # 웹캠 열기
# cap = cv2.VideoCapture(0)

# if not cap.isOpened():
#     print("[ERROR] 웹캠을 열 수 없습니다.")
#     exit()

# print("[INFO] 웹캠을 통해 마커 인식을 시작합니다. 'q'를 눌러 종료하세요.")

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         print("[ERROR] 프레임을 읽어오지 못했습니다.")
#         break

#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     # 프레임 표시
#     cv2.imshow("Aruco Marker Detection", frame)

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()
##=================== < simple camera tester > ===================#

##=================== < UDP communication tester > ===================#
# import cv2
# import socket

# # 카메라 연결
# cap = cv2.VideoCapture(0)

# # UDP 소켓 설정
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# udp_ip = "192.168.0.155"  # 수신할 서버의 IP 주소로 변경
# udp_port = 9999           # 수신할 서버의 포트 번호로 변경

# # 데이터 송신 루프
# while cap.isOpened():
#     ret, frame = cap.read()
#     if not ret:
#         break

#     # 이미지 크기 조정 (50% 크기로 축소)
#     frame_resized = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)

#     # JPG 형식으로 이미지 압축 (품질 설정: 90%)
#     encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]

#     # 이미지 인코딩 및 바이트 변환
#     result, frame_encoded = cv2.imencode('.jpg', frame_resized, encode_param)
#     frame_data = frame_encoded.tobytes()

#     # UDP로 데이터 송신
#     sock.sendto(frame_data, (udp_ip, udp_port))

# # 리소스 해제
# cap.release()
# sock.close()
##=================== < UDP communication tester > ===================#
