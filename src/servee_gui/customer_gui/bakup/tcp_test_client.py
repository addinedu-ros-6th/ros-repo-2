import sys
import socket
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QMessageBox


def main():
    
    
    try:
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.connect(('localhost', 9990))
        # 정수 전송
        client.send(number.encode('utf-8'))
        # 서버로부터 응답 수신
        response = client.recv(4096).decode('utf-8')
        print(response)
        print(response[1])

    except Exception as e:
        pass
    finally:
        client.close()

if __name__ == "__main__":
    main()