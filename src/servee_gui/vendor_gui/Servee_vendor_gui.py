import sys
import os
import socket
import ast
import threading
current_dir = os.path.dirname(os.path.abspath(__file__)) # 현재 스크립트의 디렉토리를 가져오고, 프로젝트 루트로 이동하는 상대 경로를 추가
print("Current Directory:", current_dir)
relative_path = os.path.join(current_dir, '..', '..') # 상위 폴더로 이동
absolute_path = os.path.abspath(relative_path) 
print("Relative Path:", absolute_path)
sys.path.append(relative_path)

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
#from ultralytics import YOLO
import mysql.connector
import numpy as np
from PyQt5 import uic
import time
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
import datetime
#import mplcursors
from datetime import datetime
import pandas as pd
import matplotlib.dates as mdates
from etc.db.DBmanager import MySQLConnection
from PyQt5.QtCore import Qt 
import math
#from qt_material import apply_stylesheet
#import cv2
from PyQt5 import QtWidgets, uic
from functools import partial

###########################################db sql 사용 규칙 ###########################################
# .1 db에 요청을 날리는 경우에는 함수명을 첫번째 인덱스에 쓰고 뒤에 데이터를 넣는다
class Worker(QThread):
    update_signal = pyqtSignal(str)  # 메인 스레드로 메시지 전달 신호

    def __init__(self, host="192.168.0.130", port=9992):
        super().__init__()
        self.host = host
        self.port = port
        self.running = True

    def run(self):
        # TCP 서버 소켓 생성
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((self.host, self.port))
        server_socket.listen(5)
        print(f"Listening on {self.host}:{self.port}")

       
        try:
            while self.running:
                client_socket, addr = server_socket.accept()  # 클라이언트 연결 수락
                print(f"연결됨 {addr}")
                self.handle_client(client_socket)  # 클라이언트 처리
        except Exception as e:
            print(f"Error accepting connection: {e}")
        finally:
            server_socket.close()
        

    def handle_client(self, client_socket):
        try:
            while self.running:
                data = client_socket.recv(1024)
                if not data:
                    break  # 연결이 끊어졌다면 루프 종료
                message = data.decode('utf-8')
                
                self.update_signal.emit(message)
                response = "ok"
                client_socket.send(response.encode('utf-8')) #클라이언트에게 받은 데이터 그대로 송신

        except Exception as e:
            print(f"Error handling client: {e}")
        finally:
            client_socket.close() 


    def stop(self):
        self.running = False    

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("./src/servee_gui/vendor_gui/servee_vendor.ui",self)

        self.setWindowTitle("SERVEE GUI")
        self.cancel_index =0
        self.result_price=0
        self.tcp_ip = "192.168.0.130"
        self.tcp_port = 9992

        self.dbm = MySQLConnection.getInstance()
        self.dbm.db_connect(self.tcp_ip, 3306, "SERVEE_DB", "yhc", "1234")

        self.worker = Worker(self.tcp_ip, self.tcp_port)  # 서버 IP와 포트 설정
        self.worker.update_signal.connect(self.make_orderlist)  # 신호와 슬롯 연결
        self.worker.start()  # 스레드 시작



        self.order_count=1
        self.robot_call_count=0

        self.dbm = MySQLConnection.getInstance()
        self.dbm.db_connect(self.tcp_ip, 3306, "SERVEE_DB", "yhc", "1234")

        self.pushButton.clicked.connect(lambda: self.make_orderlist(order_id=40))
        self.orderlist_tableWidget.verticalHeader().setVisible(False)
        self.orderlist_tableWidget.horizontalHeader().setStretchLastSection(True)  # 마지막 열이 남은 공간을 차지하도록 설정
        
        column_width = 150  # 원하는 너비 설정
        for i in range(self.orderlist_tableWidget.columnCount()):
            self.orderlist_tableWidget.setColumnWidth(i, column_width)
        
        #self.vendor_server = threading.Thread(target=self.receive_orderid)
        #self.vendor_server.start() 

        self.orderlist_tableWidget.setStyleSheet("QTableWidget { gridline-color: transparent; }"
                                         "QTableWidget::item { border: none;}")
        


    def receive_orderid(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind((self.tcp_ip, self.tcp_port))
        server.listen(5)
        print("서버가 시작되었습니다. 연결을 기다립니다...")

        while True:
            try:
                client_socket, addr = server.accept()
                if client_socket:
                    print(f"연결됨: {addr}")
                    request_data = client_socket.recv(1024)
                    
                    order_id = request_data.decode('utf-8')

                    order_results=self.dbm.get_order_details(order_id)
                    
                    self.make_orderlist(order_results,order_id)
                    response = "ok"
                    client_socket.send(response.encode('utf-8'))


            except Exception as e:
                pass
            finally:
                client_socket.close()


    def make_orderlist(self,order_id=41):
        row_index=0
        order_results=self.dbm.get_order_details(order_id)
        self.robot_call_count= self.robot_call_count+1

        row_count = self.orderlist_tableWidget.rowCount()

        for row_index, row in enumerate(order_results):
            
            self.orderlist_tableWidget.insertRow(row_count+row_index)
            self.orderlist_tableWidget.setItem(row_count+row_index, 0, QTableWidgetItem(str(self.order_count)))
            self.order_count = self.order_count+1
            for column_index, item in enumerate(row):
                print(column_index, item)
                # 각 셀에 데이터 설정
                self.orderlist_tableWidget.setItem(row_count+row_index, column_index+1, QTableWidgetItem(str(item)))
                if(column_index==2):
                    item = str(item).split(' ')[1]
                    self.orderlist_tableWidget.setItem(row_count+row_index, column_index+1, QTableWidgetItem(str(item)))
            self.orderlist_tableWidget.setItem(row_count+row_index, 4, QTableWidgetItem("조리중"))
            self.orderlist_tableWidget.setItem(row_count+row_index, 6, QTableWidgetItem(str(order_id)))  

        self.orderlist_tableWidget.insertRow(row_count+row_index+1)      
        self.robot_call_button = QtWidgets.QPushButton("로봇호출", self)
        self.robot_call_button.setObjectName("button_menu_" + str(self.robot_call_count))
        self.orderlist_tableWidget.setCellWidget(row_count+row_index+1, 5, self.robot_call_button)

        self.orderlist_tableWidget.setColumnHidden(6, True)                
        self.center_text_in_cells()


    def center_text_in_cells(self):
        row_count = self.orderlist_tableWidget.rowCount()
        column_count = self.orderlist_tableWidget.columnCount()
        for row in range(row_count):
            for column in range(column_count):
                item = self.orderlist_tableWidget.item(row, column)
                if item is not None:
                    item.setTextAlignment(Qt.AlignCenter)




    def closeEvent(self, event):
            # 윈도우 종료 시 데이터베이스 연결 종료
        self.dbm.disconnection()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()

    # apply_stylesheet(app, theme='dark_amber.xml')

    window.show()

    sys.exit(app.exec_())
#['dark_amber.xml', 
#'dark_blue.xml', 
#'dark_cyan.xml', 
#'dark_lightgreen.xml', 
#'dark_medical.xml', 
#'dark_pink.xml', 
#'dark_purple.xml',
#'dark_red.xml', 
#'dark_teal.xml',
#'dark_yellow.xml',
#'light_amber.xml', 
#'light_blue.xml', 
#'light_blue_500.xml', 
#'light_cyan.xml', 
#'light_cyan_500.xml', 
#'light_lightgreen.xml', 
#'light_lightgreen_500.xml', 
#'light_orange.xml', 
#'light_pink.xml', 
#'light_pink_500.xml',
#'light_purple.xml', 
#'light_purple_500.xml', 
#'light_red.xml', 
#'light_red_500.xml',
#'light_teal.xml', 
#'light_teal_500.xml', 
#'light_yellow.xml']
