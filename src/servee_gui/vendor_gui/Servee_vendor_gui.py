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

#from etc.db.DBmanager import MySQLConnection
from etc.db.dbtest_connpull import MySQLConnection
from servee_gui.observer_subscriber import ClientObserver
import queue

from PyQt5.QtCore import Qt 
import math
#from qt_material import apply_stylesheet
#import cv2
from PyQt5 import QtWidgets, uic
from functools import partial



###########################################db sql 사용 규칙 ###########################################
# .1 db에 요청을 날리는 경우에는 함수명을 첫번째 인덱스에 쓰고 뒤에 데이터를 넣는다

order_queue = queue.Queue()

class Worker(QThread):
    update_signal = pyqtSignal(str)  # 메인 스레드로 메시지 전달 신호

    def __init__(self):
        super().__init__()
     
        self.running = True
        

    def run(self):
        self.receive_state()
        

    def receive_state(self):

        #데이터 수집
        while self.running:
            try:

                results = order_queue.get()

                #빈공간 제거 해야 함
                if("SE" in results):

                    self.update_signal.emit(results.replace(" ", ""))
                else:
                    pass
            except queue.Empty:
                time.sleep(0.03) 

    def stop(self):
        self.running = False    

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("./src/servee_gui/vendor_gui/servee_vendor.ui",self)

        

        self.setWindowTitle("SERVEE GUI")
        self.cancel_index =0
        self.result_price=0
        self.host = "192.168.0.130"
        self.port = 9993
        self.running = True
        #self.dbm = MySQLConnection()
        #self.dbm.db_connect(self.tcp_ip, 3306, "SERVEE_DB", "kjc", "1234")
        self.ob_client = ClientObserver(self.host, self.port,order_queue)
        self.dbm = MySQLConnection.getInstance()



        self.client_first_receive = threading.Thread(target=self.ob_client.receive_updates)
        self.client_first_receive.start()
        
        self.worker = Worker()  # 서버 IP와 포트 설정
        self.worker.update_signal.connect(self.make_orderlist)  # 신호와 슬롯 연결
        self.worker.start()  # 스레드 시작

        #self.client_order_update = threading.Thread(target=self.receive_state)
        #self.client_order_update.start()

        self.orderlist_tableWidget.setColumnHidden(6, True)  
        self.order_count=1
        self.robot_call_count=0

        current_date = QDate.currentDate()
        formatted_date = current_date.toString("yyyy년 MM월 dd일")

        
        self.day_label.setText(f"<b><center>Today</center>{formatted_date}</b>")


        self.orderlist_tableWidget.verticalHeader().setVisible(False)
        self.orderlist_tableWidget.horizontalHeader().setStretchLastSection(True)  # 마지막 열이 남은 공간을 차지하도록 설정
        
        column_width = 150  # 원하는 너비 설정
        for i in range(self.orderlist_tableWidget.columnCount()):
            self.orderlist_tableWidget.setColumnWidth(i, column_width)
        
        #self.vendor_server = threading.Thread(target=self.receive_orderid)
        #self.vendor_server.start() 

        self.orderlist_tableWidget.setStyleSheet("QTableWidget { gridline-color: transparent; }"
                                         "QTableWidget::item { border: none;}")
        

    def make_orderlist(self,result):
        #command_type = results.split(',')[0]
        #call_type = results.split(',')[1]
        print("vendor 오더테이블 : ", result)
        if("CREATE" in result):

            order_id = result.split(',')[2]
            #call_state = results.split(',')[3]
            row_index=0
            
            print("리스트 만드는 오더아이디 : ", order_id)
            for i in range(3):
                order_results=self.dbm.get_order_details(order_id)
    
    
            #self.robot_call_count= self.robot_call_count+1
            print(order_results)
            row_count = self.orderlist_tableWidget.rowCount()
    
            for row_index, row in enumerate(order_results):
                
                self.orderlist_tableWidget.insertRow(row_count+row_index)
                self.orderlist_tableWidget.setItem(row_count+row_index, 0, QTableWidgetItem(str(self.order_count)))
                self.order_count = self.order_count+1
                for column_index, item in enumerate(row):
                
                    # 각 셀에 데이터 설정
                    self.orderlist_tableWidget.setItem(row_count+row_index, column_index+1, QTableWidgetItem(str(item)))
                    if(column_index==2):
                        item = str(item).split(' ')[1]
                        self.orderlist_tableWidget.setItem(row_count+row_index, column_index+1, QTableWidgetItem(str(item)))
                self.orderlist_tableWidget.setItem(row_count+row_index, 4, QTableWidgetItem("조리중"))
                self.orderlist_tableWidget.setItem(row_count+row_index, 6, QTableWidgetItem(str(order_id)))  
    
            self.orderlist_tableWidget.insertRow(row_count+row_index+1)      
            self.robot_call_button = QtWidgets.QPushButton("로봇호출", self)
            self.robot_call_button.setObjectName("button_serve_" + str(order_id))
            self.robot_call_button.clicked.connect(partial(self.cal_robot_serve, order_id))
            self.orderlist_tableWidget.setCellWidget(row_count+row_index+1, 5, self.robot_call_button)
            self.orderlist_tableWidget.setItem(row_count+row_index+1, 6, QTableWidgetItem(str(order_id)))
            self.orderlist_tableWidget.setColumnHidden(6, True)                
            self.center_text_in_cells()

    def cal_robot_serve(self ,order_id):
        # QTableWidget의 모든 셀 텍스트를 흐리게 설정
        for i in range(self.orderlist_tableWidget.rowCount()):

            if self.orderlist_tableWidget.item(i, 6).text()==order_id:
                for j in range(self.orderlist_tableWidget.columnCount()):
                    item = self.orderlist_tableWidget.item(i, j)
                    if item:
                        # 글꼴 두께를 줄이기
                        font = item.font()
                        font.setWeight(QFont.Light)
                        item.setFont(font)  
                        # 색상을 변경하여 흐리게 보이게 설정 (옵션)
                        item.setForeground(Qt.gray)
            robot_call_button = self.findChild(QPushButton, "button_serve_" + str(order_id))            
            robot_call_button.setEnabled(False)

        self.ob_client.send_update_command("SE",order_id,"서빙중") 


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
