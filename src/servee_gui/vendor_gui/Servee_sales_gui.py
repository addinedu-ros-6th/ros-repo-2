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
order_queue = queue.Queue()

class Store:
    def __init__(self, store_id):
        self.store_id = store_id
        self.order_count = 1
        self.order_data = {}  # 주문 ID를 키로 하고 상태를 값으로 저장하는 딕셔너리
        self.button_status = {}

        self.table_widget = QTableWidget()
        self.table_widget.setObjectName(f"table_widget_{self.store_id}")
        self.table_widget.setColumnCount(7)  # 필요한 컬럼 수 설정
        self.table_widget.setHorizontalHeaderLabels(["주문번호", "음식명", "수량", "주문시간", "상태", "로봇호출", "order_id"])
        column_width = 150

        for i in range(self.table_widget.columnCount()):
            self.table_widget.setColumnWidth(i, column_width)
#
        self.table_widget.setStyleSheet("QTableWidget { gridline-color: transparent; }"
                                                 "QTableWidget::item { border: none; }")
        
        self.table_widget.verticalHeader().setVisible(False)
        self.table_widget.horizontalHeader().setStretchLastSection(True)
        self.table_widget.setColumnHidden(6, True)
        

    def add_order(self, order_id, order_store_data):
        # 주문을 추가하고 상태를 초기화
        if order_id in self.order_data:
            self.order_data[order_id].append(order_store_data)  # 리스트에 추가
        else:
            self.order_data[order_id] = [order_store_data] 
        
        # 여기서 주문 데이터를 처리하는 로직을 추가할 수 있습니다.

    def update_button_status(self, order_id, status):
        # 주문 상태 업데이트
        if order_id not in self.button_status:
            self.button_status[order_id] = status

    def get_button_status(self, order_id):
        #return self.order_status.get(order_id, None)
        return self.button_status.get(order_id, None)
        
    def get_table_widget(self):
        return self.table_widget    
        
        
        
class SalesWindow():
    def __init__(self):
    
        
        self.tab_widget = QTabWidget()
        self.sales_layout = QVBoxLayout(self.tab_widget)
        self.tab_widget.addTab(QLabel("Content of Tab 1"), "Tab 1")
        self.tab_widget.addTab(QLabel("Content of Tab 2"), "Tab 2")
        self.tab_widget.addTab(QLabel("Content of Tab 3"), "Tab 3")
        #self.tab_widget.hide()
        #self.setCentralWidget(self.tab_widget)
       
        
        
    def test(self):
        self.sales_layout.(self.tab_widget)
        #self.tab_widget.show()
            

    def on_combobox_changed(self):
        # 스토어 이름 변경 시 주문 목록 업데이트
        self.update_order_list()

   
    def closeEvent(self, event):
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SalesWindow()
    window.show()
    sys.exit(app.exec_())