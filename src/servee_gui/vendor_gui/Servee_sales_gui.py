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



#import mplcursors
from datetime import datetime

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

class PlotCanvas(FigureCanvas):
    def __init__(self, parent=None):
        fig = Figure()
        self.ax = fig.add_subplot(111)
        super().__init__(fig)
        self.setParent(parent)

    def plot(self, x, y):
        self.ax.clear()  # 이전 그래프 지우기
        self.ax.plot(x, y, 'r-')  # 그래프 그리기
        self.ax.set_title('te')
        self.ax.set_xlabel('Xlabel')
        self.ax.set_ylabel('Ylabel')
        self.draw()  # 그래프 새로 그리기
        
        
        
class SalesWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        #self.sales_widget = QWidget(self)
        #self.sales_layout = QVBoxLayout(self.sales_widget)
        #self.sales_groupBox = QGroupBox("Sales Group Box")

        #self.button1 = QPushButton("버튼 1")
        #self.button1.setObjectName("button_sales_1" )
        #self.button1.setGeometry(20, 60, 100, 30)
        #self.button1.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding) 
        #self.button2 = QPushButton("버튼 66")
        #self.button2.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding) 

        self.stacked_widget = QStackedWidget()
        self.stacked_widget.setGeometry(400, 100, 550, 400)
        #self.stacked_widget.setParent(self)
        
        page1 = QWidget()
        page2 = QWidget()
        page1_layout = QVBoxLayout(page1)
        page2_layout = QVBoxLayout(page2)

        canvas = PlotCanvas(page1)
        canvas2 = PlotCanvas(page2)  # PlotCanvas 생성
        page1_layout.addWidget(canvas)
        page2_layout.addWidget(canvas2)


        self.stacked_widget.addWidget(page1)
        self.stacked_widget.addWidget(page2)
        self.stacked_widget.setParent(self)


        x = np.linspace(0, 10, 100)
        y = np.sin(x)  # y = sin(x)
        canvas.plot(x, y)  # 그래프 그리기

        x2 = np.linspace(0, 50, 100)
        y2 = np.sin(x2)  # y = sin(x)
        canvas2.plot(x2, y2)  # 그래프 그리기

        #self.button1.clicked.connect(lambda: self.show_content(0))
        #self.button2.clicked.connect(lambda: self.show_content(1))
        #self.button3.clicked.connect(lambda: self.show_content(2))

    def make_tapwidget(self):
        self.sales_tabwidget = QTabWidget()
        self.sales_tabwidget.setParent(self)
        self.sales_tabwidget.setFixedSize(921, 561)
        self.sales_tabwidget.move(40, 10)

        self.tab1 = QWidget()
        self.tab2 = QWidget()
        self.tab3 = QWidget()  # 3번째 탭

        self.sales_tabwidget.addTab(self.tab1, "Tab 1")
        self.sales_tabwidget.addTab(self.tab2, "Tab 2")
        self.sales_tabwidget.addTab(self.tab3, "Tab 3")
        self.sales_tabwidget.setParent(self)
        group = self.make_groupbox()
        group.setParent(self.tab3)

        return self.sales_tabwidget

    def make_groupbox(self):
        self.sales_groupBox = QGroupBox("Sales Group Box")
        self.sales_groupBox.setParent(self)
        self.sales_groupBox.setFixedSize(921, 561)
        self.sales_groupBox.move(40, 10)
        #self.groupbox_layout = QHBoxLayout()
        #self.sales_groupBox.setLayout(self.groupbox_layout)
        self.make_button(self.sales_groupBox)
        return self.sales_groupBox

    def make_button(self,sales_groupBox):
        
        y=60
        for i in range(4):
            
            self.button_store = QPushButton(f"버튼_{i}")
            self.button_store.setObjectName(f"button_store_{i}")
            self.button_store.setParent(sales_groupBox)
            #button_layout = QVBoxLayout()
           
            self.button_store.setGeometry(20, y, 100, 30)
            y = y+70
            #self.groupbox_layout.addWidget(self.button_store)
            
            #self.button1.setGeometry(20, 60, 100, 30)
            #self.button1.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)


        #self.button_store.clicked.connect(lambda: self.show_content(i))


    def show_content(self, index):
        self.stacked_widget.setCurrentIndex(index)   
    

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