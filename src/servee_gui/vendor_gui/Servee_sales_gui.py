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
import matplotlib.ticker as ticker


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

import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

font_path = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"  # 예시: NanumGothic 폰트 경로
font_prop = fm.FontProperties(fname=font_path, size=12)

plt.rc('font', family=font_prop.get_name())
plt.rc('axes', unicode_minus=False)
order_queue = queue.Queue()

class PlotCanvas(FigureCanvas):
    def __init__(self, parent=None):


        self.fig = Figure(figsize=(6.5, 2.3))

        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)
        self.setParent(parent)
        
    def plot(self, x, y):
        self.ax.clear()
        self.ax.bar(x ,y, color='skyblue')  # 이전 그래프 지우기
        #self.ax.plot(x, y)  # 그래프 그리기
        self.ax.yaxis.set_major_formatter(ticker.FuncFormatter(lambda val, pos: f'{int(val):,}'))
        #self.ax.set_title('te')
        self.ax.set_xlabel('기 간',fontproperties=font_prop)
        self.ax.set_ylabel('매출액',fontproperties=font_prop)
        self.fig.tight_layout()
        
        self.draw()  # 그래프 새로 그리기
        
        
class SalesWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.sales_groupBox = QGroupBox()
        #self.sales_widget = QWidget(self)

        self.dbm = MySQLConnection.getInstance()
        #self.graph_widget = QWidget()

        self.button_text ="마파궁전"
        

    def make_groupbox(self, x,y,w,h):
        
        
        self.sales_groupBox.setParent(self)
        #elf.sales_groupBox.setGeometry(40,10,921,561)
        self.sales_groupBox.setGeometry(x,y,w,h)
        self.sales_groupBox.setStyleSheet("background-color: lightblue;")
       
        #self.make_button(self.sales_groupBox,)
        return self.sales_groupBox

    def make_label_month(self,x,y,w,h):
        self.store_label_month = QLabel()
        self.store_label_month.setStyleSheet("background-color: transparent; color: black;")
        self.store_label_month.setGeometry(x,y,w,h)
        self.store_label_month.setParent(self.sales_groupBox)
        self.store_label_month.setText(f"{self.button_text}의 월 매출현황")
        self.store_label_month.setAlignment(Qt.AlignHCenter)

    def make_label_day(self,x,y,w,h):
        self.store_label_day = QLabel()
        self.store_label_day.setStyleSheet("background-color: transparent; color: black;")
        self.store_label_day.setGeometry(x,y,w,h)
        self.store_label_day.setParent(self.sales_groupBox)
        self.store_label_day.setText(f"{self.button_text}의 일 매출현황")    
        self.store_label_day.setAlignment(Qt.AlignHCenter)
    
    def make_graph_month(self, sales_groupbox,gx,gy,gw,gh):
       
        date_year=self.sales_by_month_year.currentText()

        year = date_year.strip().replace('년', '')
        df=self.dbm.get_sales_by_month(self.button_text,year)
   
        #graph_layout = QVBoxLayout()  # 그래프 위젯의 레이아웃 생성
        #self.graph_widget.setLayout(graph_layout)
        self.graph_widget_month = QWidget()
        self.sales_graph = PlotCanvas()
        
        total_amount = df["total_amount"]
        month = df["order_month"]

        self.sales_graph.plot(month,total_amount)
        #graph_layout.addWidget(self.sales_graph, stretch=1) 
        #self.graph_widget.setGeometry(x,y,w,h)
        self.sales_graph.setParent(self.graph_widget_month)
        self.graph_widget_month.setParent(sales_groupbox)
        self.graph_widget_month.setGeometry(gx,gy,gw,gh)
        self.graph_widget_month.show()

    def make_graph_day(self, sales_groupbox,gx,gy,gw,gh):
       
        date_year=self.sales_by_month_year.currentText()
        year = date_year.strip().replace('년', '')

        date_month=self.sales_by_day_month.currentText()
        month = date_month.strip().replace('월', '')

        df=self.dbm.get_sales_by_day(self.button_text,year,month)
   
        #graph_layout = QVBoxLayout()  # 그래프 위젯의 레이아웃 생성
        #self.graph_widget.setLayout(graph_layout)
        self.graph_widget_day = QWidget()
        self.sales_graph = PlotCanvas()
        
        total_amount = df["total_amount"]
        month = df["order_date"]

        self.sales_graph.plot(month,total_amount)
        #graph_layout.addWidget(self.sales_graph, stretch=1) 
        #self.graph_widget.setGeometry(x,y,w,h)
        self.sales_graph.setParent(self.graph_widget_day)
        self.graph_widget_day.setParent(sales_groupbox)
        self.graph_widget_day.setGeometry(gx,gy,gw,gh)
        self.graph_widget_day.show()    

    def make_button_search_by_month(self,sales_groupBox, x,y,w,h,gx,gy,gw,gh):

        self.search_by_month_year = QPushButton("검색")
        self.search_by_month_year.setObjectName(f"search_by_month_year")
        self.search_by_month_year.setGeometry(x, y, w, h)

        self.search_by_month_year.setParent(sales_groupBox)
        
        self.search_by_month_year.clicked.connect(partial(self.make_graph_month,self.sales_groupBox,gx,gy,gw,gh))
    

    def make_button_search_by_day(self,sales_groupBox, x,y,w,h,gx,gy,gw,gh):
        
        self.search_by_month_year = QPushButton("검색")
        self.search_by_month_year.setObjectName(f"search_by_day")
        self.search_by_month_year.setGeometry(x, y, w, h)

        self.search_by_month_year.setParent(sales_groupBox)
        
        self.search_by_month_year.clicked.connect(partial(self.make_graph_day,self.sales_groupBox,gx,gy,gw,gh))    

    def make_combobox_by_month(self,sales_groupBox, x,y,w,h):
        self.sales_by_month_year = QComboBox()
        self.sales_by_month_year.setObjectName(f"combo_by_month_year")
        self.sales_by_month_year.setGeometry(x, y, w, h)
        for i in range(2024,2015,-1):
            self.sales_by_month_year.addItem(f"{i}년")

        self.sales_by_month_year.setParent(sales_groupBox)

    def make_combobox_year_by_day(self,sales_groupBox, x,y,w,h):
        self.sales_by_day_year = QComboBox()
        self.sales_by_day_year.setObjectName(f"combo_by_day_year")
        self.sales_by_day_year.setGeometry(x, y, w, h)
        for i in range(2024,2015,-1):
            self.sales_by_day_year.addItem(f"{i}년")

        self.sales_by_day_year.setParent(sales_groupBox)

    def make_combobox_month_by_day(self,sales_groupBox, x,y,w,h):
        self.sales_by_day_month = QComboBox()
        self.sales_by_day_month.setObjectName(f"combo_by_day_month")
        self.sales_by_day_month.setGeometry(x, y, w, h)
        for i in range(1,13):
            self.sales_by_day_month.addItem(f"{i}월")

        self.sales_by_day_month.setParent(sales_groupBox)    
        
    def make_store_button(self,sales_groupBox, x,y,w,h,addwidth):

        results=self.dbm.get_stores()

        for name in results:
            
            button_store = QPushButton(name[0])
            button_store.setObjectName(f"button_store_{str(name[0])}")
            button_store.setParent(sales_groupBox)
            #button_layout = QVBoxLayout()
            
            button_store.setGeometry(x, y, w, h)
            y = y+addwidth

            button_store.clicked.connect(partial(self.store_button_state, button_store))
            

    def store_button_state(self, button_store):
        self.button_text = button_store.text()
        
        self.store_label_month.setText(f"{self.button_text}의 월 매출현황")
        self.store_label_day.setText(f"{self.button_text}의 일 매출현황")

    def make_exit_button(self):
        self.search_by_month_year = QPushButton("나가기")
        self.search_by_month_year.setObjectName(f"exit")
        self.search_by_month_year.setGeometry(10,560,100,60)

        self.search_by_month_year.setParent(self.sales_groupBox)
        
        self.search_by_month_year.clicked.connect((self.exit))

    def exit(self):
        self.sales_groupBox.hide()         



if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SalesWindow()
    window.show()
    sys.exit(app.exec_())