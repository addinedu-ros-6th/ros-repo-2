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
from servee_gui.vendor_gui.Servee_sales_gui import SalesWindow

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

            except queue.Empty:
                time.sleep(0.03) 

    def stop(self):
        self.running = False    

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

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("./src/servee_gui/vendor_gui/servee_vendor.ui", self)

        self.setWindowTitle("SERVEE GUI")

        self.salesWindow = SalesWindow()

        self.store_widget = QWidget(self)
        #self.setCentralWidget(self.store_widget)
        self.store_layout = QVBoxLayout(self.store_widget)
        
         
        self.store_layout.setContentsMargins(10,30, 10, 60)
        
        date_label = QLabel()
        current_date = QDate.currentDate()
        formatted_date = current_date.toString("yyyy년 MM월 dd일")
        date_label.setText(f"<b><center>Today</center>{formatted_date}</b>")
        
        date_label.setFixedSize(131, 41)
        date_label.move(40, 30)
        date_label.setParent(self)
       
        #self.store_layout.addWidget(date_label)
        #
        #self.store_layout.setContentsMargins(10, 1, 10, 1)
        
        self.store_comboBox = QComboBox(self)
        self.store_comboBox.setFixedSize(211, 25)
        self.store_comboBox.move(40, 550)
        self.store_comboBox.setParent(self)
        # 서버 정보
        self.host = "localhost"
        self.port = 9999

        # 데이터베이스 연결
        self.dbm = MySQLConnection.getInstance()
        self.ob_client = ClientObserver(self.host, self.port, order_queue)

        # 스토어 인스턴스 저장
        self.stores = {}
        self.store_table_dic = {} 
        self.call_states = {}

        self.sales_button = QtWidgets.QPushButton("매출현황",self)
        self.sales_button.setObjectName("button_sales")
        self.sales_button.move(820, 20)
        self.sales_button.clicked.connect(self.sales_page)
        self.sales_button.setParent(self)

        #self.sales_button_2.clicked.connect(self.test)
        items = self.dbm.get_store_info()
        for item in items:
            if(str(item[0])=="101"):
                continue
            store_id = str(item[0])
            store_name = item[1]
            store_instance = Store(store_id)
            self.stores[store_id] =store_instance
            self.store_comboBox.addItem(store_name)

            store_table_widget = store_instance.get_table_widget()
            store_table_widget.setFixedSize(921, 361)
            store_table_widget.move(40, 120)
            self.store_layout.addWidget(store_table_widget)
            self.store_table_dic[store_id] = store_table_widget
            store_table_widget.setParent(self)
        
        # 나머지 초기화 코드...
        self.client_first_receive = threading.Thread(target=self.ob_client.receive_updates)
        self.client_first_receive.start()

        self.worker1 = Worker()
        self.worker1.update_signal.connect(self.make_orderlist)
        self.worker1.start()

        #self.orderlist_tableWidget.setColumnHidden(6, True)
        self.order_count = 1

        for widget in self.store_table_dic.values():
            widget.hide()

        
        self.store_table_dic[str(items[0][0])].show()
        self.store_comboBox.currentIndexChanged.connect(self.on_combobox_changed)

        ##############################매출 그래프 불러오는 부분############################################


    def sales_page(self):
    
        self.box = self.salesWindow.make_groupbox(40,10,1021,631)
        
        self.salesWindow.make_store_button(self.box, 20,60,120,60,100)

        self.salesWindow.make_label_month(650,-20,200,80)
        self.salesWindow.make_combobox_by_month(self.box, 300,60,80,60)
        self.salesWindow.make_button_search_by_month(self.box, 300,150,80,60,400,50,741,251)
        self.salesWindow.make_graph_month(self.box,400,50,741,251)

        self.salesWindow.make_label_day(650,290,200,80)
        self.salesWindow.make_combobox_year_by_day(self.box, 300,360,80,60)
        self.salesWindow.make_combobox_month_by_day(self.box, 300,450,80,60)
        self.salesWindow.make_button_search_by_day(self.box, 300,540,80,60,400,360,741,251)
        self.salesWindow.make_graph_day(self.box,400,360,741,251)
        self.salesWindow.make_exit_button()

        self.box.setParent(self)
  
        self.box.show()
        
    def on_combobox_changed(self):
        # 스토어 이름 변경 시 주문 목록 업데이트
        self.update_order_list()


    def find_store_table_widget(self, store_id):
        table_widget = self.findChild(QTableWidget, f"table_widget_{store_id}")
        return table_widget

    def update_order_list(self):
        

        # 현재 선택된 스토어에 대한 주문 목록을 가져옵니다.
        self.store_name = self.store_comboBox.currentText()
        store_id = self.dbm.get_store_id_vendor(self.store_name)[0][0]  # 선택된 스토어 ID 가져오기
        print(self.store_table_dic)
        current_store = self.stores[str(store_id)]

        for widget in self.store_table_dic.values():
            widget.hide()

        self.orderlist_tableWidget = self.find_store_table_widget(store_id)

        self.orderlist_tableWidget.setRowCount(0)
        
        self.order_count = 1  # 주문 카운트 초기화
        
        print(current_store.order_data)

        for order_id, order_data in current_store.order_data.items():
     
            print(len(order_data))
            row_count = self.orderlist_tableWidget.rowCount()
            print(row_count)
            for row_index, row in enumerate(order_data):
                self.orderlist_tableWidget.insertRow(row_count+row_index)
                self.orderlist_tableWidget.setItem(row_count+row_index, 0, QTableWidgetItem(str(self.order_count)))
                self.order_count += 1

                for column_index, item in enumerate(row):
                       
                    if(column_index==0):
                        continue
                    self.orderlist_tableWidget.setItem(row_count+row_index, column_index, QTableWidgetItem(str(item)))

                self.orderlist_tableWidget.setItem(row_count+row_index, 4, QTableWidgetItem(self.call_states[order_id]))
                self.orderlist_tableWidget.setItem(row_count+row_index, 6, QTableWidgetItem(str(order_id)))

            
            robot_call_button = QPushButton("로봇호출", self)
            robot_call_button.setObjectName("button_serve_" + str(order_id))
            robot_call_button.clicked.connect(partial(self.cal_robot_serve, order_id,self.orderlist_tableWidget,current_store))
            
            self.orderlist_tableWidget.insertRow(row_count+len(order_data))
            self.orderlist_tableWidget.setCellWidget(row_count+len(order_data), 5, robot_call_button)
            self.orderlist_tableWidget.setItem(row_count+len(order_data), 6, QTableWidgetItem(str(order_id)))

            print("확인해보자 : ", order_id)
            if(current_store.get_button_status(order_id) == False):
                self.state_setfalse(order_id,self.orderlist_tableWidget)
                

        #    ## 주문 상태를 확인하여 이전 상태를 유지
        #    ##if order_id in self.stores[get_store_id].order_status:
        #    ##    self.update_order_ui(row_index, order_id)
        #    self.orderlist_tableWidget.setColumnHidden(6, True)


        self.store_table_dic[str(store_id)].show()
        
    def make_orderlist(self, result):
        self.store_name = self.store_comboBox.currentText()
        store_id_main = self.dbm.get_store_id_vendor(self.store_name)[0][0]
        if "CREATE" in result:
            
            order_id = result.split(',')[2]
            
            status = result.split(',')[3]
            self.call_states[order_id] = "조리중"
            #store_name = self.store_comboBox.currentText()
            #print(f"리스트 만드는 오더아이디 : {order_id} for store {store_name}")
            store_info = self.dbm.get_store_info()
            order_results = self.dbm.get_order_store_details(order_id)

            for store_id, store_name in store_info:
                if(store_id==101):
                    continue
                #print("결과 : ", order_results)

                for index, value in enumerate(order_results):
                    if(store_id == value[0]):
                       
                        self.stores[str(value[0])].add_order(order_id, value)

            self.orderlist_tableWidget = self.find_store_table_widget(store_id_main)
            self.orderlist_tableWidget.setRowCount(0)
            
            self.order_count = 1  # 주문 카운트 초기화        
            current_store = self.stores[str(store_id_main)]
                        
            for order_id, order_data in current_store.order_data.items():
            
                print(len(order_data))
                row_count = self.orderlist_tableWidget.rowCount()
                
                for row_index, row in enumerate(order_data):
                    self.orderlist_tableWidget.insertRow(row_count+row_index)
                    self.orderlist_tableWidget.setItem(row_count+row_index, 0, QTableWidgetItem(str(self.order_count)))
                    self.order_count += 1
                    for column_index, item in enumerate(row):
                        print("데이터 확인 : ", item)
                        if(column_index==0):
                            continue
                        self.orderlist_tableWidget.setItem(row_count+row_index, column_index, QTableWidgetItem(str(item)))
                    self.orderlist_tableWidget.setItem(row_count+row_index, 4, QTableWidgetItem(self.call_states[order_id]))
                    self.orderlist_tableWidget.setItem(row_count+row_index, 6, QTableWidgetItem(str(order_id)))

                robot_call_button = QPushButton("로봇호출", self)
                robot_call_button.setObjectName("button_serve_" + str(order_id))
                robot_call_button.clicked.connect(partial(self.cal_robot_serve, order_id, self.orderlist_tableWidget,current_store))
                self.orderlist_tableWidget.insertRow(row_count+len(order_data))
                self.orderlist_tableWidget.setCellWidget(row_count+len(order_data), 5, robot_call_button)
                self.orderlist_tableWidget.setItem(row_count+len(order_data), 6, QTableWidgetItem(str(order_id)))

                if(current_store.get_button_status(order_id) == False):
                    self.state_setfalse(order_id,self.orderlist_tableWidget)
                self.center_text_in_cells()    
        elif "UPDATE" in result:

            #self.update_order_list(result)
            order_id = result.split(',')[2]
            call_state = result.split(',')[3]

            self.call_states[order_id] = call_state

            # 콤보 박스 변경 시 상태를 업데이트
            self.update_order_list()

            #orderlist_tableWidget = self.find_store_table_widget(store_id_main)
            #row_count = orderlist_tableWidget.rowCount()
            #column_count = orderlist_tableWidget.columnCount()
#
            #for row in range(row_count):
            #    item = self.orderlist_tableWidget.item(row, column_count-1).text()
            #    print("item : ",item)
            #    print("order_id : ",order_id)
            #    if(item == order_id):
            #        if call_state =="waiting_serverbot":
            #            call_state = "로봇 호출중"
            #        self.orderlist_tableWidget.setItem(row, 4, QTableWidgetItem(call_state))
                       
                    #if(current_store.get_button_status(order_id)==False):
                #   for i in range(self.orderlist_tableWidget.rowCount()):
            #
                #        if self.orderlist_tableWidget.item(i, 6).text() == order_id:
                #            for j in range(self.orderlist_tableWidget.columnCount()):
                #                item = self.orderlist_tableWidget.item(i, j)
                #                if item:
                #                    font = item.font()
                #                    font.setWeight(QFont.Light)
                #                    item.setFont(font)
                #                    item.setForeground(Qt.gray)
#           #
                #        robot_call_button = self.findChild(QPushButton, "button_serve_" + str(order_id))
                #        if robot_call_button:
                #            robot_call_button.setEnabled(False) 
            #for store_id, store in self.stores.items():
            #    print(f"Store ID: {store_id}")
            #    print(f"Order Data: {store.order_data}")             

            #for row_index, row in enumerate(order_results):
            #    self.orderlist_tableWidget.insertRow(row_count + row_index)
            #    self.orderlist_tableWidget.setItem(row_count + row_index, 0, QTableWidgetItem(str(self.order_count)))
            #    self.order_count += 1
#
            #    for column_index, item in enumerate(row):
            #        self.orderlist_tableWidget.setItem(row_count + row_index, column_index + 1, QTableWidgetItem(str(item)))
            #        if column_index == 2:
            #            item = str(item).split(' ')[1]
            #            self.orderlist_tableWidget.setItem(row_count + row_index, column_index + 1, QTableWidgetItem(str(item)))
#
            #    self.orderlist_tableWidget.setItem(row_count + row_index, 4, QTableWidgetItem("조리중"))
            #    self.orderlist_tableWidget.setItem(row_count + row_index, 6, QTableWidgetItem(str(order_id)))

                # 새 주문 상태 저장
            
                

            # 로봇 호출 버튼 추가
            #self.orderlist_tableWidget.insertRow(row_count + len(order_results))
            #self.robot_call_button = QPushButton("로봇호출", self)
            #self.robot_call_button.setObjectName("button_serve_" + str(order_id))
            #self.robot_call_button.clicked.connect(partial(self.cal_robot_serve, order_id))
            #self.orderlist_tableWidget.setCellWidget(row_count + len(order_results), 5, self.robot_call_button)
            #self.orderlist_tableWidget.setItem(row_count + len(order_results), 6, QTableWidgetItem(str(order_id)))
#
            #self.orderlist_tableWidget.setColumnHidden(6, True)
            #self.center_text_in_cells()

    #def update_order_ui(self, row_index, order_id):
    #    # 주문 상태에 따라 UI 업데이트
    #    store_name = self.store_comboBox.currentText()
#
    #    if self.stores[store_name].get_order_status(order_id) == "서빙중":
    #        for j in range(self.orderlist_tableWidget.columnCount()):
    #            item = self.orderlist_tableWidget.item(row_index, j)
    #            if item:
    #                font = item.font()
    #                font.setWeight(QFont.Light)
    #                item.setFont(font)
    #                item.setForeground(Qt.gray)
#
    #        robot_call_button = self.findChild(QPushButton, "button_serve_" + str(order_id))
    #        if robot_call_button:
    #            robot_call_button.setEnabled(False)

    def cal_robot_serve(self, order_id, orderlist_tableWidget,current_store):
        self.store_name = self.store_comboBox.currentText()
        #store_id = self.dbm.get_store_id_vendor(self.store_name)[0][0]  # 선택된 스토어 ID 가져오기
        
        print("오더아이디 타입", type(order_id))
        #orderlist_tableWidget = self.find_store_table_widget(store_id)
        print("개수 : ", orderlist_tableWidget.rowCount())
        print("데이터 확인",orderlist_tableWidget.item(1, 6).text())

        #for i in range(orderlist_tableWidget.rowCount()):
        #    
        #    if orderlist_tableWidget.item(i, 6).text() == order_id:
        #        for j in range(orderlist_tableWidget.columnCount()):
        #            item = orderlist_tableWidget.item(i, j)
        #            if item:
        #                font = item.font()
        #                font.setWeight(QFont.Light)
        #                item.setFont(font)
        #                item.setForeground(Qt.gray)
##
        #    robot_call_button = self.findChild(QPushButton, "button_serve_" + str(order_id))
        #    if robot_call_button:
        #        robot_call_button.setEnabled(False)
        self.state_setfalse(order_id, orderlist_tableWidget)    
        current_store.update_button_status(order_id,False)
        self.center_text_in_cells()
        self.ob_client.send_update_command("SE",order_id,"waiting_serverbot")
#
        #self.stores[store_name].update_order_status(order_id, "서빙중")  # 상태 업데이트
        #self.ob_client.send_update_command("SE", order_id, "서빙중")

    def center_text_in_cells(self):
        row_count = self.orderlist_tableWidget.rowCount()
        column_count = self.orderlist_tableWidget.columnCount()
        for row in range(row_count):
            for column in range(column_count):
                item = self.orderlist_tableWidget.item(row, column)
                if item is not None:
                    item.setTextAlignment(Qt.AlignCenter)

    def state_setfalse(self,order_id, orderlist_tableWidget):                
        for i in range(orderlist_tableWidget.rowCount()):
            
            if orderlist_tableWidget.item(i, 6).text() == order_id:
                for j in range(orderlist_tableWidget.columnCount()):
                    item = orderlist_tableWidget.item(i, j)
                    if item:
                        font = item.font()
                        font.setWeight(QFont.Light)
                        item.setFont(font)
                        item.setForeground(Qt.gray)
#       
                #robot_call_button = self.findChild(QPushButton, "button_serve_" + str(order_id))
                robot_call_button = orderlist_tableWidget.cellWidget(i, 5)
                if robot_call_button:
                    print("여기 오더 아이디", order_id)
                    print("여기까지 들어오나?")
                    robot_call_button.setEnabled(False)
            
    def closeEvent(self, event):
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())