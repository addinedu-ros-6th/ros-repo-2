import sys
import os
import socket
import ast
import threading
current_dir = os.path.dirname(os.path.abspath(__file__)) # 현재 스크립트의 디렉토리를 가져오고, 프로젝트 루트로 이동하는 상대 경로를 추가

relative_path = os.path.join(current_dir, '..', '..') # 상위 폴더로 이동
absolute_path = os.path.abspath(relative_path) 

sys.path.append(relative_path)
print(relative_path)
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


from PyQt5.QtCore import Qt 
import math
#from qt_material import apply_stylesheet
#import cv2
from PyQt5 import QtWidgets, uic
from functools import partial
import queue
###########################################db sql 사용 규칙 ###########################################
# .1 db에 요청을 날리는 경우에는 함수명을 첫번째 인덱스에 쓰고 뒤에 데이터를 넣는다



class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("./src/servee_gui/customer_gui/servee_customer.ui",self)

        self.order_queue = queue.Queue()

        
        self.setWindowTitle("SERVEE GUI")
        self.cancel_index =0
        self.result_price=0
        self.port = 9999
        self.host = "localhost"

        self.running =True
        self.ob_client = ClientObserver(self.host, self.port, self.order_queue)

        self.dbm = MySQLConnection.getInstance()

        self.client_first_receive = threading.Thread(target=self.ob_client.receive_updates)
        self.client_first_receive.start()

        self.client_state_update = threading.Thread(target=self.receive_state)
        self.client_state_update.start()

        self.scrollArea_1.setVisible(True)
        self.scrollArea_2.setVisible(False)
        self.scrollArea_3.setVisible(True)
        self.button_retrieve.setEnabled(True) 

        self.menu_list_make('scrollArea_1',1)
        self.menu_list_make('scrollArea_2',2)
        self.menu_list_make('scrollArea_3',3)
        
        self.shopping_tableWidget.setEditTriggers(QTableWidget.NoEditTriggers)
        self.vendor_1_button.clicked.connect(self.china_scroallArea_1_show)
        self.vendor_2_button.clicked.connect(self.china_scroallArea_2_show)
        self.vendor_3_button.clicked.connect(self.italy_scroallArea_3_show)

        self.selected_item = self.table_comboBox.currentText()
        self.selected_item  = self.selected_item .split(' ')
        self.table_num = self.selected_item [1]
        self.table_comboBox.currentIndexChanged.connect(self.on_combobox_changed)
        

        self.button_payment.clicked.connect(self.payment_alarm)

        self.button_service_call.clicked.connect(self.service_alarm)
        self.order_tableWidget.setColumnHidden(5, True)
        self.button_retrieve.clicked.connect(self.retrieve_alarm)

        self.shopping_tableWidget.horizontalHeader().setStretchLastSection(True)  # 마지막 열이 남은 공간을 차지하도록 설정
        self.order_tableWidget.horizontalHeader().setStretchLastSection(True)

    def on_combobox_changed(self):
        selected_item = self.table_comboBox.currentText()
        selected_item = selected_item.split(' ')
        self.table_num = selected_item[1]


    def data_select_receive(self, data):
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect((self.tcp_ip, self.port))
            
            # 정수 전송
            client.send(str(data).encode('utf-8'))
            # 서버로부터 응답 수신
            self.results = client.recv(4096).decode('utf-8')
            self.results = ast.literal_eval(self.results)
            
        except Exception as e:
            pass
        finally:
            client.close()
        return self.results

    def menu_list_make(self, scroll_area_name="scrollArea_1",store_id=1):
        
        self.scroll_area = self.findChild(QtWidgets.QScrollArea, scroll_area_name)  # scrollArea는 Qt Designer에서 설정한 이름입니다.
        layout = QtWidgets.QGridLayout(self.scroll_area)
        layout.setVerticalSpacing(1) 

        menu_results=self.dbm.get_order_menu(store_id)
        if(len(menu_results)!=0):
            for index, value in enumerate(menu_results):
                
                pixmap = QPixmap(value[4])
                
                self.button = QtWidgets.QPushButton(self)
                self.button.setObjectName("button_menu_" + str(value[1])+"_"+str(value[0]))

                self.button.clicked.connect(partial(self.shopping_cart_temp_save, value[1], value[0]))

                self.button.setFixedSize(120, 100)
                
                button_size = self.button.size()
                # 아이콘 생성
                scaled_pixmap = pixmap.scaled(button_size, Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
                self.button.setIcon(QIcon(scaled_pixmap))
                self.button.setIconSize(button_size) 

                text_label =QtWidgets.QLabel(f"{value[2]} : {value[3]}", self)  # 버튼 아래에s표시할 텍스트
                text_label.setAlignment(Qt.AlignCenter) 
                text_label.setAttribute(Qt.WA_TransparentForMouseEvents)

                if(index /3 ==0):
                    layout.addWidget(self.button,int(index /3),int(index%3))
                else:
                    layout.addWidget(self.button,int(index /3)+int(index/3),int(index%3))

                if(int(index /3)+1 ==1):
                    layout.addWidget(text_label, int(index /3)+1, index)
                else:
                    layout.addWidget(text_label,int(index /3)+2,int(index%3))
                  
            
            self.scroll_area.setLayout(layout)
            
    def china_scroallArea_1_show(self):  
       self.scrollArea_1.setVisible(True)
       self.scrollArea_2.setVisible(False)

    def china_scroallArea_2_show(self):  
       self.scrollArea_1.setVisible(False)
       self.scrollArea_2.setVisible(True)

    def italy_scroallArea_3_show(self):  

        self.scrollArea_3.setVisible(True)

    def shopping_cart_temp_save(self,store_id,menu_id):
        #data = f"get_order_detail_menu/{store_id}/{menu_id}"
        #menu_detail = self.data_select_receive(data)
        menu_detail = self.dbm.get_order_detail_menu(store_id,menu_id)
        total_list = []
        self.total_price=0
         
        if(len(menu_detail)!=0):
            for value in menu_detail:
                print("스토어 이름 : ", value[0])
                print("넌 뭐니 : ", value[1])
                item_found = False
                for row in range(self.shopping_tableWidget.rowCount()): 
                    
                    if(self.shopping_tableWidget.item(row, 0).text()==value[0] and self.shopping_tableWidget.item(row, 1).text()==value[1]):
                        
                        count = int(self.shopping_tableWidget.item(row, 2).text())+1
                        self.shopping_tableWidget.setItem(row, 0, QTableWidgetItem(str(value[0])))
                        self.shopping_tableWidget.setItem(row, 1, QTableWidgetItem(str(value[1])))
                        self.shopping_tableWidget.setItem(row, 2, QTableWidgetItem(str(count)))
                        self.shopping_tableWidget.setItem(row, 3, QTableWidgetItem(str(value[2])))
                        
                        item_found=True
                        
                        break

                if not item_found:
                    if(self.shopping_tableWidget.rowCount()==0):

                        self.cancel_index = self.cancel_index+1
                        button = QtWidgets.QPushButton("주문취소",self)
                        button.setObjectName("button_cancel_" + str(self.cancel_index))
                        row = self.shopping_tableWidget.rowCount()
                        self.shopping_tableWidget.insertRow(row)
                        self.shopping_tableWidget.setItem(row, 0, QTableWidgetItem(str(value[0])))
                        self.shopping_tableWidget.setItem(row, 1, QTableWidgetItem(str(value[1])))
                        self.shopping_tableWidget.setItem(row, 2, QTableWidgetItem(str(1)))
                        self.shopping_tableWidget.setItem(row, 3, QTableWidgetItem(str(value[2])))
                        self.shopping_tableWidget.setCellWidget(row, 4, button)

                        button.clicked.connect(self.order_cancel)

                    elif(self.shopping_tableWidget.item(0, 0).text()==value[0]):
                        self.cancel_index = self.cancel_index+1
                        button = QtWidgets.QPushButton("주문취소",self)
                        button.setObjectName("button_cancel_" + str(self.cancel_index))
                        row = self.shopping_tableWidget.rowCount()
                        self.shopping_tableWidget.insertRow(row)
                        self.shopping_tableWidget.setItem(row, 0, QTableWidgetItem(str(value[0])))
                        self.shopping_tableWidget.setItem(row, 1, QTableWidgetItem(str(value[1])))
                        self.shopping_tableWidget.setItem(row, 2, QTableWidgetItem(str(1)))
                        self.shopping_tableWidget.setItem(row, 3, QTableWidgetItem(str(value[2])))
                        self.shopping_tableWidget.setCellWidget(row, 4, button)
                        button.clicked.connect(self.order_cancel)
                    else:
                        self.store_diff_alarm()


                
        for row in range(self.shopping_tableWidget.rowCount()):
            if(int(self.shopping_tableWidget.rowCount())==0):
                
                break
            else:
                
                item_count = self.shopping_tableWidget.item(row,2).text()
                item_price = self.shopping_tableWidget.item(row,3).text()
                each_price = int(item_count)*int(item_price)
                self.total_price += each_price
        self.label_predict_price.setText(f"{str(self.total_price)}원")
                
                   

    def order_cancel(self):
        selected_row = self.shopping_tableWidget.currentRow()
        if selected_row >= 0:  # 유효한 행이 선택된 경우
            self.shopping_tableWidget.removeRow(selected_row)

    def payment_alarm(self):
        # 알람 메시지 박스 생성
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Information) 
        if(self.shopping_tableWidget.rowCount()==0):

            msg.setText("마 주문안하나")  # 메시지 텍스트 설정
            msg.setWindowTitle("주문")  # 창 제목 설정
            msg.setStandardButtons(QMessageBox.Yes)
            msg.exec_()

        else:
            msg.setText("결제 하시겠습니까?")  # 메시지 텍스트 설정
            msg.setWindowTitle("결제")  # 창 제목 설정
            msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
            # 메시지 박스 표시
            result = msg.exec_()
            if result == QMessageBox.Yes:
                self.confirm_payment()
            else:
                print("사용자가 NO를 선택했습니다.")

    def retrieve_alarm(self):
        msg = QMessageBox()     
        msg.setIcon(QMessageBox.Information) 
        msg.setText("그릇 회수 호출 하시겠습니까?")  # 메시지 텍스트 설정
        msg.setWindowTitle("회수")  # 창 제목 설정
        msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        result = msg.exec_()
        if result == QMessageBox.Yes:
            
            self.ob_client.send_create_command("RV", table_id=self.table_num)
            self.order_tableWidget.setRowCount(0)
            self.button_retrieve.setEnabled(False) 

    def service_alarm(self):
        # 알람 메시지 박스 생성
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Information) 
        msg.setText("직원 호출 하시겠습니까?")  # 메시지 텍스트 설정
        msg.setWindowTitle("호출")  # 창 제목 설정
        msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        # 메시지 박스 표시
        result = msg.exec_()
        if result == QMessageBox.Yes:
            self.dbm.insert_servicecall(self.table_num)
        else:
            print("사용자가 NO를 선택했습니다.")

    def store_diff_alarm(self):
        # 알람 메시지 박스 생성
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Information) 
        msg.setText("동일한 영업점에서 선택해주세요")  # 메시지 텍스트 설정
        msg.setWindowTitle("영업점")  # 창 제목 설정
        msg.setStandardButtons(QMessageBox.Yes)
        # 메시지 박스 표시
        msg.exec_()
    
                      

    def confirm_payment(self):
        
        row_count = self.shopping_tableWidget.rowCount()
        column_count = self.shopping_tableWidget.columnCount()
        order_id = self.dbm.insert_ordercalls(self.table_num)

        #db에 넣을 데이터 확인
        for row in range(row_count):
            raw_data = []
            #for column in range(2):
            store_name = self.shopping_tableWidget.item(row, 0).text()
            menu_name = self.shopping_tableWidget.item(row, 1).text()
            quantity = self.shopping_tableWidget.item(row, 2).text()

              # 특정 셀의 아이템 가져오기
            #    raw_data.append(item.text())  # 아이템의 텍스트 추가
            result=self.dbm.select_store_menu_id(store_name, menu_name)

            store_id =result[0][0]
            menu_id = result[0][1]
            #print("스토어아이디 : ", store_id)
            #print("오더아이디 : ", order_id)
            #print("테이블 넘버 : ", self.table_num)
            #print("수량 : ", quantity)
            #print("메뉴아이디 : ", menu_id)
            self.dbm.insert_orderdetails(order_id,menu_id, quantity)


        # 두 번째 테이블의 행 수를 업데이트
        current_row_count = self.order_tableWidget.rowCount()
        
        for row in range(row_count):
            self.order_tableWidget.insertRow(self.order_tableWidget.rowCount())

            for column in range(column_count-1):
                item = self.shopping_tableWidget.item(row, column).text()
                
                if item is not None:
                    # 첫 번째 테이블의 아이템을 복사하여 두 번째 테이블에 설정
                    new_item =QTableWidgetItem(item)
                    # 기존의 행 수 뒤에 새 아이템 추가
                    
                    self.order_tableWidget.setItem(current_row_count + row, column, new_item)

            self.order_tableWidget.setItem(current_row_count + row, column_count-1, QTableWidgetItem("주문 대기중"))
            self.order_tableWidget.setItem(current_row_count + row, column_count, QTableWidgetItem(str(order_id)))

            self.order_tableWidget.setColumnHidden(column_count, True)

            QTimer.singleShot(500, lambda row=row: self.updateOrderStatus(row+current_row_count))

        

        self.shopping_tableWidget.setRowCount(0)
        label_text = self.label_predict_price.text()  # 현재 label의 텍스트 가져오기
        price_only_number = ''.join(filter(str.isdigit, label_text)) 
        self.result_price += int(price_only_number)

        last_row_count = self.order_tableWidget.rowCount()
        state = self.order_tableWidget.item(last_row_count-1, 4).text()
        if state == "서빙완료":
            self.button_retrieve.setEnabled(True)
        else:
            self.button_retrieve.setEnabled(False) 

        self.label_total_price.setText((f"{str(self.result_price )}원"))
        self.label_predict_price.setText(f"0원")

        #vendor manager와 통신 하기
        self.client_thread = threading.Thread(target=self.ob_client.send_create_command, args=("SE", order_id, store_id,self.table_num))
        self.client_thread.start()

        
        
    def updateOrderStatus(self, row):
            # 주문 상태를 "조리 중"으로 변경
            self.order_tableWidget.item(row, 4).setText("조리 중")

    def receive_state(self):

        #데이터 수집
        while self.running:
            try:

                results = self.order_queue.get()
                print("데이터가 들어왔다는 증거")
                #빈공간 제거 해야 함
                if("UPDATE" in results):
                    self.update_ordertable(results.replace(" ", ""))
                else:
                    pass
            except queue.Empty:
                time.sleep(0.03)   

    def update_ordertable(self,results):
        
        print("업데이트 오더테이블 : ", results)

        command_type = results.split(',')[0]
        
        call_type = results.split(',')[1]
        order_id = results.split(',')[2]
        call_state = results.split(',')[3]
        row_count = self.order_tableWidget.rowCount()
        column_count = self.order_tableWidget.columnCount()
        
        for row in range(row_count):
            item = self.order_tableWidget.item(row, column_count-1).text()

            if(item == order_id):
                if call_state =="waiting_serverbot":
                    call_state = "서빙봇 대기중"
                    #call_state = "서빙완료"
                elif call_state =="waiting_handover":
                    call_state = "음식 인계 대기중"
                elif call_state =="serving":
                    call_state = "서빙중"
                elif call_state =="done_serving":
                    call_state = "서빙완료"             
                self.order_tableWidget.setItem(row, 4, QTableWidgetItem(call_state))
        
        
        state = self.order_tableWidget.item(row_count-1, 4).text()
        print("스테이트 뭐냐 : ",state)
        if state == "서빙완료":
            self.button_retrieve.setEnabled(True)
        else:
            self.button_retrieve.setEnabled(False)   

    def closeEvent(self, event):
        # 윈도우 종료 시 데이터베이스 연결 종료

        #self.dbm.disconnection()
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
