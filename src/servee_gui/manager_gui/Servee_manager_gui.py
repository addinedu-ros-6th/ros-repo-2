from PyQt5.QtWidgets import QMainWindow, QDialog, QApplication, QTableWidgetItem, QHeaderView
from PyQt5.QtGui import QPixmap, QPainter, QPen
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, QDate
from PyQt5 import uic

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path

from datetime import datetime
import queue
import threading
import sys
import time

from etc.db.dbtest_connpull import MySQLConnection
from servee_gui.observer_subscriber import ClientObserver


'''
TODO:
### 장애물이랑 로봇 회피하는 행동 시각화 할거 준비하기
### 특히 로봇은 서로의 경로까지 보여주기

1. 로봇 자세 현황 검색 기능 구현 okay
2. 사장 호출 버튼 수신 구현하기
3. 맵에서 로봇마다 색상 다르게 하기 okay
4. 매장별 매출 현황 띄우기 - 보류
5. robot status 윈도우 너비랑 높이 조정하기
+
- 로봇 위치랑 경로 3개까지 동시에 다 해보기
- manager에서 주문 인스턴스 없애기

소켓으로 받을 데이터:
- 서빙(수거) 횟수
- 매장별 주문현황 주문 상태

토픽으로 받을 데이터:
- 주행거리 관련 데이터  <- 이건 임의로 계산 때리기

Log table dummy data:
insert into Log value (1, 1, 1, 1, 'SE', '2024-11-11 12:00:00', '2024-11-11 12:00:00', '2024-11-11 13:00:00');
'''


robot_info_queue = queue.Queue()

# 옵저버 클라이언트의 데이터 가져오는 클래스
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
                results = robot_info_queue.get()
                if("SE" in results):
                    self.update_signal.emit(results.replace(" ", ""))
                else:
                    pass
            except queue.Empty:
                time.sleep(0.03) 

    def stop(self):
        self.running = False

# 로봇 토픽 구독 노드
class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        self.robots_pose = [None, None, None]
        self.robots_path = [None, None, None]
        self.robots_state = [None, None, None]
        self.robots_battery = [100, 100, 100]

        self.pose_sub_1 = self.create_subscription(Pose, '/pose', self.robot_pose_callback(0), 10)
        self.pose_sub_2 = self.create_subscription(Pose, '/robot2/pose', self.robot_pose_callback(1), 10)
        self.pose_sub_3 = self.create_subscription(Pose, '/robot3/pose', self.robot_pose_callback(2), 10)

        self.path_sub_1 = self.create_subscription(Path, '/plan', self.robot_path_callback(0), 10)  # 토픽이름 꼭 확인하기
        self.path_sub_2 = self.create_subscription(Path, '/robot2/plan', self.robot_path_callback(1), 10)
        self.path_sub_3 = self.create_subscription(Path, '/robot3/plan', self.robot_path_callback(2), 10)

        self.state_sub_1 = self.create_subscription(String, '/state', self.robot_state_callback(0), 10)
        self.state_sub_2 = self.create_subscription(String, '/robot2/state', self.robot_state_callback(1), 10)
        self.state_sub_3 = self.create_subscription(String, '/robot3/state', self.robot_state_callback(2), 10)

        # self.battery_sub_1 = self.create_subscription()
        # self.battery_sub_2 = self.create_subscription()
        # self.battery_sub_3 = self.create_subscription()
    

    def robot_pose_callback(self, robot_index):
        def callback(msg: Pose):
            self.robots_pose[robot_index] = msg
        return callback
    
    def robot_path_callback(self, robot_index):
        def callback(msg: Path):
            self.robots_path[robot_index] = msg.poses
        return callback
    
    def robot_state_callback(self, robot_index):
        def callback(msg: String):
            self.robots_state[robot_index] = msg.data
        return callback


ui_info = uic.loadUiType('./src/servee_gui/manager_gui/servee_manager.ui')[0]
class ManagerGUI(QMainWindow, ui_info):
    def __init__(self, node: SubscriberNode):
        super().__init__()
        self.setupUi(self)
        self.node = node

        # 통신 연결
        ip = "192.168.0.19"
        port = 3939
        self.ob_client = ClientObserver(ip, port, robot_info_queue)
        self.ob_client_thread = threading.Thread(target=self.ob_client.receive_updates)
        self.ob_client_thread.start()

        self.worker = Worker()
        self.worker.update_signal.connect(self.get_status)
        self.worker.start()

        self.status_list = []  # list[tuple[instance_id, status]]  (instance_id = order_id, instance_id = table_id)

        # DB 연결
        self.dbm = MySQLConnection.getInstance()  #########################################

        # 지도 사진 삽입
        self.pixmap = QPixmap('./src/servee_gui/manager_gui/maps/map_final_3_scaled.pgm')
        self.label_map_1.setPixmap(self.pixmap)
        self.label_map_2.setPixmap(self.pixmap)

        # 로봇 상태 매핑
        self.robot_serving_state_map = {"idle": "충전중", "running1": "픽업중", "standby1": "음식 대기중", "running2": "배달중", "standby2": "수령 대기중"}
        self.robot_retrieving_state_map = {"idle": "충전중", "running1": "수거중", "standby1": "수거 대기중", "running2": "퇴식중", "standby2": "퇴식 대기중"}
        self.color_map = [Qt.red, Qt.green, Qt.blue]
        self.color_map_text = ['red', 'green', 'blue']

        # 로봇 현황 시작
        self.robot_type = []  # [{robot_id: type}, ...]
        self.robots_pose = [None, None, None]
        self.robots_path = [None, None, None]
        self.robots_state = [None, None, None]
        self.robots_battery = [None, None, None]

        robots = self.dbm.get_robots()
        header = self.table_robots_status.horizontalHeader()
        self.table_robots_status.setColumnWidth(0, 120)
        self.table_robots_status.setColumnWidth(1, 80)
        self.table_robots_status.setColumnWidth(2, 40)
        header.setStretchLastSection(True)
        self.table_robots_status.setRowCount(len(robots))

        for i, robot_info in enumerate(robots):
            id_1 = QTableWidgetItem(f"Servee_Robot_{robot_info[0]}")
            self.robot_type.append({id_1.text(): robot_info[1]})
            _state = QTableWidgetItem(" ")
            _battery = QTableWidgetItem("100")

            id_1.setTextAlignment(Qt.AlignCenter)
            _state.setTextAlignment(Qt.AlignCenter)
            _battery.setTextAlignment(Qt.AlignCenter)

            self.table_robots_status.setItem(i, 0, id_1)
            self.table_robots_status.setItem(i, 1, _state)
            self.table_robots_status.setItem(i, 2, _battery)
    
        # 일정 주기마다 지도 업데이트
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_map)
        self.timer.start(100)

        # 주문 현황 조회 탭 클릭 시 DB 데이터 조회
        self.tab_manager.currentChanged.connect(self.tab_clicked_callback)

        # 표 더블클릭 시 자세한 현황 조회
        self.table_robots_status.cellDoubleClicked.connect(self.table_robot_dclicked)
        self.table_stores_status.cellDoubleClicked.connect(self.table_store_dclicked)

        # 매출 현황 조회에서 버튼 클릭하면 식당 페이지로 이동
        self.btn_store_1.clicked.connect(lambda: self.show_content(0))
        self.btn_store_2.clicked.connect(lambda: self.show_content(1))
        self.btn_store_3.clicked.connect(lambda: self.show_content(2))


    def __del__(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.ob_client.stop()
        self.worker.stop()  # 이것들 어떻게 멈춤?


    def get_status(self, result):
        print("data result:", result)
        if "UPDATE" in result:
            result_list = result.split(", ")
            print("result_list:", result_list)

            instance_id = result_list[2]
            status = result_list[3]
            
            # 새로운 instance_id가 들어왔다면 리스트에 append하고, 
            # 기존의 instance_id가 들어왔다면 그 값을 업데이트 한다.
            for dict_item in self.status_list:
                if instance_id in dict_item:
                    dict_item[instance_id] = status
                    break
            
            self.status_list.append({instance_id: status})

    def update_map(self):
        rclpy.spin_once(self.node, timeout_sec=0.01)  # 제일 최근값 가져오는지 확인 필요. 
        
        # try:
        # 맵 상의 로봇 위치와 경로 업데이트/그리기
        self.robots_pose = self.node.robots_pose
        self.robots_path = self.node.robots_path
        self.robots_state = self.node.robots_state
        self.robots_battery = self.node.robots_battery

        # 그리기 예제
        pixmap_copy = self.pixmap.copy()
        
        painter = QPainter(pixmap_copy)
        # pen = QPen(Qt.red, 5)
        # painter.setPen(pen)

        for i in range(len(self.robots_pose)):
            if self.robots_pose[i] is not None:
                pen = QPen(self.color_map[i], 5)
                painter.setPen(pen)

                self.label_color_1.setText(f"Robot {i+1}: {self.color_map_text[i]}")
                self.label_color_1.setStyleSheet(f"Color : {self.color_map_text[i]}")
                painter.drawEllipse(int(540 - 540/2.3*self.robots_pose[0].position.y), int(396 - 396/1.7*self.robots_pose[0].position.x), 30, 30)  # 408 대신에 396 확인해보기

                # 경로 그리기
                try:
                    for path in self.robots_path[0]:
                        painter.drawEllipse(int(540 - 540/2.3*path.pose.position.y), int(408 - 408/1.7*path.pose.position.x), 3, 3)
                except:
                    pass

        painter.end()

        self.label_map_1.setPixmap(pixmap_copy)
        self.label_map_2.setPixmap(pixmap_copy)

        # 로봇 현황 업데이트
        for i in range(3):
            if self.robot_type[i]['Servee_Robot_'+str(i+1)] == "서빙용":
                item_1 = QTableWidgetItem(self.robot_serving_state_map[self.robots_state[i]])
            else:
                item_1 = QTableWidgetItem(self.robot_retrieving_state_map[self.robots_state[i]])
            # item_1 = QTableWidgetItem(self.robots_state[i])
            item_2 = QTableWidgetItem(self.robots_battery[i])
            item_1.setTextAlignment(Qt.AlignCenter)
            item_2.setTextAlignment(Qt.AlignCenter)
            self.table_robots_status.setItem(i, 1, item_1)
            self.table_robots_status.setItem(i, 2, item_2)

        # except Exception as e:
        #     pass

    def tab_clicked_callback(self, index):
        if index == 1:
            stores = self.dbm.get_stores()
            store_order_recap = self.dbm.total_orders_today()  # store_order_recap: [(store_id: int, order_count: int), ...]
            store_earning_recap = self.dbm.total_earning_today()

            self.table_stores_status.setRowCount(len(stores))
            header = self.table_stores_status.horizontalHeader()
            self.table_stores_status.setColumnWidth(0, 110)
            self.table_stores_status.setColumnWidth(1, 95)
            self.table_stores_status.setColumnWidth(2, 95)
            header.setStretchLastSection(True)

            for i, store_name in enumerate(stores):
                name = QTableWidgetItem(store_name[0])
                _orders = QTableWidgetItem('0')
                _earning = QTableWidgetItem('0')
                name.setTextAlignment(Qt.AlignCenter)
                _orders.setTextAlignment(Qt.AlignCenter)
                _earning.setTextAlignment(Qt.AlignCenter)
                self.table_stores_status.setItem(i, 0, name)
                self.table_stores_status.setItem(i, 1, _orders)
                self.table_stores_status.setItem(i, 2, _earning)
            
            for i, orders in enumerate(store_order_recap):
                store_id, count = orders[0], orders[1]
                order_count = QTableWidgetItem(str(count))
                order_count.setTextAlignment(Qt.AlignCenter)
                self.table_stores_status.setItem(store_id-1, 1, order_count)
            
            for i, earnings in enumerate(store_earning_recap):
                store_id, earning = earnings[0], earnings[1]
                store_earning = QTableWidgetItem(f"{int(earning):,} 원")
                store_earning.setTextAlignment(Qt.AlignCenter)
                self.table_stores_status.setItem(store_id-1, 2, store_earning)

    def table_robot_dclicked(self, row, column):
        def date_start_callback():
            self.start_date = robot_status.date_start.date()
            print("start_date:", self.start_date)
        
        def date_end_callback():
            self.end_date = robot_status.date_end.date()
            print("end_date:", self.end_date)
        
        def search_callback():
            start_date_string = self.start_date.toString("yyyy-MM-dd")
            end_date_string = self.end_date.toString("yyyy-MM-dd")

            log_data = self.dbm.get_robot_log(robot_id, start_date_string, end_date_string)
            print("log_data:", log_data)

            robot_status.label_count_value.setText(str(len(log_data)))
            robot_status.table_history.setRowCount(len(log_data))

            for i, data in enumerate(log_data):
                table_num = QTableWidgetItem(str(data[0]))
                order_time = QTableWidgetItem(str(data[1]))

                table_num.setTextAlignment(Qt.AlignCenter)
                order_time.setTextAlignment(Qt.AlignCenter)

                robot_status.table_history.setItem(i, 0, table_num)
                robot_status.table_history.setItem(i, 1, order_time)

        robot_status = RobotStatus()

        self.start_date = QDate.currentDate()
        self.end_date = QDate.currentDate()

        robot_status.date_start.setDate(self.start_date)
        robot_status.date_end.setDate(self.end_date)

        header = robot_status.table_history.horizontalHeader()
        robot_status.table_history.setColumnWidth(0, 100)
        robot_status.table_history.setColumnWidth(1, 100)
        header.setStretchLastSection(True)

        robot_id = self.table_robots_status.item(row, 0).text()[-1]
        if self.dbm.get_robot_type(robot_id)[0][0] == "서빙용":
            robot_status.label_robot_status.setText("주문 상태")
            robot_status.label_count_title.setText("서빙 횟수")
        else:
            robot_status.label_robot_status.setText("서빙 상태")
            robot_status.label_count_title.setText("수거 횟수")

        search_callback()
        
        robot_status.date_start.dateChanged.connect(date_start_callback)
        robot_status.date_end.dateChanged.connect(date_end_callback)
        robot_status.btn_search.clicked.connect(search_callback)

        robot_status.exec_()

    def table_store_dclicked(self, row, column):
        dialog = StoreStatus()

        name = self.table_stores_status.item(row, 0).text()
        dialog.label_store_name.setText(name)

        date = datetime.now().strftime("%Y-%m-%d")
        dialog.label_datetime.setText(date)
        
        store_id = self.dbm.get_store_id(name)[0][0]

        order_details = self.dbm.order_status(store_id)
        dialog.table_order_details.setRowCount(len(order_details))
        header = dialog.table_order_details.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeToContents)

        for i, order_detail in enumerate(order_details):
            # order_detail: menu_name, quantity, call_time, order_id
            order_time_raw = order_detail[2].strftime("%Y-%m-%d %H:%M:%S")

            order_status_raw = ""
            order_id = order_detail[3]
            for dict_item in self.status_list:
                if order_id in dict_item:
                    order_status_raw = dict_item[order_id]

            name = QTableWidgetItem(str(order_detail[0]))
            count = QTableWidgetItem(str(order_detail[1]))
            order_time = QTableWidgetItem(order_time_raw)
            order_status = QTableWidgetItem(str(order_status_raw))  # 주문 상태 정해진 이름으로 매핑하고 적용하기

            name.setTextAlignment(Qt.AlignCenter)
            count.setTextAlignment(Qt.AlignCenter)
            order_time.setTextAlignment(Qt.AlignCenter)
            order_status.setTextAlignment(Qt.AlignCenter)

            dialog.table_order_details.setItem(i, 0, name)
            dialog.table_order_details.setItem(i, 1, count)
            dialog.table_order_details.setItem(i, 2, order_time)
            dialog.table_order_details.setItem(i, 3, order_status)

        dialog.exec_()
    
    def show_content(self, index):
        self.stacked_store_info.setCurrentIndex(index)


class RobotStatus(QDialog):
    def __init__(self):
        super().__init__()
        uic.loadUi('./src/servee_gui/manager_gui/robot_status_gui.ui', self)


class StoreStatus(QDialog):
    def __init__(self):
        super().__init__()
        uic.loadUi('./src/servee_gui/manager_gui/order_status_gui.ui', self)


def main():
    rclpy.init(args=None)

    node = SubscriberNode()

    app = QApplication(sys.argv)
    window = ManagerGUI(node)

    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()