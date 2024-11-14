from PyQt5.QtWidgets import QMainWindow, QDialog, QApplication, QTableWidgetItem, QHeaderView
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, QTimer
from PyQt5 import uic

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

import datetime
import sys

from etc.db.dbtest_connpull import MySQLConnection


# ros2 노드 새로 만드는 버전

'''
TODO:
1. 맵이랑 로봇 위치 띄우기 - 일단 대기
2. 로봇별 현황 띄우기 - okay?
3. 매장별 주문 현황 띄우기 - 
4. 매출현황 조회에서 세로로 된 탭 고치기
5. 로봇별 자세 현황 띄우기
6. 매장별 자세 현황 띄우기
7. 매장별 매출 현황 띄우기
8. 서빙로봇이랑 수거로봇 구분
9. 지도에 로봇이랑 경로 그리기
10. 사장 호출 버튼 수신 구현하기

통신으로 받을 데이터:
- 로봇 상태
- 로봇 배터리
- 주행거리 관련 데이터
- 서빙(수거) 횟수
- 매장별 주문현황 주문 상태
'''


class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        self.robots_pose = (None, None, None)
        self.robots_path = (None, None, None)
        self.robots_state = (None, None, None)
        self.robots_battery = (None, None, None)

        self.robot1_pose = None
        self.robot2_pose = None
        self.robot3_pose = None

        self.robot1_path = None
        self.robot2_path = None
        self.robot3_path = None

        # self.pose_sub_1 = self.create_subscription()
        # self.pose_sub_2 = self.create_subscription()
        # self.pose_sub_3 = self.create_subscription()

        # self.path_sub_1 = self.create_subscription()
        # self.path_sub_2 = self.create_subscription()
        # self.path_sub_3 = self.create_subscription()

        # self.state_sub_1 = self.create_subscription()
        # self.state_sub_2 = self.create_subscription()
        # self.state_sub_3 = self.create_subscription()

        # self.battery_sub_1 = self.create_subscription()
        # self.battery_sub_2 = self.create_subscription()
        # self.battery_sub_3 = self.create_subscription()


from_class = uic.loadUiType('./src/servee_gui/manager_gui/manager_scaled.ui')[0]
class ManagerGUI(QMainWindow, from_class):
    def __init__(self, node: SubscriberNode):
        super().__init__()
        self.setupUi(self)
        self.node = node

        self.robots_pose = (None, None, None)
        self.robots_path = (None, None, None)

        # DB 연결
        self.dbm = MySQLConnection.getInstance()

        # 주문 현황 조회 탭 클릭 시 DB 데이터 조회
        self.tab_manager.currentChanged.connect(self.tab_clicked_callback)

        # 표 더블클릭 시 자세한 현황 조회
        self.table_robots_status_2.cellDoubleClicked.connect(self.table_robot_dclicked)
        self.table_stores_status_2.cellDoubleClicked.connect(self.table_store_dclicked)

        # 지도 사진 삽입
        pixmap = QPixmap('./src/servee_gui/manager_gui/maps/map_final_3_scaled.pgm')
        # pixmap = pixmap.scaled(self.label_map.size(), Qt.KeepAspectRatio, Qt.FastTransformation)
        self.label_map.setPixmap(pixmap)
        self.label_map_3.setPixmap(pixmap)

        # 로봇 현황 시작
        header = self.table_robots_status_2.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeToContents)
        robots = self.dbm.get_robots()
        for i, robot_id in enumerate(robots):
            id_1 = QTableWidgetItem(f"Servee_Robot_{robot_id[0]}")
            id_1.setTextAlignment(Qt.AlignCenter)
            self.table_robots_status_2.setItem(i, 0, id_1)
    
        # 일정 주기마다 지도 업데이트
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_map)
        self.timer.start(100)  # 100ms마다 업데이트

    def __del__(self):
        self.node.destroy_node()
        rclpy.shutdown()

    
    def update_map(self):
        rclpy.spin_once(self.node, timeout_sec=0.01)  # 제일 최근값 가져오는지 확인 필요. 
        
        # 맵 상의 로봇 위치와 경로 업데이트/그리기
        self.robots_pose = (self.node.robots_pose[0], self.node.robots_pose[1], self.node.robots_pose[2])
        self.robots_path = (self.node.robots_path[0], self.node.robots_path[1], self.node.robots_path[2])

        # 로봇 현황 업데이트
        for i in range(3):
            item_1 = QTableWidgetItem(self.robots_pose[i])
            item_2 = QTableWidgetItem(self.robots_path[i])
            item_1.setTextAlignment(Qt.AlignCenter)
            item_2.setTextAlignment(Qt.AlignCenter)
            self.table_robots_status_2.setItem(i, 1, item_1)
            self.table_robots_status_2.setItem(i, 2, item_2)
    
    def tab_clicked_callback(self, index):
        '''
        1. select * from Stores; 해서 매점 개수만큼 row 지정. 
        2. name column을 업체명에 등록. 
        3. Log table에서 store_id를 이용하여 금일 주문 건수를 등록. 
        4. 
        '''
        if index == 1:
            stores = self.dbm.get_stores()
            store_order_recap = self.dbm.total_orders_today()  # store_order_recap: [(store_id: int, order_count: int), ...]
            store_earning_recap = self.dbm.total_earning_today()

            self.table_stores_status_2.setRowCount(len(stores))
            header = self.table_stores_status_2.horizontalHeader()
            header.setSectionResizeMode(QHeaderView.ResizeToContents)

            for i, store_name in enumerate(stores):
                name = QTableWidgetItem(store_name[0])
                _orders = QTableWidgetItem('0')
                _earning = QTableWidgetItem('0')
                name.setTextAlignment(Qt.AlignCenter)
                _orders.setTextAlignment(Qt.AlignCenter)
                _earning.setTextAlignment(Qt.AlignCenter)
                self.table_stores_status_2.setItem(i, 0, name)
                self.table_stores_status_2.setItem(i, 1, _orders)
                self.table_stores_status_2.setItem(i, 2, _earning)
            
            for i, orders in enumerate(store_order_recap):
                store_id, count = orders[0], orders[1]
                order_count = QTableWidgetItem(str(count))
                order_count.setTextAlignment(Qt.AlignCenter)
                self.table_stores_status_2.setItem(store_id-1, 1, order_count)
            
            for i, earnings in enumerate(store_earning_recap):
                store_id, earning = earnings[0], earnings[1]
                store_earning = QTableWidgetItem(str(earning))
                store_earning.setTextAlignment(Qt.AlignCenter)
                self.table_stores_status_2.setItem(store_id-1, 2, store_earning)

    def table_robot_dclicked(self, row, column):
        print("orbot table double clicked")
        dialog = RobotStatus()
        dialog.exec_()

    def table_store_dclicked(self, row, column):
        print("store table double clicked")
        dialog = StoreStatus()

        date = datetime.datetime.now().strftime("%Y-%m-%d")
        dialog.label_datetime.setText(date)

        name = self.table_stores_status_2.item(row, 0)  # 여기서 아이템 오브젝트를 리턴한다. 값을 가져오게 고쳐라. 
        print("name:", name)
        store_id = self.dbm.get_store_id(name)[0]
        print("store_id:", store_id)

        order_details = self.dbm.order_status(store_id)
        for i, order_detail in enumerate(order_details):
            print(order_details)

        dialog.exec_()




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