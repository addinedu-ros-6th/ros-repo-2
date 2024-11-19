import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from servee_interfaces.msg import TaskGoalPose, TaskGoalData

import threading
import queue
import time
import math
import datetime


## 타이머 버전

# 인스턴스 클래스들
# 주문 인스턴스는 order_id로 접근하고 수거 인스턴스는 store_id로 접근
class ServingInstance:
    def __init__(self, order_id, store_id, table_id, call_time):
        self.order_id = order_id
        self.store_id = store_id
        self.table_id = table_id
        self.call_time = call_time
        self.status = "1"  # 초기 상태
        self.table_location = "0-0-0/0-0-0-0"  # table_id 로 위치 접근, 추후 DB 연동
        self.store_location = "0-0-0/0-0-0-0"  # store_id 로 위치 접근, 추후 DB 연동

class RetrievalInstance:
    def __init__(self, store_id, table_id, call_time):
        self.store_id = store_id
        self.table_id = table_id
        self.call_time = call_time
        self.status = "1"  # 초기 상태
        self.table_location = "0-0-0/0-0-0-0"  # table_id 로 위치 접근, 추후 DB 연동
        self.retrieval_location = "0-0-0/0-0-0-0"  # store_id 로 위치 접근, 추후 DB 연동

serving_state = ['charging', 
                 'recieving', 
                 'waiting_food', 
                 'delivering', 
                 'waiting_customer', 
                 'returning_home']

retrieving_state = ['charging', 
                    'retrieving', 
                    'waiting_customer', 
                    'returning_plates', 
                    'waiting_plates', 
                    'returning_home']

order_details = queue.Queue()  # order_id
retrieve_details = queue.Queue()  # table_id

'''
TODO: 
- DB에 올릴 데이터도 종합 하기
    - robot_id, task_start_time, task_end_time -> OK
- 일 끝나고 집가는 좌표도 추가하기 <- 로봇 안에 있음?
- 로봇한테 좌표는 한번에 두개 다 보내주면 된다 -> OK
- 통신 불안정 문제로 좌표 받았다는 응답 올때까지 계속 반복해서 보내라 -> OK
- 
'''

class RobotTask(Node):
    def __init__(self):
        super().__init__('robot_task')
        self.get_logger().info("robot_task initialized")

        # 로봇 1, 2가 서빙로봇이라고 가정
        # 여기 나중에 지울거임. 정리좀 해라. 
        self.robots_asigned_status = {'robot1': {'state': 'charging', 
                                                 'dest_1': (float, float, float, float), 
                                                 'dest_2': (float, float, float, float), 
                                                 'task_start_time': datetime.datetime.now(), 
                                                 'task_end_time': datetime.datetime.now()}, 
                                      'robot2': {'state': 'charging', 
                                                 'dest_1': (float, float, float, float), 
                                                 'dest_2': (float, float, float, float), 
                                                 'task_start_time': datetime.datetime.now(), 
                                                 'task_end_time': datetime.datetime.now()}, 
                                      'robot3': {'state': 'charging', 
                                                 'dest_1': (float, float, float, float), 
                                                 'dest_2': (float, float, float, float), 
                                                 'task_start_time': datetime.datetime.now(), 
                                                 'task_end_time': datetime.datetime.now()}}

        # 구독이랑 아래 비동기문이랑 충돌 안나는지 확인 필요
        self.robot1_pose = TransformStamped()
        self.robot1_pose_sub = self.create_subscription(TransformStamped, '/robot1/pose', self.robot1_pose_callback, 10)  # 토픽이름 나중에 수정
        self.robot2_pose = TransformStamped()
        self.robot2_pose_sub = self.create_subscription(TransformStamped, '/robot2/pose', self.robot2_pose_callback, 10)
        # self.robot3_pose = TransformStamped()
        # self.robot3_pose_sub = self.create_subscription(TransformStamped, '/robot3/pose', self.robot3_pose_callback, 10)

        self.robot1_state = String()
        self.robot1_state_sub = self.create_subscription(String, '/robot1/state', self.robot1_state_callback, 10)
        self.robot2_state = String()
        self.robot2_state_sub = self.create_subscription(String, '/robot2/state', self.robot2_state_callback, 10)
        # self.robot3_state = String()
        # self.robot3_state_sub = self.create_subscription(String, '/robot3/state', self.robot3_state_callback, 10)

        # 작업 시작했는지 확인하는 토픽
        # 이거 이렇게 토픽으로 받지 말고 robot_state_prev로 처리해도 된다
        self.robot1_isStarted = String()
        self.robot1_start_subscriber = self.create_subscription(String, '/robot1/servee/started', self.robot1_start_callback, 10)  # 토픽 이름이랑 타입 나중에 수정
        self.robot2_isStarted = String()
        self.robot2_start_subscriber = self.create_subscription(String, '/robot2/servee/started', self.robot2_start_callback, 10)
        # self.robot3_isStarted = String()
        # self.robot3_start_subscriber = self.create_subscription(String, '/robot3/servee/started', self.robot3_start_callback, 10)

        self.robot1_destination = TaskGoalPose()
        self.robot1_dest_publisher = self.create_publisher(TaskGoalData, '/robot1/servee/task_data', 10)
        self.robot2_destination = TaskGoalPose()
        self.robot2_dest_publisher = self.create_publisher(TaskGoalData, '/robot2/servee/task_data', 10)
        # self.robot3_destination = TaskGoalPose()
        # self.robot3_dest_publisher = self.create_publisher(TaskGoalPose, '/robot3/servee/task_goal_pose', 10)

        self.serving_publisher_timer = self.create_timer(0.5, self.serving_publisher_callback)
        self.retrieving_publisher_timer = self.create_timer(0.5, self.retrieving_publisher_callback)
        
        self.robots_dest_publishers = {'robot1': self.robot1_dest_publisher, 
                                       'robot2': self.robot2_dest_publisher, 
                                       'robot3': self.robot3_dest_publisher}
        
        self.robots_isStarted = {'robot1': self.robot1_isStarted, 
                                 'robot2': self.robot2_isStarted, 
                                 'robot3': self.robot3_isStarted}

        self.pending_order = None
        self.pending_retrieval = None

    def run(self):
        # 콜백함수들 실행하는데 오류나면 이거로 한번 해보기
        # 아니면 비동기로 실행
        # executor = MultiThreadedExecutor()
        # executor.add_node(self)
        # executor.spin()

        rclpy.spin(self)

    def stop(self):
        # executor.shutdown()
        rclpy.shutdown()
        self.get_logger().info("robot_task stopped")
    

    def robot1_pose_callback(self, msg: TransformStamped):
        self.robot1_pose = msg
    
    def robot2_pose_callback(self, msg: TransformStamped):
        self.robot2_pose = msg
    
    def robot3_pose_callback(self, msg: TransformStamped):
        self.robot3_pose = msg
    
    def robot1_state_callback(self, msg: String):
        self.robot1_state_prev = self.robots_asigned_status['robot1']['state']
        self.robots_asigned_status['robot1']['state'] = msg

        if (self.robot1_state_prev == 'waiting_customer') and (self.robots_asigned_status['robot1']['state'] == 'returning_home'):
            self.robots_asigned_status['robot1']['task_end_time'] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            # 여기서 robot_id, task_start_time, task_end_time call_task로 보내라
    
    def robot2_state_callback(self, msg: String):
        self.robot2_state_prev = self.robots_asigned_status['robot2']['state']
        self.robots_asigned_status['robot1']['state'] = msg

        if (self.robot2_state_prev == 'waiting_customer') and (self.robots_asigned_status['robot2']['state'] == 'returning_home'):
            self.robots_asigned_status['robot2']['task_end_time'] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            # 여기서도 보내라
    
    def robot3_state_callback(self, msg: String):
        self.robot3_state_prev = self.robots_asigned_status['robot3']['state']
        self.robots_asigned_status['robot1']['state'] = msg

        if (self.robot3_state_prev == 'waiting_plates') and (self.robots_asigned_status['robot3']['state'] == 'returning_home'):
            self.robots_asigned_status['robot3']['task_end_time'] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            # 여기서도 보내라
    
    def robot1_start_callback(self, msg: String):
        self.robot1_isStarted = msg
    
    def robot2_start_callback(self, msg: String):
        self.robot2_isStarted = msg
    
    def robot3_start_callback(self, msg: String):
        self.robot3_isStarted = msg


    # 서빙 로봇 작업 할당
    def serving_publisher_callback(self):
        # 할당 대기중인 주문이 없다면 주문을 가져옴
        if self.pending_order is None:
            self.pending_order = order_details.get()
        
        # 주문을 로봇에 할당
        asigned_robot = self.serving_robot_asigner(self.pending_order)

        # 할당된 로봇이 있다면
        if asigned_robot is not None:
            # 주문 인스턴스에 할당 로봇 기록
            pass

            # 로봇의 목표지점들을 지정
            self.robots_asigned_status[asigned_robot]['dest_1'] = (self.pending_order.store_location.x, 
                                                                   self.pending_order.store_location.y, 
                                                                   self.pending_order.store_location.z, 
                                                                   self.pending_order.store_location.w)
            self.robots_asigned_status[asigned_robot]['dest_2'] = (self.pending_order.table_location.x, 
                                                                   self.pending_order.table_location.y, 
                                                                   self.pending_order.table_location.z, 
                                                                   self.pending_order.table_location.w)
            self.robots_asigned_status[asigned_robot]['task_start_time'] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            # 목표지점을 로봇에게 반복하여 발행
            count = 0
            while True:
                self.robots_dest_publishers[asigned_robot].publish([self.robots_asigned_status[asigned_robot]['dest_1'], 
                                                                    self.robots_asigned_status[asigned_robot]['dest_2']])
                
                # 로봇이 명령을 받았다고 답장하면 발행을 중단
                if self.robots_isStarted[asigned_robot] == 'started':
                    self.pending_order = None
                    break

                count += 1

                # 카운트가 10이 넘어가면 주문을 주문 큐에 다시 삽입하고 발행을 중단
                if count >= 10:
                    order_details.put(self.pending_order)
                    self.pending_order = None
                    break

                # time.sleep(0.2)

    # 회수 로봇 작업 할당
    def retrieving_publisher_callback(self):
        # 할당 대기중인 회수가 없다면 회수항목을 가져옴
        if self.pending_retrieval is None:
            self.pending_retrieval = retrieve_details.get()
        
        # 회수를 로봇에 할당
        asigned_robot = self.retrieving_robot_asigner(self.pending_retrieval)

        # 할당된 로봇이 있다면
        if asigned_robot is not None:
            # 회수 인스턴스에 할당 로봇 기록
            pass

            # 로봇의 목표지점들을 지정
            self.robots_asigned_status[asigned_robot]['dest_1'] = (self.pending_retrieval.table_location.x, 
                                                                   self.pending_retrieval.table_location.y, 
                                                                   self.pending_retrieval.table_location.z, 
                                                                   self.pending_retrieval.table_location.w)
            self.robots_asigned_status[asigned_robot]['dest_2'] = (self.pending_retrieval.retrieval_location.x, 
                                                                   self.pending_retrieval.retrieval_location.y, 
                                                                   self.pending_retrieval.retrieval_location.z, 
                                                                   self.pending_retrieval.retrieval_location.w)
            self.robots_asigned_status[asigned_robot]['task_start_time'] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            # 목표지점을 로봇에게 반복하여 발행
            count = 0
            while True:
                self.robots_dest_publishers[asigned_robot].publish([self.robots_asigned_status[asigned_robot]['dest_1'], 
                                                                    self.robots_asigned_status[asigned_robot]['dest_2']])
                
                # 로봇이 명령을 받았다고 답장하면 발행을 중단
                if self.robots_isStarted[asigned_robot] == 'started':
                    self.pending_retrieval = None
                    break

                count += 1

                # 카운트가 10이 넘어가면 주문을 주문 큐에 다시 삽입하고 발행을 중단
                if count >= 10:
                    retrieve_details.put(self.pending_retrieval)
                    self.pending_retrieval = None
                    break
                    
                # time.sleep(0.2)

    def serving_robot_asigner(self, order):
        if order is None:
            return None

        if (self.robots_asigned_status['robot1']['state'] in ['charging', 'returning_home']) and (self.robots_asigned_status['robot2']['state'] in ['charging', 'returning_home']):
            # 로봇 둘다 서빙 가능한 상태일때 더 가까이 있는 로봇에게 명령을 전달
            store_coord_x = order.store_coord_x
            store_coord_y = order.store_coord_y

            distance_1 = math.hypot((self.robot1_pose.transform.translation.x - store_coord_x), 
                                    (self.robot1_pose.transform.translation.y - store_coord_y))
            distance_2 = math.hypot((self.robot2_pose.transform.translation.x - store_coord_x), 
                                    (self.robot2_pose.transform.translation.y - store_coord_y))
            if distance_1 < distance_2:
                return 'robot1'
            elif distance_2 < distance_1:
                return 'robot2'
            else:
                # 혹시라도 로봇들이 같은 거리에 있을경우 루프를 한번 더 돌림
                self.get_logger().info("Robots are equally distanced")
                return None
            
        elif self.robots_asigned_status['robot1']['state'] in ['charging', 'returning_home']:
            # 로봇1이 가능한 상태이면 1에게 명령 전달
            return 'robot1'
        
        elif self.robots_asigned_status['robot2']['state'] in ['charging', 'returning_home']:
            # 로봇2가 가능한 상태이면 2에게 명령 전달
            return 'robot2'
        
        else:
            # 둘다 불가능한 상태이면 가능한 상태가 나올때까지 루프를 돌림
            return None
    
    def retrieving_robot_asigner(self, retrieval):
        if retrieval is None:
            return None
        
        if self.robots_asigned_status['robot3']['state'] in ['charging', 'returning_home']:
            return 'robot3'
        else:
            return None


class VendorTask:
    def __init__(self):
        print("vendor_task initialized")

    def run(self):
        while True:
            print("vendor_task running")
            time.sleep(1)

    def stop(self):
        print("vendor_task stopped")


def main():
    rclpy.init()

    robot_task = RobotTask()
    vendor_task = VendorTask()

    robot_thread = threading.Thread(target=robot_task.run, daemon=True)
    vendor_thread = threading.Thread(target=vendor_task.run, daemon=True)

    robot_thread.start()
    vendor_thread.start()

    # 메인 루프 대기
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down...")

    # 종료 처리
    robot_task.stop()
    vendor_task.stop()
    robot_thread.join()
    vendor_thread.join()
    print("All managers stopped")

if __name__ == '__main__':
    main()
