import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose
from servee_interfaces.msg import TaskGoalData
import threading
import queue
import time
import math
from datetime import datetime
import re
from .observer_publisher import Server, ServingInstance, RetrievalInstance, Command, \
    CreateInstanceCommand, UpdateStatusCommand, DeleteInstanceCommand, DBManager

class Robot:
    def __init__(self, robot_id, name, robot_type):
        self.robot_id = robot_id
        self.name = name
        self.robot_type = robot_type
        self.state = 'idle'
        self.prev_state = 'idle'
        self.assigned_task_id = None
        self.pose = None
        self.executed_tasks = 0

    def calculate_distance(self, location_coordinates):
        if self.pose is None:
            return float('inf')
        dx = self.pose.position.x - location_coordinates[0]
        dy = self.pose.position.y - location_coordinates[1]
        return (dx**2 + dy**2)**0.5

class RobotTask(Node):
    def __init__(self, server, db_manager):
        super().__init__('robot_task')
    
        self.server = server
        self.db_manager = db_manager

        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = MutuallyExclusiveCallbackGroup()
        self.group4 = MutuallyExclusiveCallbackGroup()
        self.group5 = MutuallyExclusiveCallbackGroup()
        self.group6 = MutuallyExclusiveCallbackGroup()

        # ? actual code
        # self.robots = {
        #     'robot1': Robot(1, 'robot1', 'Server'),
        #     'robot2': Robot(2, 'robot2', 'Server'),
        #     'robot3': Robot(3, 'robot3', 'Retriever')}
   
        # self.publishers = self.init_publishers()
        # self.init_subscriptions()

        self.serving_task_queue = self.server.serving_task_queue
        self.retrieving_task_queue = self.server.retrieving_task_queue

        # ! code for communication test
        self.robots = {'robot': Robot(1, 'robot', 'Server')}
        # self.robots = {'robot': Robot('robot', 'Retriever')}

        self.task_publishers = self.init_publishers()
        self.create_subscription(Pose, 'pose', self.robot_pose_callback('robot'), 10, callback_group=self.group1)
        self.create_subscription(String, 'state', self.robot_state_callback('robot'), 10, callback_group=self.group4)
        
        time.sleep(3)
        self.create_timer(0.5, self.assign_tasks)

    def init_publishers(self):
        publishers = { 
            ## ? actual code
            # 'robot1': self.create_publisher(TaskGoalData, '/robot1/servee/task_goal_data', 10),
            # 'robot2': self.create_publisher(TaskGoalData, '/robot2/servee/task_goal_data', 10),
            # 'robot3': self.create_publisher(TaskGoalData, '/robot3/servee/task_goal_data', 10)

            # ! code for communication test
            'robot': self.create_publisher(TaskGoalData, '/servee/task_goal_data', 10)
        }
        return publishers

    def init_subscriptions(self): 
        self.create_subscription(Pose, '/robot1/pose', self.robot_pose_callback('robot1'), 10, callback_group=self.group1)
        self.create_subscription(Pose, '/robot2/pose', self.robot_pose_callback('robot2'), 10, callback_group=self.group2)
        # self.create_subscription(TransformStamped, '/robot2/pose', self.robot_pose_callback('robot2'), 10, callback_group=self.group3)
        self.create_subscription(String, '/robot1/state', self.robot_state_callback('robot1'), 10, callback_group=self.group4)
        self.create_subscription(String, '/robot2/state', self.robot_state_callback('robot2'), 10, callback_group=self.group5)
        # self.create_subscription(String, '/robot2/state', self.robot_state_callback('robot2'), 10, callback_group=self.group6)

    def robot_pose_callback(self, robot_name):
        def callback(msg):
            self.robots[robot_name].pose = msg
        return callback

    def robot_state_callback(self, robot_name):
        # * robot state type : "idle", "running1", "standby1", "running2", "standy2", "returning_home", "low_battery" 
        def callback(msg):
            robot = self.robots[robot_name]
            robot.prev_state = robot.state
            robot.state = msg.data
            if robot.assigned_task_id:
                if robot.prev_state in ['idle','returning_home','low_battery'] and robot.state == 'running1':
                    if robot.robot_type == 'Server':
                        command = UpdateStatusCommand(call_type='SE', order_id=robot.assigned_task_id, 
                                                    new_status='waiting_handover')
                    elif robot.robot_type == 'Retriever':
                        command = UpdateStatusCommand(call_type='RV', table_id=robot.assigned_task_id, 
                                                    new_status='start_retrieving')
                    command.execute(self.server)
                    time.sleep(1)
                elif robot.prev_state=='running1' and robot.state == 'standby1':
                    if robot.robot_type == 'Server':
                        command = UpdateStatusCommand(call_type='SE', order_id=robot.assigned_task_id, 
                                                    new_status='waiting_handover')
                    elif robot.robot_type == 'Retriever':
                        command = UpdateStatusCommand(call_type='RV', table_id=robot.assigned_task_id, 
                                                    new_status='waiting_handover')
                    command.execute(self.server)
                    time.sleep(1)
                elif robot.prev_state=='standby1' and robot.state == 'running2':
                    if robot.robot_type == 'Server':
                        command = UpdateStatusCommand(call_type='SE', order_id=robot.assigned_task_id, 
                                                    new_status='serving')
                    elif robot.robot_type == 'Retriever':
                        command = UpdateStatusCommand(call_type='RV', table_id=robot.assigned_task_id, 
                                                    new_status='retrieving')
                    command.execute(self.server)
                    time.sleep(1)
                elif robot.prev_state=='running2' and robot.state == 'standby2':
                    if robot.robot_type == 'Server':
                        command = UpdateStatusCommand(call_type='SE', order_id=robot.assigned_task_id, 
                                                    new_status='done_serving')
                    elif robot.robot_type == 'Retriever':
                        command = UpdateStatusCommand(call_type='RV', table_id=robot.assigned_task_id, 
                                                    new_status='done_retrieving')
                    command.execute(self.server)
                    time.sleep(1)
                elif robot.prev_state == 'standby2' and robot.state in ['idle', 'low_battery'] :
                    if robot.robot_type == 'Server':
                        order_id=robot.assigned_task_id
                        self.db_manager.insert_log(robot_id=robot.robot_id, 
                                                   table_id=self.server.SE_instances[order_id].table_id, 
                                                   store_id=self.server.SE_instances[order_id].store_id,
                                                   call_type='서빙',
                                                   call_time=self.server.SE_instances[order_id].call_time,
                                                   task_start_time=self.server.SE_instances[order_id].task_start_time,
                                                   task_end_time=datetime.now()
                                                   )
                        print("log inserted to DB")
                        
                        command = DeleteInstanceCommand(call_type='SE', order_id=order_id)
                    elif robot.robot_type == 'Retriever':
                        table_id=robot.assigned_task_id
                        self.db_manager.insert_log(robot_id=robot.robot_id, 
                                                   table_id=self.server.RV_instances[order_id].table_id, 
                                                   store_id=self.server.RV_instnaces[order_id].store_id,
                                                   call_type='회수',
                                                   call_time=self.server.RV_instnaces[order_id].call_time,
                                                   task_start_time=self.server.RV_instnaces[order_id].task_start_time,
                                                   task_end_time=datetime.now()
                                                   )
                        print("log inserted to DB")
                        
                        command = DeleteInstanceCommand(call_type='RV', table_id=table_id)
                    command.execute(self.server)
                    robot.executed_tasks += 1
                    print(robot.name, "finished task ", robot.assigned_task_id)
                    print(robot.name, "has executed ", robot.executed_tasks, "tasks so far!")
                    
                    robot.assigned_task_id = None
                    time.sleep(1)

        return callback

    def location_parser(self, location_data_string):
        # * data_string = "x:1.2, y:3.4, z:5.6, qx:0.1, qy:0.2, qz:0.3, qw:0.4"
        try:
            pattern = r"x:(-?\d+\.?\d*), y:(-?\d+\.?\d*), z:(-?\d+\.?\d*), qx:(-?\d+\.?\d*), qy:(-?\d+\.?\d*), qz:(-?\d+\.?\d*), qw:(-?\d+\.?\d*)"
            match = re.search(pattern, location_data_string)
            location_goalpose = [float(match.group(1)), float(match.group(2)), float(match.group(3)),
                                 float(match.group(4)), float(match.group(5)),
                                 float(match.group(6)), float(match.group(7))]
        except (RuntimeError, TypeError, NameError):
            print("error while parsing")
        
        return location_goalpose if location_goalpose else None
        
    def assign_tasks(self):
        # Check for 'Server' robots and assign serving tasks if available
        available_serverbots = [r for r in self.robots.values() if r.robot_type == 'Server' and r.state in ['idle', 'returning_home']]
        available_retrieverbots = [r for r in self.robots.values() if r.robot_type == 'Retriever' and r.state in ['idle', 'returning_home']]
        
        if self.serving_task_queue and available_serverbots:
            order = self.serving_task_queue.get()  # Get the position of the first task
            print("Assigning order ", order.order_id)
   
            # # ! code for communication test
            # store_data_string = "x:1.2, y:3.4, z:5.6, qx:0.1, qy:0.2, qz:0.3, qw:0.4"
            # table_data_string = "x:9.2, y:9.4, z:9.6, qx:9.1, qy:9.2, qz:9.3, qw:9.4"

            # ? actual code
            store_data_string = order.store_location
            table_data_string = order.table_location
            store_arucomarker_id = order.store_arucomarker_id
            table_arucomarker_id = order.table_arucomarker_id

            store_goalpose = self.location_parser(store_data_string)
            table_goalpose = self.location_parser(table_data_string)

            print("store data string : ", store_data_string)
            print("store goalpose : ", store_goalpose)
            print("store arucomarker id : ", store_arucomarker_id)
            print("table arucomarker id : ", table_arucomarker_id)

            if store_goalpose and table_goalpose:
                optimal_serverbot = min(available_serverbots, key=lambda r: (r.executed_tasks, 
                                                                             r.calculate_distance(store_goalpose[:2])))
                print("Task assigned to : ", optimal_serverbot.name)
                self.assign_task_to_robot(optimal_serverbot, order.order_id, store_goalpose, 
                                          table_goalpose, store_arucomarker_id, table_arucomarker_id)

        if self.retrieving_task_queue and available_retrieverbots:
            retrieval = self.retrieving_task_queue.get()  # Get the position of the first task
            print("Assigning retrieval ", retrieval.table_id)

            # # ! code for communication test
            # store_data_string = "x:1.2, y:3.4, z:5.6, qx:0.1, qy:0.2, qz:0.3, qw:0.4"
            # table_data_string = "x:9.2, y:9.4, z:9.6, qx:9.1, qy:9.2, qz:9.3, qw:9.4"

            # ? actual code
            store_data_string = retrieval.store_location
            table_data_string = retrieval.table_location
            store_arucomarker_id = retrieval.store_arucomarker_id
            table_arucomarker_id = retrieval.table_arucomarker_id

            store_goalpose = self.location_parser(store_data_string)
            table_goalpose = self.location_parser(table_data_string)

            if store_goalpose and table_goalpose:
                optimal_retrieverbot = min(available_retrieverbots, key=lambda r: r.calculate_distance(table_goalpose[:2]))
                print("Task assigned to : ", optimal_retrieverbot.name)
                self.assign_task_to_robot(optimal_retrieverbot, retrieval.table_id, table_goalpose, 
                                          store_goalpose, table_arucomarker_id, store_arucomarker_id)

    def assign_task_to_robot(self, robot, task_id, first_goalpose, second_goalpose, 
                             first_arucomarker_id, second_arucomarker_id):
        
        robot.assigned_task_id=task_id

        task_goal_poses = TaskGoalData()
        pose_array = PoseArray()
        
        pose1 = Pose() # 첫 번째 좌표 설정
        pose1.position.x = first_goalpose[0]
        pose1.position.y = first_goalpose[1]
        pose1.position.z = first_goalpose[2]
        pose1.orientation.x = first_goalpose[3]
        pose1.orientation.y = first_goalpose[4]
        pose1.orientation.z = first_goalpose[5]
        pose1.orientation.w = first_goalpose[6]

        pose2 = Pose()  # 두 번째 좌표 설정
        pose2.position.x = second_goalpose[0]
        pose2.position.y = second_goalpose[1]
        pose2.position.z = second_goalpose[2]
        pose2.orientation.x = second_goalpose[3]
        pose2.orientation.y = second_goalpose[4]
        pose2.orientation.z = second_goalpose[5]
        pose2.orientation.w = second_goalpose[6]

        # PoseArray에 좌표 추가
        pose_array.poses.append(pose1)
        pose_array.poses.append(pose2)

        ## ? actual code 
        aruco_ids = [first_arucomarker_id, second_arucomarker_id]

        # ! test code for communication test
        # aruco_ids = [10, 11]

        # TaskGoalData에 PoseArray 할당
        task_goal_poses.goal_poses = pose_array
        task_goal_poses.aruco_id = aruco_ids

        # 퍼블리시
        while True:
            print(f"Assigning task {task_id} to {robot.name}")
            self.task_publishers[robot.name].publish(task_goal_poses)
            time.sleep(0.2)
            if robot.assigned_task_id:
                break

        print(f"Assigned successfully!")

        robot.task_id = task_id
        try:
            if robot.robot_type == 'Server':
                self.server.SE_instances[task_id].task_start_time = datetime.now()
            elif robot.robot_type == 'Retriever':
                self.server.RV_instances[task_id].task_start_time = datetime.now()
            else: 
                print("invalid robot type")
        except (RuntimeError, TypeError, NameError):
            print("error while updating task start time")
        
    def run(self):
        rclpy.spin(self)

    def stop(self):
        self.server.stop()
        print("server stopped")
        self.destroy_node()
        print("robot task node stopped")

def main(args=None):

    host = "localhost"
    port = 9998

    db_manager = DBManager(
        host="192.168.0.130",
        port=3306,
        user="kjc",
        password="1234",
        database="SERVEE_DB"
    )

    server = Server(host, port, db_manager)
    server_thread = threading.Thread(target=server.start)
    server_thread.daemon = True  # 메인 스레드 종료 시 함께 종료
    server_thread.start()

    rclpy.init(args=args)
    robot_task = RobotTask(server, db_manager)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(robot_task)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        executor.shutdown()
        robot_task.stop()
        server.stop()
        db_manager.close()
        print("All tasks stopped")

if __name__ == '__main__':
    main()
