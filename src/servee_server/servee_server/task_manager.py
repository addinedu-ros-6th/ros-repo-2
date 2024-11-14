import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose
from servee_interfaces.msg import TaskGoalPoses, TaskGoalPose

import threading
import queue
import time
import math
from datetime import datetime

import re

from .observer_publisher import Server, ServingInstance, RetrievalInstance, Command, \
    CreateInstanceCommand, UpdateStatusCommand, DeleteInstanceCommand

class Robot:
    def __init__(self, name, robot_type):
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
    def __init__(self, server):
        super().__init__('robot_task')

        self.server = server

        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = MutuallyExclusiveCallbackGroup()
        self.group4 = MutuallyExclusiveCallbackGroup()
        self.group5 = MutuallyExclusiveCallbackGroup()
        self.group6 = MutuallyExclusiveCallbackGroup()

        # ? actual code
        # self.robots = {
        #     'robot1': Robot('robot1', 'Server'),
        #     'robot2': Robot('robot2', 'Server'),
        #     'robot3': Robot('robot3', 'Retriever')}
   
        # self.publishers = self.init_publishers()
        # self.init_subscriptions()

        self.serving_task_queue = self.server.serving_task_queue
        self.retrieving_task_queue = self.server.retrieving_task_queue

        # ! code for communication test
        self.robots = {'robot': Robot('robot', 'Server')}
        # self.robots = {'robot': Robot('robot', 'Retriever')}

        self.task_publishers = self.init_publishers()
        self.create_subscription(Pose, 'pose', self.robot_pose_callback('robot'), 10, callback_group=self.group1)
        self.create_subscription(String, 'state', self.robot_state_callback('robot'), 10, callback_group=self.group4)
        
        time.sleep(3)
        self.create_timer(0.5, self.assign_tasks)

    def init_publishers(self):
        publishers = { 
            ## ? actual code
            # 'robot1': self.create_publisher(TaskGoalPoses, '/robot1/servee/task_goal_poses', 10),
            # 'robot2': self.create_publisher(TaskGoalPoses, '/robot2/servee/task_goal_poses', 10),
            # 'robot3': self.create_publisher(TaskGoalPoses, '/robot3/servee/task_goal_poses', 10)

            # ! code for communication test
            'robot': self.create_publisher(TaskGoalPoses, '/servee/task_goal_poses', 10)
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
            elif robot.prev_state == 'standby2' and robot.state in ['idle', 'low_battery'] and robot.assigned_task_id:
                if robot.robot_type == 'Server':
                    command = DeleteInstanceCommand(call_type='SE', order_id=robot.assigned_task_id)
                elif robot.robot_type == 'Retriever':
                    command = DeleteInstanceCommand(call_type='RV', table_id=robot.assigned_task_id)
                command.execute(self.server)
                time.sleep(1)
                print(robot.name, " finished task ", robot.assigned_task_id)

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
   
            # ! code for communication test
            store_data_string = "x:1.2, y:3.4, z:5.6, qx:0.1, qy:0.2, qz:0.3, qw:0.4"
            table_data_string = "x:9.2, y:9.4, z:9.6, qx:9.1, qy:9.2, qz:9.3, qw:9.4"

            # ? actual code
            # store_data_string = order.store_loaction
            # table_data_string = order.table_loaction
            store_goalpose = self.location_parser(store_data_string)
            table_goalpose = self.location_parser(table_data_string)

            if store_goalpose and table_goalpose:
                closest_serverbot = min(available_serverbots, key=lambda r: r.calculate_distance(store_goalpose[:2]))
                print("Task assigned to : ", closest_serverbot.name)
                self.assign_task_to_robot(closest_serverbot, order.order_id, store_goalpose, table_goalpose)

        if self.retrieving_task_queue and available_retrieverbots:
            retrieval = self.retrieving_task_queue.get()  # Get the position of the first task
            print("Assigning retrieval ", retrieval.table_id)

            # ! code for communication test
            store_data_string = "x:1.2, y:3.4, z:5.6, qx:0.1, qy:0.2, qz:0.3, qw:0.4"
            table_data_string = "x:9.2, y:9.4, z:9.6, qx:9.1, qy:9.2, qz:9.3, qw:9.4"

            # ? actual code
            # store_data_string = retrieval.store_loaction
            # table_data_string = retrieval.table_loaction

            store_goalpose = self.location_parser(store_data_string)
            table_goalpose = self.location_parser(table_data_string)

            if store_goalpose and table_goalpose:
                closest_retrieverbot = min(available_retrieverbots, key=lambda r: r.calculate_distance(table_goalpose[:2]))
                print("Task assigned to : ", closest_retrieverbot.name)
                self.assign_task_to_robot(closest_retrieverbot, retrieval.table_id, table_goalpose, store_goalpose)

    def assign_task_to_robot(self, robot, task_id, first_goalpose, second_goalpose):
        
        robot.assigned_task_id=task_id

        task_goal_poses = TaskGoalPoses()
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

        # TaskGoalPoses에 PoseArray 할당
        task_goal_poses.goal_poses = pose_array

        # 퍼블리시
        while True:
            print(f"Assigning task {task_id} to {robot.name}")
            self.task_publishers[robot.name].publish(task_goal_poses)
            time.sleep(0.2)
            if robot.state == 'running1':
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

    server = Server(host, port)
    server_thread = threading.Thread(target=server.start)
    server_thread.daemon = True  # 메인 스레드 종료 시 함께 종료
    server_thread.start()

    rclpy.init(args=args)
    robot_task = RobotTask(server)
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
        print("All tasks stopped")

if __name__ == '__main__':
    main()
