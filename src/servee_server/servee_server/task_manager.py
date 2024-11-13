import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from servee_interfaces.msg import TaskGoalPoses, TaskGoalPose

import threading
import queue
import time
import math
import datetime

import re

from observer_publisher import Server, ServingInstance, RetrievalInstance, Command, \
    CreateInstanceCommand, UpdateStatusCommand, DeleteInstanceCommand

class Robot:
    def __init__(self, name, robot_type):
        self.name = name
        self.robot_type = robot_type
        self.state = 'charging'
        self.assigned_task_id = None
        self.pose = None

    def calculate_distance(self, location_coordinates):

        if self.pose is None:
            return float('inf')
        dx = self.pose.transform.translation.x - location_coordinates[0]
        dy = self.pose.transform.translation.y - location_coordinates[1]
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

        self.robots = {
            'robot1': Robot('robot1', 'Server'),
            'robot2': Robot('robot2', 'Server'),
            'robot3': Robot('robot3', 'Retriever')}

        self.serving_task_queue = self.server.serving_task_queue
        self.retrieving_task_queue = self.server.retrieving_task_queue
        
        self.publishers = self.init_publishers()
        self.init_subscriptions()
        self.create_timers()

    def init_publishers(self):
        publishers = { 
            'robot1': self.create_publisher(TaskGoalPoses, '/robot1/servee/task_goal_poses', 10),
            'robot2': self.create_publisher(TaskGoalPoses, '/robot2/servee/task_goal_poses', 10),
            'robot3': self.create_publisher(TaskGoalPoses, '/robot3/servee/task_goal_poses', 10)
        }
        return publishers

    def init_subscriptions(self): 
        self.create_subscription(TransformStamped, '/robot1/pose', self.robot_pose_callback('robot1'), 10, callback_group=self.group1)
        self.create_subscription(TransformStamped, '/robot2/pose', self.robot_pose_callback('robot2'), 10, callback_group=self.group2)
        # self.create_subscription(TransformStamped, '/robot2/pose', self.robot_pose_callback('robot2'), 10, callback_group=self.group3)
        self.create_subscription(String, '/robot1/state', self.robot_state_callback('robot1'), 10, callback_group=self.group4)
        self.create_subscription(String, '/robot2/state', self.robot_state_callback('robot2'), 10, callback_group=self.group5)
        # self.create_subscription(String, '/robot2/state', self.robot_state_callback('robot2'), 10, callback_group=self.group6)

    def robot_pose_callback(self, robot_name):
        def callback(msg):
            self.robots[robot_name].pose = msg
        return callback

    def robot_state_callback(self, robot_name):
        def callback(msg):
            robot = self.robots[robot_name]
            prev_state = robot.state
            robot.state = msg.data
            if prev_state == 'waiting_customer' and robot.state == 'returning_home':
                robot.task_end_time = datetime.datetime.now()
        return callback

    def create_timers(self):
        self.create_timer(0.1, self.assign_tasks)

    def location_parser(self, location_data_string):
        # ? data_string = "x:1.2, y:3.4, z:5.6, qx:0.1, qy:0.2, qz:0.3, qw:0.4"
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
        available_serverbots = [r for r in self.robots.values() if r.robot_type == 'Server' and r.state in ['charging', 'returning_home']]
        if self.serving_task_queue and available_serverbots:
            order_id = self.serving_task_queue[0]  # Get the position of the first task

            store_data_string = self.server.SE_instances[order_id].store_loaction
            table_data_string = self.server.SE_instances[order_id].table_loaction
            store_goalpose = self.location_parser(store_data_string)
            table_goalpose = self.location_parser(table_data_string)

            if store_goalpose and table_goalpose:
                closest_serverbot = min(available_serverbots, key=lambda r: r.calculate_distance(table_goalpose[:2]))
                self.assign_task_to_robot(closest_serverbot, order_id, store_goalpose, table_goalpose)
                self.serving_task_queue.pop(0)

        available_retrieverbots = [r for r in self.robots.values() if r.robot_type == 'Server' and r.state in ['charging', 'returning_home']]
        if self.retrieving_task_queue and available_retrieverbots:
            table_id = self.retrieving_task_queue[0]  # Get the position of the first task

            store_data_string = self.server.RV_instances[table_id].store_loaction
            table_data_string = self.server.RV_instances[table_id].table_loaction
            store_goalpose = self.location_parser(store_data_string)
            table_goalpose = self.location_parser(table_data_string)

            if store_goalpose and table_goalpose:
                closest_retrieverbot = min(available_retrieverbots, key=lambda r: r.calculate_distance(table_goalpose[:2]))
                self.assign_task_to_robot(closest_retrieverbot, table_id, table_goalpose, store_goalpose)
                self.serving_task_queue.pop(0)

    def assign_task_to_robot(self, robot, task_id, first_goalpose, second_goalpose):
        # Publish the task to the closest robot's topic
        task_goal_poses = TaskGoalPoses()

        task_goal_poses.poses = [TaskGoalPose(x=first_goalpose[0], y=first_goalpose[1], 
                                                z=first_goalpose[2], qx=first_goalpose[3], 
                                                qy=first_goalpose[4], qz=first_goalpose[5], 
                                                qw=first_goalpose[6]),
                                TaskGoalPose(x=second_goalpose[0], y=second_goalpose[1], 
                                                z=second_goalpose[2], qx=second_goalpose[3], 
                                                qy=second_goalpose[4], qz=second_goalpose[5], 
                                                qw=second_goalpose[6])]

        self.publishers[robot.name].publish(task_goal_poses)
        robot.task_id = task_id

        if robot.robot_type == 'Server':
            self.server.SE_instances[task_id].task_start_time = datetime.datetime.now()
        elif robot.robot_type == 'Retriever':
            self.server.RV_instances[task_id].task_start_time = datetime.datetime.now()
        else: 
            print("invalid robot type")
        
        print(f"Assigned task {task_id} to {robot.name})")

    def run(self):
        rclpy.spin(self)

    def stop(self):
        self.destroy_node()

def main():
    host = "localhost"
    port = 9998

    server = Server(host, port)
    server.start()

    rclpy.init()
    robot_task = RobotTask(server=server)

    executor = MultiThreadedExecutor(num_threads==3)
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
