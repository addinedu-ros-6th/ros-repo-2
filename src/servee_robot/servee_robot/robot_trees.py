import rclpy
import py_trees
import py_trees_ros

import py_trees.console as console
from py_trees_ros import battery, utilities
from py_trees.blackboard import Blackboard
from py_trees.common import ParallelPolicy
from py_trees.composites import Parallel, Sequence, Selector
from py_trees import decorators
from py_trees import logging as log_tree

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from servee_interfaces.msg import TaskGoalPose, ResPath
from servee_robot.servee_behaviors import led_flasher, movement, request_path, response_path, get_curr_pose, receive_goal
from geometry_msgs.msg import PoseWithCovarianceStamped



def receive_goal_node():
    """
    목적지를 전송받는다.
    """
    # goal2bb = py_trees_ros.subscribers.ToBlackboard(
    #     name="Receive Goal",
    #     topic_name= "servee/task_goal_pose",
    #     topic_type= TaskGoalPose,
    #     blackboard_variables= "goal_pose",
    #     qos_profile=py_trees_ros.utilities.qos_profile_latched(),
    # )
    
    receive_goal_behavior = receive_goal.ReceiveGoal("receive_goal_node")
    
    def is_robot_idle_or_home(blackboard):
        return blackboard.robot_state in ['idle', 'home']
    
    goal_node = decorators.EternalGuard(
        name="Can Task?",
        condition=is_robot_idle_or_home,
        blackboard_keys={"robot_state"},
        child= receive_goal_behavior
    )
    
    return goal_node

def create_path():
    """
    이동 경로를 받아온다. 
    
    get_path (Sequence - True): 
        - receive_goal
        - path
        - test_timer
    """
    path = Parallel("Create Path", policy=ParallelPolicy.SuccessOnAll())
    
    goal = receive_goal_node()
    req_path = request_path.RequestPath("reqeuest_path_node")
    res_path = response_path.ResponsePath("response_path_node")    

    # timer_node = py_trees.timers.Timer(name="time", duration=10.0)

    path.add_children([goal, req_path, res_path])
    return path

def move_to_goal():
    """
    목적지로 이동하는 트리
    
    Move (Parallel - SuccessOnOne):
        - 
    """
    move = Parallel("Movement", policy=ParallelPolicy.SuccessOnOne())
    move_robot = movement.Movement("robot_movement_node")
    
    # 여기서 맵에 끼였을때라던지 
    # 교차로 입장에 대한 처리가 추가될 예정이다.
    move.add_children([move_robot])
    
    return move


def battery_low_alarm():
    """
    배터리 상태를 파악하고 부족하면 LED로 알린다.
    """
    better_low_alarm = led_flasher.LedFlasher("better_low_alarm")  # 배터리가 부족할 때 깜박이는 LED

    # 조건 함수 정의: 배터리가 부족하면 False를 반환하도록 설정
    def check_battery_low_on_blackboard(blackboard: Blackboard):
        return blackboard.battery_low_warning

    battery_emergency = decorators.EternalGuard(
        name="Battery Low?",
        condition=check_battery_low_on_blackboard,
        blackboard_keys={"battery_low_warning"},
        child=better_low_alarm
    )
    
    return battery_emergency
    
    

def begin_tasks():
    """
    tasks ( Selector - False)
    - 건전지 여부 체크 (get_path)
    - 경로 요청 (get_path)
    """
    tasks = Selector("Tasks", memory=False)
    
    path = create_path() # 경로를 요청한다. 
    
    move = move_to_goal() # 로봇을 이동시킨다.
    battery_alarm = battery_low_alarm()

    tasks.add_children([battery_alarm, path, move])
    return tasks


def receive_topic2bb():   
    """
    토픽으로 받아오는 데이터를 Blackboard에 저장한다.
    
    저장하는 데이터: 
        battery, curr_pose
    
    topic2bb (Parallel - SuccessOnAll)
        - battery2bb
        - curr_pose2bb
    """ 
    topic2bb = Parallel(
        name="Topic2bb",
        policy=ParallelPolicy.SuccessOnAll(
            synchronise=False)
    )
    
    # 배터리 상태
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2bb",
        topic_name="servee/battery_state",
        qos_profile=utilities.qos_profile_unlatched(),
        threshold=30.0
    )
    
    cur_pose = get_curr_pose.GetCurrPose("get_curr_pose_node")
    
    # 레이저 스캔
    laser_scan2bb = py_trees_ros.subscribers.ToBlackboard(
        name="Get Laser Scan",
        topic_name="/scan",
        topic_type=LaserScan,
        blackboard_variables="laser_scan",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
    )
    # laser_scan2bb
    topic2bb.add_children([battery2bb, cur_pose])
    return topic2bb


def create_root_tree():
    """
    최상위 tree
    root (Parallel - SuccessOnAll)
        - tasks
        - topic2bb
    """
    root = Parallel(
        name= "Servee Main",
        policy= ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )
    
    topic2bb = receive_topic2bb()
    tasks = begin_tasks()
    root.add_children([topic2bb, tasks])
    return root
    
def main():
    rclpy.init(args=None)
    try:
        root = create_root_tree()
        tree = py_trees_ros.trees.BehaviourTree(
            root= root,
            unicode_tree_debug=True,
        )
        
        # 행동 트리를 설정하고 틱 반복을 시작
        tree.setup(timeout=15)
        # log_tree.level = log_tree.Level.DEBUG
        tree.tick_tock(period_ms=200.0)
        
        # ROS2 노드를 스핀하고 행동 트리를 실행
        rclpy.spin(tree.node)
        
        
    except KeyboardInterrupt:
        pass
    
    finally:
        tree.shutdown()
        rclpy.try_shutdown()