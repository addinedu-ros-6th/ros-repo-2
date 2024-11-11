import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import LaserScan

import numpy as np


class WallAvoidance(Node):
    def __init__(self):
        super().__init__('wall_avoidance_1')
        self.declare_parameter('avoidance_backward', -0.08)
        self.declare_parameter('avoidance_forward', 0.08)
        self.declare_parameter('avoidance_left', 0.08)
        self.declare_parameter('avoidance_right', -0.08)
        self.declare_parameter('wall_threshold', 0.4)  # 오른쪽 벽에 케이블 살짝 안닿을 정도로 위치했을때 라이다 min값 0.15정도로 나옴. 참고하여 조정. 
        self.avoidance_backward = self.get_parameter('avoidance_backward').value
        self.avoidance_forward = self.get_parameter('avoidance_forward').value
        self.avoidance_left = self.get_parameter('avoidance_left').value
        self.avoidance_right = self.get_parameter('avoidance_right').value
        self.wall_threshold = self.get_parameter('wall_threshold').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.robot1_pose = TransformStamped()
        self.robot1_pose_pub = self.create_publisher(TransformStamped, 'custom_pose', 10)
        self.robot1_pose_pub_timer = self.create_timer(0.1, self.pose_callback)

        self.scan = LaserScan()
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        self.wall_collision_timer = self.create_timer(0.1, self.check_for_wall_collision)

        self.vel_pub = self.create_publisher(
            Twist, 
            '/base_controller/cmd_vel_unstamped', 
            10)
    
    def __del__(self):  # 여기 없어도 되려나? 어쩌피 빠져나가면 주행으로 가잖나
        stop = Twist()
        stop.linear.x = 0
        stop.angular.z = 0
        self.vel_pub.publish(stop)

    
    def pose_callback(self):
        try:
            # 로봇 좌표
            self.robot1_pose = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"Could not transform: {str(e)}")
        
        self.robot1_pose_pub.publish(self.robot1_pose)
    
    def scan_callback(self, msg: LaserScan):
        self.scan_ranges = np.array(msg.ranges)

    def check_for_wall_collision(self):
        min_distances = [0, 0, 0, 0]
        # 4방향 라이다값 확인하기
        for i in range(4):
            # 라이다 한 조각의 거리 최소값 저장
            min_distances[i] = min(self.scan.ranges[int(32.5 * (2*i-1)) : int(32.5 * (2*i+1))])
        
        # 거리 최소값이 임계값보다 작으면 회피동작 수행
        if min(min_distances) < self.wall_threshold:
            self.avoid_wall(min_distances.index(min(min_distances)))  # 여기 테스트할때 인덱스 제대로 들어오는지 확인 주의
    
    def avoid_wall(self, wall_index):
        '''
        wall_index - 벽이 있는 위치
        0: 뒤
        1: 오른쪽
        2: 앞
        3: 왼쪽
        '''
        match wall_index:
            case 0:
                self.avoid_forward()
            case 1:
                self.avoid_left()
            case 2:
                self.avoid_backward()
            case 3:
                self.avoid_right()

    def avoid_forward(self):
        '''
        정면으로 회피
        '''
        twist = Twist()
        twist.linear.x = self.avoidance_forward
        self.vel_pub.publish(twist)
        self.get_logger().info("Avoiding wall on the behind")

    def avoid_right(self):
        '''
        오른쪽으로 회피
        '''
        twist = Twist()
        twist.angular.z = self.avoidance_right
        self.vel_pub.publish(twist)
        self.get_logger().info("Avoiding wall on the left")

    def avoid_backward(self):
        '''
        뒤로 회피
        '''
        twist = Twist()
        twist.linear.x = self.avoidance_backward
        self.vel_pub.publish(twist)
        self.get_logger().info("Avoiding wall on the forward")

    def avoid_left(self):
        '''
        왼쪽으로 회피
        '''
        twist = Twist()
        twist.angular.z = self.avoidance_left
        self.vel_pub.publish(twist)
        self.get_logger().info("Avoiding wall on the right")


def main(args=None):
    rclpy.init(args=args)

    wall_avoidance = WallAvoidance()
    rclpy.spin(wall_avoidance)

    wall_avoidance.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()