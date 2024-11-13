import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from servee_interfaces.msg import ArucoMarkerResult
import tf_transformations
import math
import time
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32
from rclpy.time import Duration

class ParkingFSM(Node):
    def __init__(self, seconds_sleeping=5):
        super().__init__('parking_fsm')
        self._seconds_sleeping = seconds_sleeping

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.group0 = MutuallyExclusiveCallbackGroup()
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = ReentrantCallbackGroup()

        self.twist_pub = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10, callback_group=self.group0)
        self.marker_sub = self.create_subscription(ArucoMarkerResult, '/aruco_marker_data', self.marker_callback, 10, callback_group=self.group1)
        self.odom_sub = self.create_subscription(Odometry, '/base_controller/odom', self.odom_callback, 10, callback_group=self.group2)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_scan_callback, qos_profile, callback_group=self.group3)

        # Aruco 마커 검출 타임스탬프 
        self.marker_detected = False
        self.marker_detected_time = self.get_clock().now() 

        # 초기 상태 'STANDBY'
        self.state = 'STANDBY'

        # Aruco 마커 정보 
        self.target_marker_id = 10
        self.marker_id = 10
        self.marker_x = 10.0
        self.marker_z = 10.0
        self.marker_yaw = 10.0
        self.marker_centerline_error = 10.0
        self.marker_theta = 0.0  # Angle for rotation
        self.marker_detected = False
        
        # odom 정보
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        ## laser scan 결과
        self.closest_line_distance = 100.0
        self.closest_line_angle = 100.0

        ## 전진 & 회전
        self.rotation_count = 0 
        self.initial_yaw = None
        self.initial_position = None

        # 라이다로 측정한 벽의 각도 허용 오차 설정
        self.angle_tolerance = math.radians(5)  # Angle tolerance in radians

        self.declare_parameter('scale', 700)
        self.declare_parameter('hough_threshold', 60)
        self.declare_parameter('lin_vel', 0.2)
        self.declare_parameter('ang_vel', 0.2)
        self.declare_parameter('centerline_error_tolerance', 50)
        self.declare_parameter('distance_tolerance', 0.05)
        self.declare_parameter('closest_line_angle_tolerance', 0.02)  #lider 이용 회전 시 임계값
        self.declare_parameter('marker_timeout_tolerance', 3.0)
        self.declare_parameter('yaw_tolerance', 1.0)

        self.scale = self.get_parameter('scale').get_parameter_value().integer_value
        self.hough_threshold = self.get_parameter('hough_threshold').get_parameter_value().integer_value
        self.lin_vel = self.get_parameter('lin_vel').get_parameter_value().double_value
        self.ang_vel = self.get_parameter('ang_vel').get_parameter_value().double_value
        self.centerline_error_tolerance = self.get_parameter('centerline_error_tolerance').get_parameter_value().integer_value
        self.distance_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        self.closest_line_angle_tolerance = self.get_parameter('closest_line_angle_tolerance').get_parameter_value().double_value
        self.marker_timeout_tolerance = self.get_parameter('marker_timeout_tolerance').get_parameter_value().double_value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').get_parameter_value().double_value
        
        self.timer = self.create_timer(0.1, self.run_fsm)
        
    def marker_callback(self, msg):
        # Extract data from ArucoMarkerResult message
        self.marker_id = msg.id
        self.marker_x = msg.x
        self.marker_z = msg.z / 2
        self.marker_yaw = msg.yaw
        self.marker_centerline_error = msg.centerline_error

        self.marker_detected = True
        self.marker_detected_time = self.get_clock().now()  

        # Calculate theta
        self.marker_theta = math.atan2(self.marker_z, self.marker_x)
        # self.get_logger().info(f"marker ID: {self.marker_id}")

    def odom_callback(self, msg):
        # Update current position and orientation
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = tf_transformations.euler_from_quaternion(orientation_list)
        self.current_yaw = yaw
        # self.get_logger().info(f" currnent x, y, yaw: {self.current_x:.2f}, {self.current_y:.2f}, {self.current_yaw:.2f}")

    def laser_scan_callback(self, msg):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)

        # 유효한 거리 값 필터링
        valid_indices = np.isfinite(ranges)
        angles = angles[valid_indices]
        ranges = ranges[valid_indices]
        
        # 극좌표를 직교좌표로 변환
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # 직선 검출 수행
        self.detect_lines(x, y)
    
    def detect_lines(self, x, y):
        # Line detection algorithm implementation
        img_size = 500
        img = np.zeros((img_size, img_size), dtype=np.uint8)
        offset = img_size // 2

        # (x, y) 좌표들을 수직으로 쌓아서 포인트 배열을 생성
        points = np.vstack((x, y)).T
        points = np.int32(points * self.scale + offset)  # 스케일 조정 및 오프셋 적용
        for point in points:
            if 0 <= point[0] < img_size and 0 <= point[1] < img_size:
                cv2.circle(img, tuple(point), 1, 255, -1)  # 해당 위치에 흰색 점을 그림
        
        # 허프 변환을 사용하여 직선을 검출합니다.
        lines = cv2.HoughLines(img, 1, np.pi / 180, self.hough_threshold)

        if lines is not None:
            adjusted_lines = []
            
            # 검출된 각 직선에 대해 처리
            for line in lines:
                rho, theta = line[0]
                
                # 각도 범위를 -π/2 ~ π/2로 조정
                if theta > np.pi / 2:
                    theta -= np.pi
                elif theta < -np.pi / 2:
                    theta += np.pi
                adjusted_lines.append([rho, theta])

            # 특정 각도 범위(-1 ~ 1 라디안) 내의 직선만 필터링
            filtered_lines = [line for line in adjusted_lines if -1 <= line[1] <= 1]

            if filtered_lines:
                # 가장 가까운 직선을 찾습니다.
                closest_line = min(filtered_lines, key=lambda line: abs(line[0]))
                self.closest_line_distance = closest_line[0] # 직선까지의 거리 저장
                self.closest_line_angle = closest_line[1]    # 직선의 각도 저장
                
        #         # Log information
        #         self.get_logger().info(f"Closest line with distance: {self.closest_line_distance:.2f}, angle: {self.closest_line_angle:.2f}")
        #     else:
        #         self.get_logger().info("Lines not within angle range")
        # else:
        #     self.get_logger().info("No lines detected")


    def rotate_by_angle(self, target_angle):
        
        twist = Twist()
        """
        odometry를 이용해서 target_angle 만큼 회전한다.
        """
        if self.initial_yaw is None:
            self.initial_yaw = self.current_yaw
        yaw_error = target_angle - (self.current_yaw - self.initial_yaw)
        if abs(yaw_error) > 0.1:
            twist.angular.z = self.ang_vel if yaw_error > 0 else -self.ang_vel
            self.twist_pub.publish(twist)
        else:
            twist.angular.z = 0.0
            self.twist_pub.publish(twist)
            self.initial_yaw = None  # Reset for next rotation


    def move_by_distance(self, target_distance):
        """
        odometry를 이용해서 target_distance 만큼 직진한다. 
        """
        twist = Twist()
        
        # 초기 위치를 설정합니다.
        if self.initial_position is None:
            self.initial_position = (self.current_x, self.current_y)
            
        # 현재 위치와 초기 위치 간의 변위를 계산합니다.
        dx = self.current_x - self.initial_position[0]
        dy = self.current_y - self.initial_position[1]
        dist_moved = math.hypot(dx, dy)
        
        # 목표 거리(target_distance)까지 이동이 완료되지 않은 경우 전진합니다.
        if dist_moved < target_distance:
            twist.linear.x = self.lin_vel
            self.twist_pub.publish(twist)
                
        # 목표 거리에 도달한 경우 이동을 멈추고 초기 위치를 재설정합니다.
        else:
            twist.linear.x = 0.0
            self.twist_pub.publish(twist)
            self.initial_position = None  # 다음 이동을 위해 초기 위치 초기화

    def check_marker_timeout(self):
        """Check if the marker has not been detected for over 3 second."""
        current_time = self.get_clock().now()
        if self.marker_detected and (current_time - self.marker_detected_time) > Duration(seconds=self.marker_timeout_tolerance):
            self.marker_detected = False

    def run_fsm(self):
        """Finite State Machine to control parking behavior."""
        # Check for aruco marker timeout
        self.check_marker_timeout()

        if self.state == 'STANDBY':
            self.standby()
        elif self.state == 'SEARCHING':
            self.searching()
        elif self.state == 'ALIGNING':
            self.aligning()
        elif self.state == 'APPROACHING':
            self.approaching()
        elif self.state == 'YAWING':
            self.yawing()

    #STANDBY 상태: 제자리에서 회전하며 aruco marker 검출 여부 파악
    def standby(self):
        """Reset to standby state and wait for reactivation."""
        self.get_logger().info("Standby")
        time.sleep(1)
        self.state = 'SEARCHING'
        time.sleep(0.1)

    def searching(self):
        """Searching for the Aruco marker."""
        self.get_logger().info("Searching")
        
        # 마커가 검출된 동안 반복하여 로봇을 회전시켜 정렬을 시도합니다.
        while self.marker_detected:
            if abs(self.marker_centerline_error) > self.centerline_error_tolerance:
                twist = Twist()
                twist.angular.z = -self.ang_vel if self.marker_centerline_error > 0 else self.ang_vel
                self.twist_pub.publish(twist)
            else: 
                self.state = 'ALIGNING'      
                time.sleep(0.1)
                break
            
        # 마커가 검출되지 않은 경우 로봇을 회전하며 마커를 찾습니다.
        else:
            twist = Twist()
            twist.angular.z = self.ang_vel
            self.twist_pub.publish(twist)
            self.rotation_count += 1
            
            # rotation count 가 500 이상이 되면 STANDBY 상태로 전환
            if self.rotation_count >= 500:  
                self.rotation_count = 0
                self.state = 'STANDBY'
                time.sleep(0.1)
                self.get_logger().info("Marker not found. Returning to Standby.")


    #ALIGNING 상태: aruco marker 의 법선와 로봇의 시선으로 작도한 직각삼각형의 내접원의 중심으로 이동
    def aligning(self):
        """Align with the detected Aruco marker."""
        self.get_logger().info("Aligning")

        while self.marker_detected:
            self.check_marker_timeout()
            
            # 거리가 만족인 경우
            if math.hypot(self.marker_x, self.marker_z) < self.distance_tolerance:
                self.state = 'YAWING'
                time.sleep(0.1)
                break
            
            # 바라보는 방향이 만족인 경우
            elif abs(self.marker_yaw) < self.yaw_tolerance:
                self.state = 'APPROACHING'
                time.sleep(0.1)
                break
            
            
            # 마커와 정렬되지 않은 경우 회전해서 각도 조정.
            else:
                self.rotate_by_angle(-self.marker_theta / 2)
                distance = (self.marker_x * self.marker_z) / (
                    (math.sin(self.marker_theta / 2)) * (self.marker_x + self.marker_z + math.hypot(self.marker_x, self.marker_z)))
                self.move_by_distance(distance*1.5)
                self.rotate_by_angle(np.sign(self.marker_theta) * math.pi / 4)
                while True:
                    if abs(self.marker_centerline_error) > self.centerline_error_tolerance:
                        twist = Twist()
                        twist.angular.z = -self.ang_vel if self.marker_centerline_error > 0 else self.ang_vel
                        self.twist_pub.publish(twist)
                    else: 
                        break
        else:
            self.get_logger().info("Marker not found. Returning to Searching.")
            self.state = 'SEARCHING'
            time.sleep(0.1)

    #APPROACHING 상태: 로봇과 aruco marker 이 align 되었음을 가정하고 aruco marker로 접근
    def approaching(self):
        """Approach the Aruco marker until within target distance."""
        self.get_logger().info("Approaching")

        while self.marker_detected:
            # 마커 검출 시간 초과 여부 확인
            self.check_marker_timeout()
            twist = Twist()
            
            
            # 허용 오처 범위 밖이라면 
            if math.hypot(self.marker_x, self.marker_z) > self.distance_tolerance:
                # 중심점에서 벗어났다면
                if abs(self.marker_centerline_error) > self.centerline_error_tolerance:
                    twist.linear.x = 0.0
                    twist.angular.z = -self.ang_vel * np.sign(self.marker_centerline_error)
                    
                # 전진
                else:
                    twist.linear.x = self.lin_vel
                    twist.angular.z = 0.0
                    
                self.twist_pub.publish(twist)
                self.get_logger().info("Approaching Aruco marker...")
                
            # 마커와의 거리가 허용 오차 이내인 경우 상태를 'YAWING'으로 전환
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.twist_pub.publish(twist)
                self.state = 'YAWING'
                time.sleep(0.1)
                break
            
        else:
            self.get_logger().info("Marker not found. Returning to Searching.")
            self.state = 'SEARCHING'
            time.sleep(0.1)

    #YAWING 상태: 라이다로 검출한 벽(혹은 테이블) 을 기준으로 로봇의 시선이 수직이 되도록 로봇의 yaw조정
    def yawing(self):
        """Stop at the target distance and finish the yawing."""
        self.get_logger().info("Yawing")

        print("has scan attribute : ", hasattr(self, 'scan_data'))

        twist = Twist()
        
         # 라이다로 검출한 벽의 각도가 허용 오차를 초과하는 경우, 각도를 조정합니다.
        if abs(self.closest_line_angle) > self.closest_line_angle_tolerance:
            twist.angular.z = self.ang_vel * np.sign(self.closest_line_angle)
            self.twist_pub.publish(twist)
        else:
            # 각도가 허용 오차 이내라면 회전을 멈추고 상태를 'STANDBY'로 전환합니다.
            twist.angular.z = 0.0
            self.twist_pub.publish(twist)

            self.get_logger().info("Parking accomplished")
            self.state = 'STANDBY'
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    parking_fsm = ParkingFSM()

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(parking_fsm)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        parking_fsm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()