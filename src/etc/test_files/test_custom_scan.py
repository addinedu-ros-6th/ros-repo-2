import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data
import math


class CustomScan(Node):
    def __init__(self):
        super().__init__('custom_scan')
        self.robot1_pose = Pose()
        self.robot1_pose_subscriber = self.create_subscription(Pose, '/robot1/pose', self.pose_callback_robot1, 10)

        self.robot2_pose = Pose()
        self.robot2_pose_subscriber = self.create_subscription(Pose, '/robot2/pose', self.pose_callback_robot2, 10)

        self.robot3_pose = Pose()
        self.robot3_pose_subscriber = self.create_subscription(Pose, '/robot3/pose', self.pose_callback_robot3, 10)

        self.scan_data = LaserScan()
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)


    def pose_callback_robot1(self, msg: Pose):
        self.robot1_pose = msg
    
    def pose_callback_robot2(self, msg: Pose):
        self.robot2_pose = msg
    
    def pose_callback_robot3(self, msg: Pose):
        self.robot3_pose = msg

    def scan_callback(self, msg: LaserScan):
        scan = msg
        num_readings = 260

        # 로봇1 방향
        robot1_yaw = euler_from_quaternion([0, 
                                            0, 
                                            self.robot1_pose.orientation.z, 
                                            self.robot1_pose.orientation.w])[2]
        scan_index_from_yaw = int((robot1_yaw - scan.angle_min) / scan.angle_increment)  # 회전 보상을 위한 라이다 인덱스 값

        # 로봇1 기준 로봇2의 거리와 각도
        dx_1 = self.robot2_pose.position.x - self.robot1_pose.position.x
        dy_1 = self.robot2_pose.position.y - self.robot1_pose.position.y
        
        distance_1 = math.hypot(dx_1, dy_1)
        angle_1 = math.atan2(dy_1, dx_1)

        # 로봇2 위치에 대한 라이다 값 인덱스
        index_1 = (int((angle_1 - scan.angle_min) / scan.angle_increment) - scan_index_from_yaw - 130) % num_readings
        
        scan.ranges[index_1] = distance_1
        scan.ranges[(index_1+1)%num_readings] = distance_1 / math.cos(scan.angle_increment)
        scan.ranges[(index_1+2)%num_readings] = distance_1 / math.cos(scan.angle_increment*2)
        scan.ranges[index_1-1] = distance_1 / math.cos(scan.angle_increment)
        scan.ranges[index_1-2] = distance_1 / math.cos(scan.angle_increment*2)

        print("center index:", index_1)
        print("ranges[index_1]:", scan.ranges[index_1])
        print("ranges[index_1+1]:", scan.ranges[(index_1+1)%num_readings])
        print("ranges[index_1+2]:", scan.ranges[(index_1+2)%num_readings])
        print("ranges[index_1-1]:", scan.ranges[index_1-1])
        print("ranges[index_1-2]:", scan.ranges[index_1-2])

        # 로봇1 기준 로봇3의 거리와 각도
        # dx_2 = self.robot3_pose.position.x - self.robot1_pose.position.x
        # dy_2 = self.robot3_pose.position.y - self.robot1_pose.position.y

        # distance_2 = math.hypot(dx_2, dy_2)
        # angle_2 = math.atan2(dy_2, dx_2)

        # index_2 = (int((angle_2 - scan.angle_min) / scan.angle_increment) - scan_index_from_yaw - 130) % num_readings

        # scan.ranges[index_2] = distance_2
        # scan.ranges[index_2+1] = distance_2 / math.cos(scan.angle_increment)
        # scan.ranges[index_2+2] = distance_2 / math.cos(scan.angle_increment*2)
        # scan.ranges[index_2-1] = distance_2 / math.cos(scan.angle_increment)
        # scan.ranges[index_2-2] = distance_2 / math.cos(scan.angle_increment*2)


def main(args=None):
    rclpy.init(args=args)

    node = CustomScan()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
