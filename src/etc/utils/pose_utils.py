from geometry_msgs.msg import Pose, PoseStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class PoseUtils:
    @staticmethod
    def create_pose(x, y, z=0.0):
        """
        지정된 x, y, z 좌표로 Pose 메시지를 생성합니다.

        Parameters:
            x (float): 위치의 x 좌표.
            y (float): 위치의 y 좌표.
            z (float, optional): 위치의 z 좌표. 기본값은 0.0

        Returns:
            Pose: 지정된 위치를 가지는 Pose 메시지.
        """
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        return pose

    @staticmethod
    def create_pose_stamped(x, y, yaw, timestamp=None, frame_id='map'):
        """
        지정된 위치와 yaw 회전 값으로 PoseStamped 메시지를 생성합니다.

        Parameters:
            x (float): 위치의 x 좌표.
            y (float): 위치의 y 좌표.
            yaw (float): yaw 회전(Z축 회전) 각도(라디안 단위).
            timestamp (builtin_interfaces.msg.Time, optional): PoseStamped 헤더의 타임스탬프.
                값이 주어지지 않으면 타임스탬프는 설정되지 않습니다.
            frame_id (str, optional): 좌표계 프레임 ID. 기본값은 'map'

        Returns:
            PoseStamped: 지정된 위치와 회전, 타임스탬프, frame_id를 포함하는 PoseStamped 메시지.
        """
        q_x, q_y, q_z, q_w = quaternion_from_euler(0, 0, yaw)

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        if timestamp:
            pose.header.stamp = timestamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w

        return pose    

    @staticmethod
    def get_yaw_from_quaternion(orientation):
        """
        쿼터니언에서 yaw (Z축 회전) 각도를 추출합니다.

        Parameters:
            orientation (geometry_msgs.msg.Quaternion): 방향을 나타내는 쿼터니언.

        Returns:
            float: 라디안 단위의 yaw 각도.
        """
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]
        return yaw