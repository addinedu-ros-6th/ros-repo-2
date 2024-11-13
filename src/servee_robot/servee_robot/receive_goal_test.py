import rclpy
from rclpy.node import Node
from servee_interfaces.msg import TaskGoalPoses
from geometry_msgs.msg import PoseArray, Pose

class ReceiveGoalTest(Node):
    def __init__(self):
        super().__init__('servee_goal_test_node')
        self.publisher = self.create_publisher(
            TaskGoalPoses,
            '/servee/task_goal_poses',
            10
        )

    def test_pub(self):
        # TaskGoalPoses 메시지 생성
        task_goal_poses = TaskGoalPoses()
        pose_array = PoseArray()

        # 첫 번째 좌표 설정
        pose1 = Pose()
        pose1.position.x = 0.67
        pose1.position.y = 2.1
        pose1.position.z = 0.0
        pose1.orientation.x = 0.0
        pose1.orientation.y = 0.0
        pose1.orientation.z = 0.0
        pose1.orientation.w = 1.0

        # 두 번째 좌표 설정
        pose2 = Pose()
        pose2.position.x = 0.0
        pose2.position.y = 0.0
        pose2.position.z = 0.0
        pose2.orientation.x = 0.0
        pose2.orientation.y = 0.0
        pose2.orientation.z = 0.0
        pose2.orientation.w = 1.0

        # PoseArray에 좌표 추가
        pose_array.poses.append(pose1)
        pose_array.poses.append(pose2)

        # TaskGoalPoses에 PoseArray 할당
        task_goal_poses.goal_poses = pose_array

        # 퍼블리시
        self.publisher.publish(task_goal_poses)
    
def main(args=None):
    rclpy.init(args=args)
    node = ReceiveGoalTest()
    node.test_pub() 
    node.destroy_node()
    rclpy.shutdown()
    
    
if __name__ =="__main__":
    main()