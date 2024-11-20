import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import random

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        
        # Pose publisher
        self.pose_publisher = self.create_publisher(Pose, '/robot1/pose', 10)
        # State publisher
        self.state_publisher = self.create_publisher(String, '/robot1/state', 10)

        # Timer to publish pose data every 0.1 seconds
        self.pose_timer = self.create_timer(0.1, self.publish_pose)
        # Timer to publish state data every 0.5 seconds
        self.state_timer = self.create_timer(0.2, self.publish_state)

        # State management
        self.states = ["idle", "running1", "standby1", "running2", "standby2"]
        self.state_counts = {"idle": 3, "running1": 10, "standby1": 10, "running2": 10, "standby2": 10}
        self.state_index = 0  # Index to track the current state
        self.current_count = 0  # Track how many times the current state has been published

    def publish_pose(self):
        # Create a Pose message with random vibration within a range
        pose_msg = Pose()
        pose_msg.position.x = random.uniform(-1.0, 1.0)
        pose_msg.position.y = random.uniform(-1.0, 1.0)
        pose_msg.position.z = random.uniform(0.0, 0.5)
        pose_msg.orientation.x = random.uniform(-0.1, 0.1)
        pose_msg.orientation.y = random.uniform(-0.1, 0.1)
        pose_msg.orientation.z = random.uniform(-0.1, 0.1)
        pose_msg.orientation.w = 1.0  # Keeping w constant for simplicity

        self.pose_publisher.publish(pose_msg)
        # self.get_logger().info(f'Published Pose: {pose_msg}')

    def publish_state(self):
        # Get the current state and the maximum count for it
        current_state = self.states[self.state_index]
        max_count = self.state_counts[current_state]

        # Publish the current state
        state_msg = String()
        state_msg.data = current_state
        self.state_publisher.publish(state_msg)
        self.get_logger().info(f'Published State: {state_msg.data} ({self.current_count + 1}/{max_count})')

        # Increment the count for the current state
        self.current_count += 1

        # Check if the current state has been published enough times
        if self.current_count >= max_count:
            # Move to the next state
            self.state_index = (self.state_index + 1) % len(self.states)
            self.current_count = 0  # Reset the count for the new state

def main(args=None):
    rclpy.init(args=args)
    test_publisher = TestPublisher()
    
    try:
        rclpy.spin(test_publisher)
    except KeyboardInterrupt:
        pass

    test_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()