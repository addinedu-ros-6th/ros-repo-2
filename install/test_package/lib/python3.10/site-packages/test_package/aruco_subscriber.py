import rclpy
from rclpy.node import Node
from test_package_msgs.msg import ArucoMarker  # Import the custom message

class ArucoMarkerSubscriber(Node):
    def __init__(self):
        super().__init__('aruco_marker_subscriber')
        
        # Create a subscription to the aruco_marker_data topic
        self.subscription = self.create_subscription(
            ArucoMarker,
            'aruco_marker_data',
            self.marker_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def marker_callback(self, msg):
        # Print the received data
        self.get_logger().info(
            f"Received Marker - ID: {msg.id}, "
            f"Distance: {msg.distance:.2f}m, "
            f"X: {msg.x:.2f}m, Y: {msg.y:.2f}m, Z: {msg.z:.2f}m"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()