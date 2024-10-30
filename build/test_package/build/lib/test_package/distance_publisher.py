import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float32

class DistancePublisher(Node):
    def __init__(self):
        super().__init__('distance_publisher')
        self.publisher_ = self.create_publisher(Float32, 'distance', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)  # Update with your port
        self.timer = self.create_timer(0.5, self.publish_distance)

    def publish_distance(self):
        if self.serial_port.in_waiting > 0:
            try:
                # Reading the line sent by Arduino
                data = self.serial_port.readline().decode('utf-8').strip()
                msg = Float32()
                msg.data = float(data)
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing distance: {data} cm')
                # if data.startswith("Distance:"):
                #     distance_value = float(data.split(':')[1].replace('cm', ''))
                #     msg = Float32()
                #     msg.data = distance_value
                #     self.publisher_.publish(msg)
                #     self.get_logger().info(f'Publishing distance: {distance_value} cm')
            except Exception as e:
                self.get_logger().error(f'Error reading serial data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DistancePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
