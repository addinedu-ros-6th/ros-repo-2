import rclpy
from rclpy.node import Node
import py_trees_ros
from rclpy.parameter import Parameter
import sensor_msgs.msg as sensor_msgs


class BatteryStatePub(Node):
    def __init__(self):
        super().__init__("battery_state_pub")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('charging', False),
                ('charging_increment', 0.1),
                ('charging_percentage', 100.0),
            ]
        )

        self.publisher = self.create_publisher(
            sensor_msgs.BatteryState,
            'servee/battery_state',
            10
        )
        
                    
        self.battery = sensor_msgs.BatteryState()      
                
        self.timer = self.create_timer(
            timer_period_sec=0.2,
            callback=self.update_and_publish
        )

        
    def update_and_publish(self):
        """
        배터리 상태를 Update 한 후에 Publish
        """           
        charging = self.get_parameter("charging").value
        charging_increment = self.get_parameter("charging_increment").value
        charging_percentage = self.get_parameter("charging_percentage").value
                
        if charging:
            charging_percentage = min(100.0, charging_percentage + charging_increment)
            if charging_percentage % 5.0 < 0.1:
                self.get_logger().debug("Charging...{:.1f}%%".format(charging_percentage))
                
        else:
            charging_percentage = max(0.0, charging_percentage - charging_increment)
            if charging_percentage % 2.5 < 0.1:
                self.get_logger().debug("Discharging...{:.1f}%%".format(charging_percentage))

        self.battery.header.stamp = rclpy.clock.Clock().now().to_msg()
        self.battery.percentage = charging_percentage
        
        if charging_percentage == 100.0:
            self.battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_FULL
        elif charging:
            self.battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            self.battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        
        self.publisher.publish(msg=self.battery)
        
        self.set_parameters(
           [Parameter("charging_percentage", Parameter.Type.DOUBLE, float(charging_percentage))]
        )
        
def main():
    rclpy.init()
    battery = BatteryStatePub()
    rclpy.spin(battery)
    battery.destroy_node()
    rclpy.shutdown()
