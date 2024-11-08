from typing import Any
from py_trees.behaviour import Behaviour
from py_trees.common import Status


from rclpy.node import Node
from std_msgs.msg import Bool

class LedFlasher(Behaviour):
    def __init__(self, name: str):
        super(LedFlasher, self).__init__(name)
    
    def setup(self, **kwargs: Any) -> None:
        self.flash_pub = FlashLedPublisher()
        
    def update(self) -> Status:
        # LED 깜빡이는거 추가 필요
        
        self.flash_pub.publish()
        return Status.SUCCESS
    
    
class FlashLedPublisher(Node):
    def __init__(self):
        super().__init__("flash_led_pub_node")
        
        # 추후에 minibot_interfaces/msg/LampCommand 로 변경할 것. 
        self.publisher_ = self.create_publisher(Bool, "/minibot_io_controller/set_lamp", 10)
        
    def publish(self):  
        self.get_logger().debug("LED 깜빡이는 것은 추후에 개발할 예정입니다.")
        # 추후에 . . .       
        #self.publisher_.publish(Bool(data=True))