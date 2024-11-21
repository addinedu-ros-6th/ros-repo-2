from py_trees.behaviour import Behaviour
from typing import Any
from py_trees.common import Status, Access
from rclpy.node import Node
from std_msgs.msg import String

class HumanStopAndGo(Behaviour):
    def __init__(self, name: str):
        super(HumanStopAndGo, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)

        self.blackboard.register_key(key="is_human_detected", access=Access.WRITE)
        self.blackboard.register_key(key="is_human_detected", access=Access.READ)
        self.blackboard.is_human_detected = False
        
        
    def setup(self, **kwargs: Any) -> None:
        self.node : Node = kwargs['node']
    
        self.node.create_subscription(
            String,
            "table_status",
            self.callback,
            10
        )
    
    def update(self) -> Status:
        if self.blackboard.is_human_detected: 
            return Status.SUCCESS
        else:
            return Status.FAILURE
    
    
    def callback(self, msg):
        is_human = msg.data
        
        if is_human == "stop":
            self.blackboard.is_human_detected = True
        else:
            self.blackboard.is_human_detected = False