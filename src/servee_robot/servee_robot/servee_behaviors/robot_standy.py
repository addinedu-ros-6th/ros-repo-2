from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access


class RobotStandy(Behaviour):
    def __init__(self, name:str):
        super(RobotStandy, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="robot_state", access=Access.WRITE)
        self.blackboard.register_key(key="robot_state", access=Access.READ)
        
        self.blackboard.register_key(key="aruco_ids", access=Access.READ)
        self.blackboard.register_key(key="aruco_id_index", access=Access.WRITE)
        self.blackboard.register_key(key="aruco_id_index", access=Access.READ)
        
        self.blackboard.register_key(key="waypoint", access=Access.WRITE)
        
    def setup(self, **kwargs):
        self.test_timer = 0  
    
    def update(self):
        
        if self.blackboard.robot_state not in ['standy']:
            return Status.SUCCESS
        

        # 쿨타임 체크
        if self.test_timer >= 50:
            
            self.test_timer = 0
            
            if self.blackboard.aruco_id_index >= len(self.blackboard.aruco_ids):
                # 모든 동선을 돌았다. (여기에 홈 추가할 예정.)
                self.blackboard.robot_state = 'idle'
                    
            else:
                self.blackboard.aruco_id_index += 1
                self.blackboard.waypoint = 0
                self.blackboard.robot_state = 'task'
           
        # 쿨타임 체크     
        else:
            self.test_timer += 1
        
        return Status.SUCCESS