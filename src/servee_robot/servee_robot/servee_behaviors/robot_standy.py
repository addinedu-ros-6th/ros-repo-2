from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access
from std_msgs.msg import Bool
from servee_interfaces.msg import TableState

class RobotStandy(Behaviour):
    def __init__(self, name:str):
        super(RobotStandy, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="robot_state", access=Access.WRITE)
        self.blackboard.register_key(key="robot_state", access=Access.READ)
        
        self.blackboard.register_key(key="goal_poses", access=Access.READ)
        self.blackboard.register_key(key="goal_pose", access=Access.WRITE)
        
        self.blackboard.register_key(key="aruco_ids", access=Access.READ)
        self.blackboard.register_key(key="aruco_id_index", access=Access.WRITE)
        self.blackboard.register_key(key="aruco_id_index", access=Access.READ)
        
        self.blackboard.register_key(key="robot_type", access=Access.READ)

    def setup(self, **kwargs):
        self.node: Node = kwargs['node'] 
        
        self.test_timer = 0
        
        
        self.node.create_subscription(
            TableState,
            "/table_check",
            self.callback,
            10
        )
        
        
    def callback(self, msg):
              
        
          
        # 로봇 타입 키 존재 여부 확인
        if self.blackboard.exists("robot_type") == False:
            self.node.get_logger().warn(f"robot_type: {self.blackboard.robot_type}")
            return
        
        # 현재 상태 확인하고 
        if self.blackboard.robot_state not in ['standy']:
            return
        
        # 회수 로봇인지 확인하고
        if self.blackboard.robot_type == "serving":
            self.node.get_logger().warn(f"IS serving: {self.blackboard.robot_type}")
            return
        
        # 회수용 로봇은 첫 방문지가 테이블이다. 
        if self.blackboard.aruco_id_index != 0:
            self.node.get_logger().warn(f"index : {self.blackboard.aruco_id_index}")
            return
        
        
        aruco_id = self.blackboard.aruco_ids[self.blackboard.aruco_id_index]
        self.node.get_logger().warn(f"aruco_id : {aruco_id}")
        
        table_id = aruco_id - 12
        self.node.get_logger().warn(f"table_id : {table_id}")
        
        # self.node.get_logger().warn(f"msg.data[table_id]: : {msg.status[table_id]}")
        
        if msg.status[table_id] == True:
            self.request_next_path()
        
 
 
    def request_next_path(self):
        self.blackboard.aruco_id_index += 1
        self.blackboard.goal_pose = self.blackboard.goal_poses.poses[self.blackboard.aruco_id_index]
        self.blackboard.robot_state = 'receive_goal'
        self.node.get_logger().warn(f"다음 목적지 {self.blackboard.goal_pose}")
    
    
    
    def is_serving_complete(self):
        
        if self.blackboard.robot_type != "serving":
            return
        
        if self.blackboard.aruco_id_index == 0:
            self.request_next_path()
        
        # 서빙 로봇은 두번째 방문지가 테이블이다. 
        if self.blackboard.aruco_id_index != 1:
            return
        
        self.node.get_logger().warn(f"Timer Check")
                
        if self.test_timer >= 50:
            self.node.get_logger().error(f"다음 동선으로")
            self.test_timer = 0
            self.request_next_path()
        
             
        # 쿨타임 체크     
        else:
            self.test_timer += 1


    def update(self):
        # 현재 상태 확인하고 
        if self.blackboard.robot_state not in ['standy']:
            return Status.FAILURE
        
        if self.blackboard.aruco_id_index >= len(self.blackboard.aruco_ids) -1:
            self.blackboard.robot_state = 'idle'
            self.node.get_logger().warn(f"standby: idle 변환, 현재 동선: {self.blackboard.goal_poses}")
            
        else:
            self.is_serving_complete()
            
        return Status.SUCCESS
        


        

            
        
        
        


