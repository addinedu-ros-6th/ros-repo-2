from rclpy.node import Node
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access


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

    def setup(self, **kwargs):
        self.node: Node = kwargs['node'] 
        self.test_timer = 0  
    
    def update(self):
        
        if self.blackboard.robot_state not in ['standy']:
            return Status.FAILURE
        

        # 쿨타임 체크
        if self.test_timer >= 50:
            
            self.test_timer = 0
            
            if self.blackboard.aruco_id_index >= len(self.blackboard.aruco_ids):
                # 모든 동선을 돌았다. (여기에 홈 추가할 예정.)
                self.blackboard.robot_state = 'idle'
                self.node.get_logger().warn(f"standby: idle 변환, 현재 동선: {self.blackboard.goal_poses}")
                return Status.SUCCESS
                    
            else:
                
                self.blackboard.aruco_id_index += 1
                self.blackboard.goal_pose = self.blackboard.goal_poses.poses[self.blackboard.aruco_id_index]
                self.blackboard.robot_state = 'receive_goal'
                self.node.get_logger().warn(f"다음 목적지 {self.blackboard.goal_pose}")
                return Status.SUCCESS
            
                # self.blackboard.goal_poses = msg.goal_poses
                # self.blackboard.aruco_ids = msg.aruco_id
                # self.blackboard.aruco_id_index = 0
                
                # self.node.get_logger().info(f"배열로 받은 목적지 {self.blackboard.goal_poses}")
                
                # self.blackboard.goal_pose = self.blackboard.goal_poses.poses[0]
                # self.blackboard.robot_state = "receive_goal"
                
           
        # 쿨타임 체크     
        else:
            self.test_timer += 1
            return Status.SUCCESS
        
