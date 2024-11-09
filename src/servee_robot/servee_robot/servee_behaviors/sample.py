def update(self) -> Status:
    if self.are_you_ready() == False:
        return Status.FAILURE
        
    # odom으로 계산한 오차가 yaw_tolerance보다 작은지 확인.
    if abs(self.odom_yaw_error) <= self.odom_yaw_threshold:
        # 상대 좌표 관련 변수 초기화
        self.reset_values()
        
        # 목표 방향으로 회전해야 하는 각도 계산
        target_yaw = self.calculate_target_angle()
        yaw_error = target_yaw - self.blackboard.yaw
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi
        
        if abs(yaw_error) >= self.absolute_yaw_threshold:
            self.odom_yaw_error = yaw_error
            self.node.get_logger().info(f"절대 오차를 odom_yaw_error에 추가: {self.odom_yaw_error}")
            return Status.FAILURE
        else:
            # yaw 오차 없음. 
            self.node.get_logger().info(f"yaw 오차가 없다구? {self.odom_yaw_error}")
            return Status.SUCCESS   
    else:            
        # 회전하며 self.odom_yaw_error값 줄이기
        current_time = self.node.get_clock().now()
        
        # prev_time이 None이면 초기화 후 첫 프레임에서의 dt를 0으로 설정
        if self.prev_time is None:
            dt = 0.1
            self.prev_time = current_time
        else:
            dt = (current_time - self.prev_time).nanoseconds * 1e-9  # 초 단위로 변환
        
        # 현재 yaw 값 계산
        curr_yaw = PoseUtils.get_yaw_from_quaternion(self.blackboard.odom_pose.orientation)
        
        if self.prev_yaw is not None:
            yaw_difference = curr_yaw - self.prev_yaw
            # 여기서 각도 차이를 -π에서 π 사이로 정규화
            yaw_difference = (yaw_difference + math.pi) % (2 * math.pi) - math.pi
            self.odom_yaw_error -= yaw_difference
            self.node.get_logger().info(f"현재 odom yaw 오차 {self.odom_yaw_error}")
        
        angular_speed = max(min(self.pid.compute(self.odom_yaw_error, dt), self.max_angular_speed), -self.max_angular_speed)
        self.twist_publish(angular_speed)
        
        self.prev_yaw = curr_yaw
        self.prev_time = current_time
        
        return Status.FAILURE
