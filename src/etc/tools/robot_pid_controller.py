

class RobotPIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        
        # 이전 오차 및 적분값 초기화
        self.previous_error = 0.0
        self.integral = 0.0 # 목표 상태에서 벗어나 있는 시간이 길어질수록 더 큰 값을 가진다. 
        
        
    def compute(self, error, dt):
        
        # 비례
        p = self.kp * error
        
        # 적분
        self.integral += error * dt
        i = self.ki * self.integral
        
        # 미분
        d = self.kd * (error - self.previous_error) / dt if dt > 0 else 0.0
        self.previous_error = error
        
        if output < 0:
            output += -0.2
        elif output > 0:
            output += 0.2
        
        return output