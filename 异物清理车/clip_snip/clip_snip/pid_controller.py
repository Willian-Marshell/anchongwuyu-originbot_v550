class PIDController:
    def __init__(self, kp, ki, kd, max_output, min_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max = max_output
        self.min = min_output
        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self, setpoint, current_value, dt):
        error = setpoint - current_value
        
        # PID计算
        proportional = self.kp * error
        self.integral += self.ki * error * dt
        derivative = self.kd * (error - self.previous_error) / dt
        
        # 抗积分饱和
        self.integral = max(min(self.integral, self.max), self.min)
        
        output = proportional + self.integral + derivative
        self.previous_error = error
        
        return max(min(output, self.max), self.min)