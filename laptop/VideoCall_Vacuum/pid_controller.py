class PIDController:
    def __init__(self, kp, ki, kd, setpoint, output_limits=(-1, 1), deadzone=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self.deadzone = deadzone
        self.integral = 0
        self.previous_error = 0
        self.last_time = None

    def calculate(self, current_value, current_time, deadzone):
        if self.last_time is None:
            self.last_time = current_time
            return 0.0

        dt = current_time - self.last_time
        if dt == 0:
            return 0.0
        error = self.setpoint - current_value

        if abs(error) < deadzone:
            self.integral = 0
            self.previous_error = 0
            return 0.0

        self.integral += error * dt
        derivative = (error - self.previous_error) / dt

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        output = max(self.output_limits[0], min(self.output_limits[1], output))

        self.previous_error = error
        self.last_time = current_time
        return output

    def adjust_parameters(self, new_kp, new_ki, new_kd):
        self.kp = new_kp
        self.ki = new_ki
        self.kd = new_kd

    def dynamic_adjustment(self, environment_factor, movement_factor):
        # Adjust the PID parameters based on the environment and movement factors
        self.kp = self.kp * environment_factor
        self.ki = self.ki * movement_factor
        self.kd = self.kd * (environment_factor + movement_factor) / 2
