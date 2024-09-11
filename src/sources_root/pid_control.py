class PID:
    def __init__(self, kp=0, ki=0, kd=0, max_i_term=None, max_output=None) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_term = 0.0
        self.error = 0.0
        self.last_error = 0.0
        self.max_i_term = max_i_term
        self.max_output = max_output

    def update(self, feedback: float, set_point: float) -> float:
        self.last_error = self.error
        self.error = feedback - set_point

        p_term = self.kp * self.error
        self.i_term += self.ki * self.error
        d_term = self.kd * (self.error - self.last_error)

        if self.max_i_term is not None:
            if self.i_term > self.max_i_term:
                self.i_term = self.max_i_term
            elif self.i_term < -self.max_i_term:
                self.i_term= -self.max_i_term

        output = p_term + self.i_term + d_term
        output = round(output, 3)

        if self.max_output is not None:
            if output > self.max_output:
                output = self.max_output
            elif output < -self.max_output:
                output = -self.max_output

        return output
