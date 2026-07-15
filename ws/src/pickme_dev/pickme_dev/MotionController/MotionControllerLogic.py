# Contains the logic for the Motioncontroller

ACCEL_MAX = 0.02            # in [m/s²]
ACCEL_MIN = 0               # in [m/s²]
ACCEL = 0.01                # in [m/s²]

class Controller:
    def __init__(self, kp=0, kd=0, Timebase=0):
        self.last_error = 0
        self.dt = Timebase
        self.kp = kp
        self.kd = kd
        self.accel = ACCEL

    def PDController(self, target: float, actual: float) -> float:
        """
        Implements a standard PD-Controller.
        First an error is calculated. The error is then independently
        multiplied with kp (P-Controller) as well as differentiated with
        the last error (P-Controller). The outputs are combined and then
        capped at the max acceleration values.
        
        :param target: Target position to be reached
        :type taget: float
        :param actual: Current position
        :type actual: float
        """

        accel = 0
        error = target - actual
    
        error_diff = (error - self.last_error) / self.dt
        self.last_error = error

        p_out = self.kp * error
        d_out = self.kd * error_diff

        combined = p_out + d_out

        if combined > self.accel:
            accel = self.accel
        elif combined < -self.accel:
            accel = -self.accel
        else:
            accel = combined
        
        return accel