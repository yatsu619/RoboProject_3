# Contains the logic for the Motioncontroller

ACCEL_MAX = 0.1             # in [m/s²]
ACCEL_MIN = 0               # in [m/s²]
ACCEL = 0.05                # in [m/s²]
DIST_TOLERABLE_ERR = 0.001  # Allowable error in the distance traveled in [m]


class MotionControllerLogic:
    def __init__(self):
        self.state = 1                      # Current state of the statemachine
        self.error_dist = DIST_TOLERABLE_ERR     # Errorvalue so it can be used elsewhere
        self.busy = False                   # Shows if a coordinate was received and is being executed
        self.accel = 0                      # Commanded Acceleration
        self.accel_min = ACCEL_MIN
        self.accel_max = ACCEL_MAX
        self.accel_avg = ACCEL
        self.accelerated_axis = [False, False, False]   # axis in [X, Y, Z]
        self.last_error = 0
        self.last_robot_x = 0
        self.last_velocity_x = 0
        self.p_out = 0
        self.d_out = 0
        self.controller_error = 0

class Controller:
    def __init__(self):
        self.last_error = 0

    def PDController(self, target: float, actual: float, kp: float, kd: float, dt: float) -> float:
        accel = 0
        error = target - actual
    
        error_diff = (error - self.last_error) / dt
        self.last_error = error

        p_out = kp * error
        d_out = kd * error_diff

        combined = p_out + d_out

        if combined > ACCEL:
            accel = ACCEL
        elif combined < -ACCEL:
            accel = -ACCEL
        else:
            accel = combined
        
        return accel