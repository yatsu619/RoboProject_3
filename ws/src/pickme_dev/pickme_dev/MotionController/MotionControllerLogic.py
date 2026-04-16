# Contains the logic for the Motioncontroller

ACCEL_MAX = 0.1             # in [m/s²]
ACCEL_MIN = 0               # in [m/s²]
ACCEL = 0.05                # in [m/s²]
DIST_TOLERABLE_ERR = 0.001  # Allowable error in the distance traveled in [m]

class MotionControllerLogic:
    def __init__(self):
        self.state = 1                      # Current state of the statemachine
        self.error = DIST_TOLERABLE_ERR     # Errorvalue so it can be used elsewhere
        self.busy = False                   # Shows if a coordinate was received and is being executed
        self.accel = 0                      # Commanded Acceleration
        self.accel_min = ACCEL_MIN
        self.accel_max = ACCEL_MAX
        self.accel_avg = ACCEL
        self.accelerated_axis = [False, False, False]   # axis in [X, Y, Z]
    
    def move_to_point(self, distance: list) -> str:
        """
        ## Description
        Moves a selected axis a specified distance

        :param distance: Distance the selected axis should move
        :type list: list
        :return string: String of the axis that is moved
        """

        # X-Axis
        if ((abs(distance[0]) >= DIST_TOLERABLE_ERR) & (self.accelerated_axis[0] == False)):
            self.accelerated_axis[0] = True
            if (distance[0] < 0):
                self.accel = -ACCEL
            else:
                self.accel = ACCEL
            return "X"
        
        # Y-Axis
        if ((abs(distance[1]) >= DIST_TOLERABLE_ERR) & (self.accelerated_axis[1] == False)):
            self.accelerated_axis[1] = True
            if (distance[1] < 0):
                self.accel = -ACCEL
            else:
                self.accel = ACCEL
            return "Y"
        
        # Z-Axis
        if ((abs(distance[2]) >= DIST_TOLERABLE_ERR) & (self.accelerated_axis[2] == False)):
            self.accelerated_axis[2] = True
            if (distance[2] < 0):
                self.accel = -ACCEL
            else:
                self.accel = ACCEL
            return "Z"

