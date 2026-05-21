import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import threading
import time
from ro45_portalrobot_interfaces.msg import RobotCmd, RobotPos, ExtDebug, PredictedPos
from .MotionControllerLogic import MotionControllerLogic, Controller
from rclpy.duration import Duration

from std_msgs.msg import Bool

# Timer Timebase in [s]
TIMEBASE = 0.1

# Toolcenterpoint offset in [m]
TCP_OFFSET_X = 0.060
TCP_OFFSET_Y = 0.013
TCP_OFFSET_Z = 0.250

# World Coordinate System offset in [m]
WCS_OFFSET_X = 0.005
WCS_OFFSET_Y = 0.065
WCS_OFFSET_Z = 0.095

# Sorting bins coordinates expressed as offset from WCS in [m]
# Y-Values are slightly beyond negative endstops. This is not a bug and thus will not be fixed.
SORTING_BIN_UNICORN_X = -0.008
SORTING_BIN_UNICORN_Y = -0.170

SORTING_BIN_CAT_X = -0.017
SORTING_BIN_CAT_Y = -0.170

# XY-Margin
MARGIN_X = 0.01
MARGIN_Y = 0.01

class MotionControllerNode(Node):
    def __init__(self):
        super().__init__('motioncontroller_node')
        self.callback_group = ReentrantCallbackGroup()

        self.publisher_command = self.create_publisher(
            RobotCmd,
            '/robot_command',
            10
        )

        self.subscriber_position = self.create_subscription(
            RobotPos,
            '/robot_position',
            self.robot_pos_callback,
            10
        )

        self.subscriber_prediction = self.create_subscription(
            PredictedPos,
            '/predicted_position',
            self.prediction_pos_callback,
            10
        )

        self.subscriber_extdebug = self.create_subscription(
            ExtDebug,
            '/external_debug',
            self.external_debug_callback,
            10
        )

        # Timed loop for Primary code
        self.timer = self.create_timer(TIMEBASE, self.PrimThread, callback_group=ReentrantCallbackGroup())
        
        # Message instance
        self.cmd = RobotCmd()

        # Current robot position transmitted by robot
        self.robot_x = 0
        self.robot_y = 0
        self.robot_z = 0

        self.old_robot_x = 0
        self.old_robot_y = 0
        self.old_robot_z = 0

        self.delta_x = 0
        self.delta_y = 0
        self.delta_z = 0

        self.robot_pos_lock = threading.Lock()

        # Current homing position
        # when Endstops are reached, the then current position becomes an offset added to homepos
        self.homepos_x = 0
        self.homepos_y = 0
        self.homepos_z = 0

        # Position in WCS
        self.wcs_pos_x = 0
        self.wcs_pos_y = 0
        self.wcs_pos_z = 0

        # Point / Data received via topic
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.obj_id = 0
        self.detected_type = "CAT" # Possible states: "CAT", "UNICORN"
        self.external_debug_lock = threading.Lock()

        # PD-Controller and logic instance(s)
        self.controller_logic = MotionControllerLogic()
        self.controller_x = Controller()
        self.controller_y = Controller()
        self.controller_z = Controller()

        # Should be self explanatory
        self.state = "IDLE" # Possible states: "IDLE", "PICK", "PLACE", "APPROACH" (APPROACH is just for debugging (so the z-axis doesn't move))
        self.init_complete = False
        self.pick_and_place_complete = False


    def robot_pos_callback(self, msg: RobotPos):
        with self.robot_pos_lock:
            self.old_robot_x = robot_x
            self.old_robot_y = robot_y
            self.old_robot_z = robot_z

            self.robot_x = msg.pos_x
            self.robot_y = msg.pos_y
            self.robot_z = msg.pos_z

            self.delta_x = abs(self.old_robot_x - self.robot_x)
            self.delta_y = abs(self.old_robot_y - self.robot_y)
            self.delta_z = abs(self.old_robot_z - self.robot_z)
            
            # Debug for terminal
            print("\n---- Current Robot Position ----")
            print("RCS (X Y Z)      : ", self.robot_x, " ", self.robot_y, " ", self.robot_z)
            print("old RCS (X Y Z)  : ", self.old_robot_x, " ", self.old_robot_y, " ", self.old_robot_z)
            print("Delta (X Y Z)    : ", self.delta_x, " ", self.delta_y, " ", self.delta_z)
            print("       ---- DEBUG END ----      ")
    

    def prediction_pos_callback(self, msg: PredictedPos):
        with self.external_debug_lock:
            if (msg.obj_id >= 0) and (msg.obj_id == self.obj_id):
                self.pos_x = msg.x
                self.pos_y = msg.y
                self.pos_z = msg.z
                self.obj_id = msg.obj_id
                self.state = "APPROACH"

                # Debug for terminal
                print("\n---- Current Robot Position ----")
                print("Predicted X: ", self.pos_x)
                print("Predicted Y: ", self.pos_y)
                print("Predicted Z: ", self.pos_z)
                print("Tracking Obj: ", self.obj_id)
                print("       ---- DEBUG END ----      ")


    def PrimThread(self):
        """Primary thread that executes all the code"""

        if self.init_complete == False:
            self.AccelerateAxis("X", 0.0)
            self.AccelerateAxis("Y", 0.0)
            self.AccelerateAxis("Z", 0.0)
            self.publisher_command.publish(self.cmd)
            self.DriveToHomePos()

            while not (delta_x and delta_y and delta_z) == 0:
                time.sleep(1)
            
            self.homepos_x = self.robot_x - TCP_OFFSET_X
            self.homepos_y = self.robot_y - TCP_OFFSET_Y
            self.homepos_z = self.robot_z - TCP_OFFSET_Z

            self.wcs_pos_x = self.homepos_x - WCS_OFFSET_X
            self.wcs_pos_y = self.homepos_y - WCS_OFFSET_Y
            self.wcs_pos_z = self.homepos_z - WCS_OFFSET_Z

            self.init_complete = True

            # Debug
            print("Aktuelle Homepos TCP (X Y Z): ", self.homepos_x, " ", self.homepos_y, " ", self.homepos_z)
            print("Aktuelle Robopos (X Y Z):     ", self.robot_x, " ", self.robot_y, " ", self.robot_z)
            print("Repräsentation im WKS (X Y Z): ", self.wcs_pos_x, " ", self.wcs_pos_y, " ", self.wcs_pos_z)
            print("Homing Komplett")
        else:
            # Internal RCS expressed as WCS offset
            current_x = self.robot_x - self.wcs_pos_x
            current_y = self.robot_y - self.wcs_pos_y
            current_z = self.robot_z - self.wcs_pos_z

            match self.state:
                case "IDLE":
                    # Robot does nothing and holds the current position
                    self.cmd.accel_x = self.controller_x.PDController(current_x, current_x, 1, 3, TIMEBASE)
                    self.cmd.accel_y = self.controller_y.PDController(current_y, current_y, 1, 3, TIMEBASE)
                    self.cmd.accel_z = self.controller_z.PDController(self.homepos_z, self.homepos_z, 1, 3, TIMEBASE)
                    self.cmd.activate_gripper = False

                case "PICK":
                    # Robot picks up object from conveyor belt
                    self.cmd.accel_x = self.controller_x.PDController(self.pos_x, current_x, 1, 3, TIMEBASE)
                    self.cmd.accel_y = self.controller_y.PDController(self.pos_y, current_y, 1, 3, TIMEBASE)
                    self.cmd.accel_z = self.controller_z.PDController(0.005, current_z, 1, 3, TIMEBASE)
                    self.cmd.activate_gripper = True
                
                case "PLACE":
                    # Robot (hopefully) has picked up an object and drops it into a bin
                    match self.detected_type:
                        case "CAT":
                            self.cmd.accel_x = self.controller_x.PDController(SORTING_BIN_CAT_X, current_x, 1, 3, TIMEBASE)
                            self.cmd.accel_y = self.controller_y.PDController(SORTING_BIN_CAT_Y, current_y, 1, 3, TIMEBASE)
                            self.cmd.accel_z = self.controller_z.PDController(self.homepos_z, self.homepos_z, 1, 3, TIMEBASE)
                            self.cmd.activate_gripper = True

                        case "UNICORN":
                            self.cmd.accel_x = self.controller_x.PDController(SORTING_BIN_UNICORN_X, current_x, 1, 3, TIMEBASE)
                            self.cmd.accel_y = self.controller_y.PDController(SORTING_BIN_UNICORN_Y, current_y, 1, 3, TIMEBASE)
                            self.cmd.accel_z = self.controller_z.PDController(self.homepos_z, self.homepos_z, 1, 3, TIMEBASE)
                            self.cmd.activate_gripper = True

                        case _:
                            print("Detected object is not valid")

                    if (abs(current_x - SORTING_BIN_CAT_X) < MARGIN_X) and (self.delta_y == 0):
                        self.cmd.activate_gripper = False
                        self.state = "IDLE"
                    if (abs(current_x - SORTING_BIN_UNICORN_X) < MARGIN_X) and (self.delta_y == 0):
                        self.cmd.activate_gripper = False
                        self.state = "IDLE"

                case "APPROACH":
                    # Robot approaches a point expressed in WCS form (just X and Y, as Z=0 is the conveyor belt itself)  -  DEBUG only!
                    self.cmd.accel_x = self.controller_x.PDController(self.pos_x, current_x, 1, 3, TIMEBASE)
                    self.cmd.accel_y = self.controller_y.PDController(self.pos_y, current_y, 1, 3, TIMEBASE)
                    self.cmd.accel_z = self.controller_z.PDController(current_z, current_z, 1, 3, TIMEBASE)
                    self.cmd.activate_gripper = False


            self.publisher_command.publish(self.cmd)

    
    def DriveToHomePos(self):
        """Logic that drives all axis to their homing position
        (positive Endstops)"""

        self.AccelerateAxis("X", self.controller_logic.accel_avg)
        self.AccelerateAxis("Y", self.controller_logic.accel_avg)
        self.AccelerateAxis("Z", -(self.controller_logic.accel_avg))
        self.publisher_command.publish(self.cmd)
        time.sleep(1)
        self.AccelerateAxis("X", 0.0)
        self.AccelerateAxis("Y", 0.0)
        self.AccelerateAxis("Z", 0.0)
        self.publisher_command.publish(self.cmd)

    def AccelerateAxis(self, axis: str, accel: int) -> None:
        """Sets the values of the ROS2 message interface 'RobotCmd'.
        Only used in 'DriveToHomePos' and 'PrimThread' as initialization."""

        if axis == "X":
            self.cmd.accel_x = accel
        elif axis == "Y":
            self.cmd.accel_y = accel
        elif axis == "Z":
            self.cmd.accel_z = accel
        self.cmd.activate_gripper = False


def main():
    rclpy.init()
    try:
        motioncontroller_node = MotionControllerNode()
        multithread_executor = MultiThreadedExecutor()
        rclpy.spin(motioncontroller_node, executor=multithread_executor)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()