import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import threading
import time
from ro45_portalrobot_interfaces.msg import RobotCmd, RobotPos, PredictedPos
from .MotionControllerLogic import Controller
from rclpy.duration import Duration

from std_msgs.msg import Bool

# ros2 topic pub --once /predicted_position ro45_portalrobot_interfaces/msg/PredictedPos "{x: -0.2, y: 0, obj_id: 1}"
# Timer Timebase in [s]
TIMEBASE = 0.1

# Toolcenterpoint offset in [m]
TCP_OFFSET_FROM_ENDSTOP_X = 0.060
TCP_OFFSET_FROM_ENDSTOP_Y = 0.013
TCP_OFFSET_FROM_ENDSTOP_Z = 0.250

# World Coordinate System offset in [m]
WCS_OFFSET_TO_TCP_X = -0.005
WCS_OFFSET_TO_TCP_Y =  0.065
WCS_OFFSET_TO_TCP_Z = -0.095

# Sorting bins coordinates expressed as offset from WCS in [m]
# Y-Values are slightly beyond negative endstops. This is not a bug and thus will not be fixed.
SORTING_BIN_UNICORN_X = -0.060
SORTING_BIN_UNICORN_Y = -0.170

SORTING_BIN_CAT_X = -0.150
SORTING_BIN_CAT_Y = -0.170

# XY-Margins
MARGIN_X = 0.005
MARGIN_Y = 0.005
MARGIN_Z = 0.005

# Pickheight conveyor
HEIGHT_ABOVE_CONVEYOR = -0.05
PICKHEIGHT_ABOVE_CONVEYOR = -0.005
IDLE_POS = 0

# IDs
UNICORN = 1
CAT = 2

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
        self.homepos_tcp_x = 0
        self.homepos_tcp_y = 0
        self.homepos_tcp_z = 0

        # Position in WCS
        self.wcs_rcs_offset_x = 0
        self.wcs_rcs_offset_y = 0
        self.wcs_rcs_offset_z = 0

        # Point / Data received via topic
        self.predicted_obj_pos_x = 0
        self.predicted_obj_pos_y = 0
        self.obj_id = 0
        self.detected_type = "CAT" # Possible states: "CAT", "UNICORN"
        self.external_debug_lock = threading.Lock()

        # PD-Controller and logic instance(s)
        self.controller_logic = Controller()
        self.controller_x = Controller(1, 3, TIMEBASE)
        self.controller_y = Controller(1, 3, TIMEBASE)
        self.controller_z = Controller(1, 3, TIMEBASE)

        # Should be self explanatory
        self.state = "IDLE" # Possible states: "IDLE", "PICK", "PLACE", "APPROACH" (APPROACH is just for debugging (so the z-axis doesn't move))
        self.init_complete = False
        self.new_object_lock = False


    def robot_pos_callback(self, msg: RobotPos):
        with self.robot_pos_lock:
            self.old_robot_x = self.robot_x
            self.old_robot_y = self.robot_y
            self.old_robot_z = self.robot_z

            self.robot_x = msg.pos_x
            self.robot_y = msg.pos_y
            self.robot_z = msg.pos_z

            self.delta_x = abs(self.old_robot_x - self.robot_x)
            self.delta_y = abs(self.old_robot_y - self.robot_y)
            self.delta_z = abs(self.old_robot_z - self.robot_z)
    

    def prediction_pos_callback(self, msg: PredictedPos):
        with self.external_debug_lock:
            if (msg.obj_id >= 0 and self.new_object_lock == False):
                self.predicted_obj_pos_x = msg.x
                self.predicted_obj_pos_y = msg.y
                self.obj_id = msg.obj_id
                self.state = "PICK"
                if (msg.obj_id == UNICORN):
                    self.detected_type = "UNICORN"
                elif (msg.obj_id == CAT):
                    self.detected_type = "CAT"

                # Debug for terminal
                print("\n---- Current Robot Position ----")
                print("Predicted X: ", self.predicted_obj_pos_x)
                print("Predicted Y: ", self.predicted_obj_pos_y)
                print("Tracking Obj: ", self.obj_id)
                print("       ---- DEBUG END ----      ")


    def PrimThread(self):
        """Primary thread that executes all the code"""

        if self.init_complete == False:
            self.timer.cancel()
            self.AccelerateAxis("X", 0.0)
            self.AccelerateAxis("Y", 0.0)
            self.AccelerateAxis("Z", 0.0)
            self.publisher_command.publish(self.cmd)
            self.DriveToHomePos()

            while not (self.delta_x == 0 and self.delta_y == 0 and self.delta_z == 0):
                time.sleep(1)
            
            print("\n", self.delta_x, "", abs(self.old_robot_x - self.robot_x), "\n", self.delta_y, "", abs(self.old_robot_y - self.robot_y), "\n", self.delta_z, "", abs(self.old_robot_z - self.robot_z))

            self.homepos_tcp_x = self.robot_x - TCP_OFFSET_FROM_ENDSTOP_X
            self.homepos_tcp_y = self.robot_y - TCP_OFFSET_FROM_ENDSTOP_Y
            self.homepos_tcp_z = self.robot_z + TCP_OFFSET_FROM_ENDSTOP_Z

            self.wcs_rcs_offset_x = WCS_OFFSET_TO_TCP_X - self.homepos_tcp_x
            self.wcs_rcs_offset_y = WCS_OFFSET_TO_TCP_Y - self.homepos_tcp_y
            self.wcs_rcs_offset_z = WCS_OFFSET_TO_TCP_Z - self.homepos_tcp_z

            self.init_complete = True
            self.state = "IDLE"
            self.timer.reset()

            # Debug
            print("Aktuelle Homepos TCP (X Y Z): ", self.homepos_tcp_x, " ", self.homepos_tcp_y, " ", self.homepos_tcp_z)
            print("Aktuelle Robopos (X Y Z):     ", self.robot_x, " ", self.robot_y, " ", self.robot_z)
            print("Repräsentation im WKS (X Y Z): ", self.wcs_rcs_offset_x, " ", self.wcs_rcs_offset_y, " ", self.wcs_rcs_offset_z)
            print("Homing Komplett")
        else:
            # Internal RCS expressed as WCS offset
            current_x = (self.robot_x - TCP_OFFSET_FROM_ENDSTOP_X) + self.wcs_rcs_offset_x
            current_y = (self.robot_y - TCP_OFFSET_FROM_ENDSTOP_Y) + self.wcs_rcs_offset_y
            current_z = (self.robot_z + TCP_OFFSET_FROM_ENDSTOP_Z) + self.wcs_rcs_offset_z

            match self.state:
                case "IDLE":
                    self.cmd.accel_x = self.controller_x.PDController(IDLE_POS, current_x)
                    self.cmd.accel_y = self.controller_y.PDController(IDLE_POS, current_y)
                    self.cmd.accel_z = self.controller_z.PDController(HEIGHT_ABOVE_CONVEYOR, current_z)
                    self.cmd.activate_gripper = False


                case "PICK":
                    self.cmd.accel_x = self.controller_x.PDController(self.predicted_obj_pos_x, current_x)
                    self.cmd.accel_y = self.controller_y.PDController(self.predicted_obj_pos_y, current_y)
                    self.cmd.accel_z = self.controller_z.PDController(PICKHEIGHT_ABOVE_CONVEYOR, current_z)
                    self.cmd.activate_gripper = True

                    if (current_z > PICKHEIGHT_ABOVE_CONVEYOR * 1.1):
                        self.new_object_lock = True
                        print("Picked something up. Next state: AFTERPICK")
                        self.state = "AFTERPICK"


                case "AFTERPICK":
                    self.cmd.accel_x = self.controller_x.PDController(current_x, current_x)
                    self.cmd.accel_y = self.controller_y.PDController(current_y, current_y)
                    self.cmd.accel_z = self.controller_z.PDController(HEIGHT_ABOVE_CONVEYOR, current_z)
                    self.cmd.activate_gripper = True

                    if (abs(HEIGHT_ABOVE_CONVEYOR - current_z) < MARGIN_Z):
                        print("AFTERPICK DONE")
                        self.state = "PLACE"
                

                case "PLACE":
                    match self.detected_type:
                        case "CAT":
                            self.cmd.accel_x = self.controller_x.PDController(SORTING_BIN_CAT_X, current_x)
                            self.cmd.accel_y = self.controller_y.PDController(SORTING_BIN_CAT_Y, current_y)
                            self.cmd.accel_z = self.controller_z.PDController(HEIGHT_ABOVE_CONVEYOR, current_z)
                            self.cmd.activate_gripper = True

                            if (abs(SORTING_BIN_CAT_X - current_x) < MARGIN_X) and (self.delta_y == 0):
                                self.cmd.activate_gripper = False
                                self.state = "IDLE"
                                self.new_object_lock = False

                        case "UNICORN":
                            self.cmd.accel_x = self.controller_x.PDController(SORTING_BIN_UNICORN_X, current_x)
                            self.cmd.accel_y = self.controller_y.PDController(SORTING_BIN_UNICORN_Y, current_y)
                            self.cmd.accel_z = self.controller_z.PDController(HEIGHT_ABOVE_CONVEYOR, current_z)
                            self.cmd.activate_gripper = True

                            if (abs(SORTING_BIN_UNICORN_X - current_x) < MARGIN_Z) and (self.delta_y == 0):
                                print("Pick and Placed UNICORN")
                                self.cmd.activate_gripper = False
                                self.state = "IDLE"
                                self.new_object_lock = False

            self.publisher_command.publish(self.cmd)

    
    def DriveToHomePos(self):
        """Logic that drives all axis to their homing position
        (positive Endstops)"""

        self.AccelerateAxis("X", self.controller_logic.accel)
        self.AccelerateAxis("Y", self.controller_logic.accel)
        self.AccelerateAxis("Z", -(self.controller_logic.accel))
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