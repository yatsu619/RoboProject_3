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
WCS_OFFSET_X = 0
WCS_OFFSET_Y = 0
WCS_OFFSET_Z = 0

# X & Y Tolerances in [m]
XY_TOLERANCE = 0.001


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

        # Point received via topic
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.obj_id = 0
        self.external_debug_lock = threading.Lock()

        # First Point that was received with obj_id
        self.first_obj_id = -1
        self.target_x = 0.0
        self.target_y = 0.0
        self.state = "IDLE" # Possible "IDLE", "APPROACH", "DESCEND"


        # PD-Controller instances
        self.controller_logic = MotionControllerLogic()
        self.controller_x = Controller()
        self.controller_y = Controller()
        self.controller_z = Controller()

        # Self explanatory
        self.init_complete = False
    
    # Remove upon final release
    def external_debug_callback(self, msg: ExtDebug):
        with self.external_debug_lock:
            self.pos_x = msg.dist_x
            self.pos_y = msg.dist_y
            self.pos_z = msg.dist_z


    def robot_pos_callback(self, msg: RobotPos):
        with self.robot_pos_lock:
            self.robot_x = msg.pos_x
            self.robot_y = msg.pos_y
            self.robot_z = msg.pos_z
            
            # Debug for terminal
            print("\n---- Current Robot Position ----")
            print("Position X: ", self.robot_x)
            print("Position Y: ", self.robot_y)
            print("Position Z: ", self.robot_z)
            print("       ---- DEBUG END ----      ")
    

    def prediction_pos_callback(self, msg: PredictedPos):
        with self.external_debug_lock:
            if msg.obj_id >= 0 and self.state == "IDLE":
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
            self.cmd.activate_gripper = False
            self.publisher_command.publish(self.cmd)
            self.DriveToHomePos()

            # Change to dynamically calculated sleep
            time.sleep(20)
            
            self.homepos_x = self.robot_x + TCP_OFFSET_X
            self.homepos_y = self.robot_y + TCP_OFFSET_Y
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
            print("Aktuelle position im Robointernen system (X Y Z): ", (self.robot_x - self.wcs_pos_x), " ", (self.robot_y - self.wcs_pos_y), " ", (self.robot_z - self.wcs_pos_z))
            #print("[WARNUNG] NACHFOLGENDE REGELUNG AKTUELL DEAKTIVIERT! TIMER FÜR 'PrimThread()' DEAKTIVIERT! [WARNUNG]")
            #while True:
            #    self.timer.cancel()
            #    time.sleep(1)
        else:
            # Current position to represent in Robot internal coordinate system
            current_x = self.robot_x - self.wcs_pos_x
            current_y = self.robot_y - self.wcs_pos_y
            current_z = self.robot_z - self.wcs_pos_z

            match self.state:
                case "IDLE":
                    self.cmd.accel_x = self.controller_x.PDController(current_x, current_x, 1, 2, TIMEBASE)
                    self.cmd.accel_y = self.controller_y.PDController(current_y, current_y, 1, 2, TIMEBASE)
                    self.cmd.accel_z = self.controller_z.PDController(current_z, current_z, 1, 2, TIMEBASE)
                    self.cmd.activate_gripper = False

                case "APPROACH":
                    self.cmd.accel_x = self.controller_x.PDController(self.pos_x, current_x, 1, 2, TIMEBASE)
                    self.cmd.accel_y = self.controller_y.PDController(self.pos_y, current_y, 1, 2, TIMEBASE)
                    self.cmd.accel_z = self.controller_z.PDController(self.pos_z, current_z, 1, 2, TIMEBASE)
                    self.cmd.activate_gripper = False

                    error_x = abs(self.target_x - current_x)
                    error_y = abs(self.target_y - current_y)
                    if (error_x < XY_TOLERANCE) and (error_y < XY_TOLERANCE):
                        self.state = "DESCEND"

                case "DESCEND":
                    self.cmd.accel_x = self.controller_x.PDController(current_x, current_x, 1, 2, TIMEBASE)
                    self.cmd.accel_y = self.controller_y.PDController(current_y, current_y, 1, 2, TIMEBASE)
                    self.cmd.accel_z = self.controller_z.PDController(  0.01  , current_z, 1, 2, TIMEBASE)
                    self.cmd.activate_gripper = True

                    if self.pos_z > 0.01:
                        print("Done")
                        self.state = "IDLE"

            self.publisher_command.publish(self.cmd)

    
    def DriveToHomePos(self):
        """Logic that drives all axis to their homing position
        (Endstops)"""

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
        """Wrapper to accelerate axis for 1 second. Used in
        "DriveToHomePos" only and to initialize acceleration
        to 0 in the beginning."""

        if axis == "X":
            self.cmd.accel_x = accel
        elif axis == "Y":
            self.cmd.accel_y = accel
        elif axis == "Z":
            self.cmd.accel_z = accel


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