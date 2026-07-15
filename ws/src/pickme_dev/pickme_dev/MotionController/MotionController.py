import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import threading
import time
from ro45_portalrobot_interfaces.msg import RobotCmd, RobotPos, PredictedPos
from .MotionControllerLogic import Controller

TIMER_TIMEBASE = 0.1

TCP_OFFSET_FROM_ENDSTOP_X = 0.060
TCP_OFFSET_FROM_ENDSTOP_Y = 0.013
TCP_OFFSET_FROM_ENDSTOP_Z = 0.250

WCS_OFFSET_TO_TCP_X = -0.005
WCS_OFFSET_TO_TCP_Y =  0.065
WCS_OFFSET_TO_TCP_Z = -0.095

SORTING_BIN_UNICORN_X = -0.100
SORTING_BIN_UNICORN_Y = -0.040

SORTING_BIN_CAT_X = -0.200
SORTING_BIN_CAT_Y = -0.040

MARGIN_X = 0.01
MARGIN_Y = 0.01
MARGIN_Z = 0.01

HEIGHT_ABOVE_CONVEYOR = -0.04
PICKHEIGHT_ABOVE_CONVEYOR = -0.0010
IDLE_POS_X = 0
IDLE_POS_Y = 0

UNICORN = 1
CAT = 2

ROBOT_GEARING_OFFSET_FACTOR_X = 1.35

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

        self.timer = self.create_timer(TIMER_TIMEBASE, self.PrimThread, callback_group=ReentrantCallbackGroup())
        
        self.cmd = RobotCmd()

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

        self.homepos_tcp_x = 0
        self.homepos_tcp_y = 0
        self.homepos_tcp_z = 0

        self.wcs_rcs_offset_x = 0
        self.wcs_rcs_offset_y = 0
        self.wcs_rcs_offset_z = 0

        self.predicted_obj_pos_x = 0
        self.predicted_obj_pos_y = 0
        self.obj_id = 0
        self.detected_type = "CAT" # Possible states: "CAT", "UNICORN"
        self.external_debug_lock = threading.Lock()

        self.controller_logic = Controller()
        self.controller_x = Controller(1, 3, TIMER_TIMEBASE)
        self.controller_y = Controller(1, 3, TIMER_TIMEBASE)
        self.controller_z = Controller(1, 3, TIMER_TIMEBASE)

        self.state = "IDLE" # Possible states: "IDLE", "PICK", "PLACE", "APPROACH" (APPROACH is just for debugging (so the z-axis doesn't move))
        self.init_complete = False
        self.new_object_lock = True


    def robot_pos_callback(self, msg: RobotPos):
        """
        Positional data as transmitted by Robot.
        Takes transmitted data from 'msg.pos' and calculates a delta to
        determine if axis movement is zero.
        """

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
        """
        Predicted positions of an object as calculated by the
        WaypointPredictionEngine.
        The transmitted 'obj_id' determines what kind of object it is:
        '0' indicates an unknown / unsupported object
        '1' indicates a Unicorn
        '2' indicates a Cat
        Only objects with an id greater than 0 are accepted and an
        object mutex 'new_object_lock' prevents a new object from
        interfering with the Pick&Place process.
        """

        with self.external_debug_lock:
            if (msg.obj_id > 0 and self.new_object_lock == False):
                self.predicted_obj_pos_x = msg.x
                self.predicted_obj_pos_y = msg.y
                self.obj_id = msg.obj_id
                self.state = "PICK"
                if (msg.obj_id == UNICORN):
                    self.detected_type = "UNICORN"
                elif (msg.obj_id == CAT):
                    self.detected_type = "CAT"


    def PrimThread(self):
        """
        Primary functions that executes the logic.
        First the robot is initialized by driving to the axis endstops.
        Coordinate Transformations are being done and then the Pick and
        Place logic is executed.
        """

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
            self.new_object_lock = False
            self.state = "IDLE"
            self.timer.reset()
        else:
            current_x = ((self.robot_x - TCP_OFFSET_FROM_ENDSTOP_X) + self.wcs_rcs_offset_x) * ROBOT_GEARING_OFFSET_FACTOR_X
            current_y = (self.robot_y - TCP_OFFSET_FROM_ENDSTOP_Y) + self.wcs_rcs_offset_y
            current_z = (self.robot_z + TCP_OFFSET_FROM_ENDSTOP_Z) + self.wcs_rcs_offset_z

            match self.state:
                case "IDLE":
                    self.MoveRobot(
                        target_pos_x=IDLE_POS_X,
                        target_pos_y=IDLE_POS_Y,
                        target_pos_z=HEIGHT_ABOVE_CONVEYOR,
                        current_pos=[current_x, current_y, current_z],
                        gripper=False
                        )


                case "PICK":
                    self.MoveRobot(
                        target_pos_x=self.predicted_obj_pos_x,
                        target_pos_y=self.predicted_obj_pos_y,
                        target_pos_z=PICKHEIGHT_ABOVE_CONVEYOR,
                        current_pos=[current_x, current_y, current_z],
                        gripper=False
                        )

                    if (current_z > PICKHEIGHT_ABOVE_CONVEYOR * 1.1):
                        self.activate_gripper = True
                        self.new_object_lock = True
                        print("Picked something up. Next state: AFTERPICK")
                        self.state = "AFTERPICK"


                case "AFTERPICK":
                    self.MoveRobot(
                        target_pos_x=IDLE_POS_X,
                        target_pos_y=current_y,
                        target_pos_z=HEIGHT_ABOVE_CONVEYOR,
                        current_pos=[current_x, current_y, current_z],
                        gripper=True
                        )

                    if (abs(HEIGHT_ABOVE_CONVEYOR - current_z) < MARGIN_Z):
                        print("AFTERPICK DONE")
                        self.state = "PLACE"
                

                case "PLACE":
                    match self.detected_type:
                        case "CAT":
                            self.MoveRobot(
                                target_pos_x=SORTING_BIN_CAT_X,
                                target_pos_y=SORTING_BIN_CAT_Y,
                                target_pos_z=HEIGHT_ABOVE_CONVEYOR,
                                current_pos=[current_x, current_y, current_z],
                                gripper=True
                                )

                            if (abs(SORTING_BIN_CAT_X - current_x) < MARGIN_X) and (abs(SORTING_BIN_CAT_Y - current_y) < MARGIN_Y):
                                print("Pick and Placed CAT")
                                self.cmd.activate_gripper = False
                                self.state = "IDLE"
                                self.new_object_lock = False

                        case "UNICORN":
                            self.MoveRobot(
                                target_pos_x=SORTING_BIN_UNICORN_X,
                                target_pos_y=SORTING_BIN_UNICORN_Y,
                                target_pos_z=HEIGHT_ABOVE_CONVEYOR,
                                current_pos=[current_x, current_y, current_z],
                                gripper=True
                                )

                            if (abs(SORTING_BIN_UNICORN_X - current_x) < MARGIN_X) and (abs(SORTING_BIN_UNICORN_Y - current_y) < MARGIN_Y):
                                print("Pick and Placed UNICORN")
                                self.cmd.activate_gripper = False
                                self.state = "IDLE"
                                self.new_object_lock = False

            self.publisher_command.publish(self.cmd)

    
    def DriveToHomePos(self) -> None:
        """
        Wrapper for Commanding a short burst of acceleration to drive the
        axis home (towards endstops).
        """

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
        """
        Writes acceleration data to ROS2 'RobotCmd' interface.
        
        :param axis: Axis to be commanded
        :type axis: str
        :param accel: Acceleration value to be commanded
        :type accel: int
        """

        if axis == "X":
            self.cmd.accel_x = accel
        elif axis == "Y":
            self.cmd.accel_y = accel
        elif axis == "Z":
            self.cmd.accel_z = accel
        self.cmd.activate_gripper = False
    

    def MoveRobot(self, target_pos_x: float, target_pos_y: float, target_pos_z: float, current_pos: list, gripper: bool) -> None:
        """
        Wrapper function for PD-Controllers.
        
        :param target_pos_x: Target position of x-axis
        :type target_pos_x: float
        :param target_pos_y: Target position of y-axis
        :type target_pos_y: float
        :param target_pos_z: Target position of z-axis
        :type target_pos_z: float
        :param current_pos: Current Position in the following form: [pos_x, pos_y, pos_z]
        :type current_pos: list
        :param gripper: Gripper should be active or inactive
        :type gripper: bool
        """

        self.cmd.accel_x = self.controller_x.PDController(target_pos_x, current_pos[0])
        self.cmd.accel_y = self.controller_y.PDController(target_pos_y, current_pos[1])
        self.cmd.accel_z = self.controller_z.PDController(target_pos_z, current_pos[2])
        self.cmd.activate_gripper = gripper

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