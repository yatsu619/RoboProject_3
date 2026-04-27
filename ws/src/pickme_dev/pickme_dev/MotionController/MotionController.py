import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import threading
import time
from ro45_portalrobot_interfaces.msg import RobotCmd, RobotPos, ExtDebug
from .MotionControllerLogic import MotionControllerLogic, Controller
from rclpy.duration import Duration

from std_msgs.msg import Bool

TIMEBASE = 0.1    # Timebase for the timer and PrimThread [s]

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

        """
        self.subscriber_movetowaypoint = self.create_subscription(
            interface,
            topic,
            10
        )
        """

        self.subscriber_extdebug = self.create_subscription(
            ExtDebug,
            '/external_debug',
            self.external_debug_callback,
            10
        )

        self.timer = self.create_timer(TIMEBASE, self.PrimThread, callback_group=ReentrantCallbackGroup())
        
        self.cmd = RobotCmd()
        
        self.coords_ready = False

        self.robot_x = 0
        self.robot_y = 0
        self.robot_z = 0
        self.robot_pos_lock = threading.Lock()

        self.dist_x = 0
        self.dist_y = 0
        self.dist_z = 0
        self.external_debug_lock = threading.Lock()

        self.axis_speed = [0, 0, 0]
        self.axis_time = [0, 0, 0]

        self.last_distance = 0
        self.distance_left = [0, 0, 0]

        self.controller_logic = MotionControllerLogic()
        self.controller_x = Controller()
        self.controller_y = Controller()
        self.controller_z = Controller()
    

    def external_debug_callback(self, msg: ExtDebug):
        with self.external_debug_lock:
            self.dist_x = msg.dist_x
            self.dist_y = msg.dist_y
            self.dist_z = msg.dist_z

            print(self.dist_x)
            print(self.dist_x)
            print(self.dist_x)


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


    def PrimThread(self):
        """Primary thread that executes all the code"""

        self.cmd.accel_x = self.controller_x.PDController(self.dist_x, self.robot_x, 1, 2, TIMEBASE)
        self.cmd.accel_y = self.controller_y.PDController(self.dist_y, self.robot_y, 1, 2, TIMEBASE)
        self.cmd.accel_z = self.controller_z.PDController(self.dist_z, self.robot_z, 1, 2, TIMEBASE)
        self.publisher_command.publish(self.cmd)


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