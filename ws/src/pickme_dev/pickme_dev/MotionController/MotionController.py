import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import threading
import time
from ro45_portalrobot_interfaces.msg import RobotCmd, RobotPos, ExtDebug
from .MotionControllerLogic import MotionControllerLogic

from std_msgs.msg import Bool

TIMER_TIMEBASE = 0.1    # Timebase for the timer and PrimThread [s]

class MotionControllerNode(Node):
    def __init__(self):
        super().__init__('motioncontroller_node')

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

        self.timer = self.create_timer(TIMER_TIMEBASE, self.PrimThread)
        
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

        self.controller = MotionControllerLogic()
    
    def external_debug_callback(self, msg: ExtDebug):
        with self.external_debug_lock:
            self.dist_x = msg.dist_x
            self.dist_y = msg.dist_y
            self.dist_z = msg.dist_z

            if (self.dist_x | self.dist_y | self.dist_z != 0):
                if (self.controller.busy == False):
                    self.coords_ready = True

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

        # Only receive new coords if controller isn't Busy and if transmitted distance is > 0.
        if ((self.coords_ready == True) & (self.controller.busy == False)):
            self.controller.accelerated_axis = [False, False, False]
            distance_left = [self.dist_x, self.dist_y, self.dist_z]
            self.controller.busy = True
        
        # if coordinates are ready moves the robot.
        elif (self.controller.busy == True):
            axis = self.controller.move_to_point(distance_left)

            if ((self.controller.accelerated_axis[0]) & (axis == "X")):
                self.cmd.accel_x = self.controller.accel
                self.publisher_command.publish(self.cmd.accel_x)
                time.sleep(TIMER_TIMEBASE)
                self.publisher_command.publish(-(self.cmd.accel_x))
                self.axis_speed[0] = self.controller.accel * TIMER_TIMEBASE
                self.controller.accel = 0
                self.axis_time[0] = distance_left[0] / self.axis_speed[0]
                time.sleep(self.axis_time[0])
                self.publisher_command.publish(-(self.cmd.accel_x))
                time.sleep(TIMER_TIMEBASE)
                self.publisher_command.publish(self.cmd.accel_x) 
            
            if ((self.controller.accelerated_axis[1]) & (axis == "Y")):
                self.cmd.accel_y = self.controller.accel
                self.publisher_command.publish(self.cmd.accel_y)
                time.sleep(TIMER_TIMEBASE)
                self.publisher_command.publish(-(self.cmd.accel_y))
                self.axis_speed[1] = self.controller.accel * TIMER_TIMEBASE
                self.controller.accel = 0
                self.axis_time[1] = distance_left[1] / self.axis_speed[1]
                time.sleep(self.axis_time[1])
                self.publisher_command.publish(-(self.cmd.accel_y))
                time.sleep(TIMER_TIMEBASE)
                self.publisher_command.publish(self.cmd.accel_y)
            
            if ((self.controller.accelerated_axis[2]) & (axis == "Z")):
                self.cmd.accel_z = self.controller.accel
                self.publisher_command.publish(self.cmd.accel_z)
                time.sleep(TIMER_TIMEBASE)
                self.publisher_command.publish(-(self.cmd.accel_z))
                self.axis_speed[2] = self.controller.accel * TIMER_TIMEBASE
                self.controller.accel = 0
                self.axis_time[2] = distance_left[2] / self.axis_speed[2]
                time.sleep(self.axis_time[2])
                self.publisher_command.publish(-(self.cmd.accel_z))
                time.sleep(TIMER_TIMEBASE)
                self.publisher_command.publish(self.cmd.accel_z)
            
            if ((self.controller.accelerated_axis[0] & self.controller.accelerated_axis[1] & self.controller.accelerated_axis[2]) == True):
                self.controller.busy = False
        
        # This Branch is only triggered if there are no coordinates ready and controller isn't Busy
        else:
            print("GRRR. MotionController Arbeitslos. Lifestyle Teilzeit können wir uns nicht mehr leisten. GRRR")



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