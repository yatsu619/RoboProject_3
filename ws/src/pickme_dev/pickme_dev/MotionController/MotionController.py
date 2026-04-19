import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import threading
import time
from ro45_portalrobot_interfaces.msg import RobotCmd, RobotPos, ExtDebug
from .MotionControllerLogic import MotionControllerLogic

from std_msgs.msg import Bool

TIMEBASE = 0.1    # Timebase for the timer and PrimThread [s]
TIMEBASE_ACCELERATION = 1

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

        self.timer = self.create_timer(TIMEBASE, self.PrimThread)
        
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

        self.controller = MotionControllerLogic()

    

    def external_debug_callback(self, msg: ExtDebug):
        with self.external_debug_lock:
            self.dist_x = msg.dist_x
            self.dist_y = msg.dist_y
            self.dist_z = msg.dist_z

            if (self.dist_x or self.dist_y or self.dist_z != 0):
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
            self.distance_left = [self.dist_x, self.dist_y, self.dist_z]
            self.controller.busy = True
            self.coords_ready = False
        
        # if coordinates are ready moves the robot.
        elif (self.controller.busy == True):
            axis = self.controller.move_to_point(self.distance_left)

            if ((self.controller.accelerated_axis[0]) & (axis == "X")):
                # Acceleration phase
                self.Accelerate("X", float(self.controller.accel))
                time.sleep(TIMEBASE_ACCELERATION)
                self.Accelerate("X", 0.0)
                
                # Constant velocity phase
                self.axis_speed[0] = self.controller.accel * TIMEBASE_ACCELERATION
                self.controller.accel = 0
                self.axis_time[0] = self.distance_left[0] / self.axis_speed[0]
                time.sleep(self.axis_time[0])
                
                # Decelleration phase
                self.Accelerate("X", float(self.controller.accel))
                time.sleep(TIMEBASE_ACCELERATION)
                self.Accelerate("X", 0.0)
                self.distance_left[0] = 0
            

            if ((self.controller.accelerated_axis[1]) & (axis == "Y")):
                # Acceleration phase
                self.Accelerate("Y", float(self.controller.accel))
                time.sleep(TIMEBASE_ACCELERATION)
                self.Accelerate("Y", 0.0)
                
                # Constant velocity phase
                self.axis_speed[1] = self.controller.accel * TIMEBASE_ACCELERATION
                self.controller.accel = 0
                self.axis_time[1] = self.distance_left[1] / self.axis_speed[1]
                time.sleep(self.axis_time[1])
                
                # Decelleration phase
                self.Accelerate("Y", float(self.controller.accel))
                time.sleep(TIMEBASE_ACCELERATION)
                self.Accelerate("Y", 0.0)
                self.distance_left[1] = 0
            

            if ((self.controller.accelerated_axis[2]) & (axis == "Z")):
                # Acceleration phase
                self.Accelerate("Z", float(self.controller.accel))
                time.sleep(TIMEBASE_ACCELERATION)
                self.Accelerate("Z", 0.0)
                
                # Constant velocity phase
                self.axis_speed[2] = self.controller.accel * TIMEBASE_ACCELERATION
                self.controller.accel = 0
                self.axis_time[2] = self.distance_left[2] / self.axis_speed[2]
                time.sleep(self.axis_time[2])
                
                # Decelleration phase
                self.Accelerate("Z", float(self.controller.accel))
                time.sleep(TIMEBASE_ACCELERATION)
                self.Accelerate("Z", 0.0)
                self.distance_left[2] = 0
            

            if ((self.distance_left[0] or self.distance_left[1] or self.distance_left[2]) == 0):
                print("Deaktiviere controller. Nichtmehr Busy")
                self.controller.busy = False
        
        # This Branch is only triggered if there are no coordinates ready and controller isn't Busy
        else:
            self.distance_left = [0, 0, 0]
            print("Motioncontroller inaktiv. Erwartet koordinaten über '/external_debug")
            print('verwende: ros2 topic pub --once /external_debug ro45_portalrobot_interfaces/msg/ExtDebug "{dist_x: 0.5, dist_y: 0.5, dist_z: 0.5}" um die achsen zu verfahren.\n')

    

    def Accelerate(self, axis: str, acceleration: float):
        match axis:
            case "X":
                self.cmd.accel_x = acceleration
                self.publisher_command.publish(self.cmd)
            case "Y":
                self.cmd.accel_y = acceleration
                self.publisher_command.publish(self.cmd)
            case "Z":
                self.cmd.accel_z = acceleration
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