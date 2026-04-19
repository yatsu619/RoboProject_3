
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import threading
from threading import Thread
import time
import numpy as np
from ro45_portalrobot_interfaces.msg import RobotCmd, RobotPos, ExtDebug
from .MotionControllerLogic import MotionControllerLogic
import matplotlib.pyplot as plt

TIMEBASE = 0.02 # 50Hz


class MotionControllerSimNode(Node):
    def __init__(self):
        super().__init__('motioncontroller_simulator_node')

        self.subscriber_command = self.create_subscription(
            RobotCmd,
            '/robot_command',
            self.command_callback,
            10
        )

        self.subscriber_extdebug = self.create_subscription(
            ExtDebug,
            '/external_debug',
            self.external_debug_callback,
            10
        )

        self.timer = self.create_timer(TIMEBASE, self.PrimThread)
        
        self.command_lock = threading.Lock()

        self.start = False
        self.start_time = 0
        self.sim_done_time = 0
        self.done = False
        self.sim_done = False
        self.latest_accel = 0
        self.coordinate = 0

        self.points_x = []
        self.points_y = []

        self.loops = 0

    def external_debug_callback(self, msg: ExtDebug):
        dista_x = msg.dist_x
        dista_y = msg.dist_y
        dista_z = msg.dist_z

        if ((self.start == False) and (dista_x or dista_y or dista_z) > 0):
            self.start = True
            self.start_time = self.points_x[-1]
            self.coordinate = dista_x


    def command_callback(self, msg: RobotCmd):
        with self.command_lock:
            self.latest_accel = abs(msg.accel_x)

            if (msg.activate_gripper == True):
                self.done = True


    def PrimThread(self):
        self.points_y.append(self.latest_accel)
        self.points_x.append((self.loops + 1))
        
        self.loops += 1
        
        if (self.done == True):
            self.timer.cancel()
            self.sim_done_time = self.points_x[-1]
            self.sim_done = True

    def plotfnc(self):
        # Recalculate sampling to timer timebase
        for i, value in enumerate(self.points_x):
            self.points_x[i] = value * TIMEBASE
        self.start_time = self.start_time * TIMEBASE
        self.sim_done_time = self.sim_done_time * TIMEBASE

        # Acceleration plot
        plt.subplot(3, 1, 1)
        plt.title("Acceleration plot")
        plt.plot(np.array(self.points_x), np.array(self.points_y), '-')
        plt.ylabel("Acceleration [m/s²]")
        plt.xlabel("Time [s]")
        plt.axvline(self.start_time, color = 'b', label = 'Movement Begin')
        plt.axvline(self.sim_done_time, color = 'r', label = 'Movement End')
        plt.legend()
        plt.grid()

        """
        # Speed plot
        plt.subplot(3, 1, 2)
        plt.title("Speed plot")
        #plt.plot(xpoints, ypoints, 'g-o')
        plt.ylabel("Speed [m/s]")
        #plt.axvline(movement_begin, color = 'b', label = 'Movement Begin')
        #plt.axvline(movement_end, color = 'r', label = 'Movement End')
        plt.legend()
        plt.grid()

        # Positioning plot
        plt.subplot(3, 1, 3)
        plt.title("Position plot")
        #plt.plot(xpoints, ypoints, 'b-o')
        plt.xlabel("Time [s]")
        plt.ylabel("Distance [m]")
        #plt.axvline(movement_begin, color = 'b', label = 'Movement Begin')
        #plt.axvline(movement_end, color = 'r', label = 'Movement End')
        plt.legend()
        plt.grid()
        """

        print(self.points_x)
        print(self.points_y)
        plt.tight_layout()
        plt.show()

def main():
    rclpy.init()
    simulator_node = MotionControllerSimNode()
    try:
        thread = Thread(target=rclpy.spin, args=(simulator_node, ), daemon=True)
        thread.start()
        multithread_executor = MultiThreadedExecutor()
        while rclpy.ok() and not simulator_node.sim_done:
            rclpy.spin_once(simulator_node, executor=multithread_executor, timeout_sec=0.1)
        simulator_node.plotfnc()
    finally:
        simulator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()