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

TIMEBASE = 0.01 # 50Hz


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

        self.publisher_position = self.create_publisher(
            RobotPos,
            '/robot_position',
            10
        )

        self.timer = self.create_timer(TIMEBASE, self.PrimThread)
        
        self.pos = RobotPos()
        
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

        self.point_position = 0
        self.velocity = 0

        self.loops = 0
        self.oversampling = 0

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
            self.latest_accel = msg.accel_x

            if (msg.activate_gripper == True):
                self.done = True


    def PrimThread(self):
        # Append the latest commanded Acceleration
        self.points_y.append(self.latest_accel)

        # Integrate acceleration to velocity and velocity to position
        self.velocity += self.latest_accel * TIMEBASE
        self.point_position += self.velocity * TIMEBASE
        self.pos.pos_x = self.point_position
        self.publisher_position.publish(self.pos)
        print(self.point_position)
        
        # Append the timebase (x-axis) with + 1. Offset by one to not start at 0.
        self.points_x.append((self.loops + 1))
        self.loops += 1
        
        if (self.done == True):
            self.oversampling += 1
            if (self.oversampling > 50):
                self.timer.cancel()
                self.sim_done_time = self.points_x[-1]
                self.sim_done = True

    def plotfnc(self):
        # Recalculate sampling to timer timebase
        for i, value in enumerate(self.points_x):
            self.points_x[i] = value * TIMEBASE
        self.start_time = self.start_time * TIMEBASE
        self.sim_done_time = self.sim_done_time * TIMEBASE
        
        # Conversion to numpy arrays
        time = np.array(self.points_x)
        accel = np.array(self.points_y)

        # Integrate Acceleration to get velocity and position
        velocity = np.cumsum(accel) * TIMEBASE
        position = np.cumsum(velocity) * TIMEBASE

        # Acceleration plot
        plt.subplot(3, 1, 1)
        plt.title("Acceleration plot")
        plt.plot(time, accel, '-')
        plt.ylabel("Acceleration [m/s²]")
        plt.axvline(self.start_time, color = 'blue', label = 'Simulation Begin')
        plt.axvline(self.sim_done_time, color = 'blue', label = 'Simulation End')
        plt.axhline(0.1, color = 'red', label = 'Max. Acceleration', linestyle='--')
        plt.axhline(-0.1, color = 'red', linestyle='--')
        if (any(accel) > 0.1):
            plt.fill_between(time, accel, 0.1, where=(accel > 0.1), facecolor='none', hatch='//', edgecolor='red', label='Critical Acceleration')
            plt.fill_between(time, accel, -0.1, where=(accel < -0.1), facecolor='none', hatch='//', edgecolor='red')
        plt.legend()
        plt.grid()

        # Speed plot
        plt.subplot(3, 1, 2)
        plt.title("Speed plot")
        plt.plot(time, velocity, '-')
        plt.ylabel("Speed [m/s]")
        plt.axvline(self.start_time, color = 'blue', label = 'Simulation Begin')
        plt.axvline(self.sim_done_time, color = 'blue', label = 'Simulation End')
        plt.legend()
        plt.grid()

        # Positioning plot
        plt.subplot(3, 1, 3)
        plt.title("Position plot")
        plt.plot(time, position, '-')
        plt.xlabel("Time [s]")
        plt.ylabel("Distance [m]")
        plt.axvline(self.start_time, color = 'blue', label = 'Simulation Begin')
        plt.axvline(self.sim_done_time, color = 'blue', label = 'Simulation End')
        plt.axhline(self.coordinate, color = 'g', label = "Target")
        plt.legend()
        plt.grid()

        print(time)
        print(accel)
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