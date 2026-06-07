import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import threading
import random
from ro45_portalrobot_interfaces.msg import RobotCmd, RobotPos
import matplotlib.pyplot as plt

# Timer Timebase in [s]
TIMEBASE = 0.1

# Positions of Endstops in [m]
ENDSTOP_X_POS = 0.000
ENDSTOP_X_NEG = -0.40

ENDSTOP_Y_POS = 0.000
ENDSTOP_Y_NEG = -0.20

ENDSTOP_Z_POS = 0.000
ENDSTOP_Z_NEG = -0.345

TCP_OFFSET_FROM_ENDSTOP_X = 0.060
TCP_OFFSET_FROM_ENDSTOP_Y = 0.013
TCP_OFFSET_FROM_ENDSTOP_Z = 0.250

class simulator(Node):
    def __init__(self):
        super().__init__('simulator_node')
        self.callback_group = ReentrantCallbackGroup()

        self.publisher_pos = self.create_publisher(
            RobotPos,
            '/robot_position',
            10
        )

        self.subscriber_command = self.create_subscription(
            RobotCmd,
            '/robot_command',
            self.robot_command_callback,
            10
        )

        # Randomly generate init position (VOR dem Timer und direkt in den Zustand)
        self.last_integrated_positionx = -float(random.randint(0, 40)) / 100
        self.last_integrated_positiony = -float(random.randint(0, 20)) / 100
        self.last_integrated_positionz = -float(random.randint(0, 30)) / 100

        self.timer = self.create_timer(TIMEBASE, self.main, callback_group=self.callback_group)

        # Robot position state
        self.pos = RobotPos()
        self.position_x = self.last_integrated_positionx
        self.position_y = self.last_integrated_positiony
        self.position_z = self.last_integrated_positionz
        self.pos_x_array = []
        self.pos_y_array = []
        self.pos_z_array = []

        self.pos_x_tcp_array = []
        self.pos_y_tcp_array = []
        self.pos_z_tcp_array = []

        self.time_array = []
        self.time = 0

        # Robotcmd interface
        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0
        self.activate_gripper = 0
        self.robotcmd_lock = threading.Lock()

        # Last integrated values
        self.last_speedx = 0
        self.last_speedy = 0
        self.last_speedz = 0
        
        self.last_accelx = 0
        self.last_accely = 0
        self.last_accelz = 0

        self.last_integrated_speedx = 0
        self.last_integrated_speedy = 0
        self.last_integrated_speedz = 0


    def robot_command_callback(self, msg: RobotCmd):
        with self.robotcmd_lock:
            self.last_accelx = self.accel_x
            self.last_accely = self.accel_y
            self.last_accelz = self.accel_z

            self.accel_x = msg.accel_x
            self.accel_y = msg.accel_y
            self.accel_z = msg.accel_z
            self.activate_gripper = msg.activate_gripper


    def main(self):
        # 1. Integral: Beschleunigung -> Geschwindigkeit
        self.speedx = self.integrator(self.accel_x, self.last_accelx, self.last_integrated_speedx)
        self.speedy = self.integrator(self.accel_y, self.last_accely, self.last_integrated_speedy)
        self.speedz = self.integrator(self.accel_z, self.last_accelz, self.last_integrated_speedz)

        # 2. Integral: Geschwindigkeit -> Position (Nutzt jetzt Geschwindigkeit!)
        self.position_x = self.integrator(self.speedx, self.last_speedx, self.last_integrated_positionx)
        self.position_y = self.integrator(self.speedy, self.last_speedy, self.last_integrated_positiony)
        self.position_z = self.integrator(self.speedz, self.last_speedz, self.last_integrated_positionz)

        # Check if endstops are reached
        self.check_endstops()

        # Update der "Last"-Werte für den nächsten Integrationsschritt
        self.last_speedx = self.speedx
        self.last_speedy = self.speedy
        self.last_speedz = self.speedz
        self.last_integrated_speedx = self.speedx
        self.last_integrated_speedy = self.speedy
        self.last_integrated_speedz = self.speedz
        self.last_integrated_positionx = self.position_x
        self.last_integrated_positiony = self.position_y
        self.last_integrated_positionz = self.position_z

        # Publish current position
        self.pos.pos_x = self.position_x
        self.pos.pos_y = self.position_y
        self.pos.pos_z = self.position_z
        self.publisher_pos.publish(self.pos)

        self.pos_x_array.append(self.position_x)
        self.pos_y_array.append(self.position_y)
        self.pos_z_array.append(self.position_z)

        self.pos_x_tcp_array.append(self.position_x - TCP_OFFSET_FROM_ENDSTOP_X)
        self.pos_y_tcp_array.append(self.position_y - TCP_OFFSET_FROM_ENDSTOP_Y)
        self.pos_z_tcp_array.append(self.position_z + TCP_OFFSET_FROM_ENDSTOP_Z)

        self.time_array.append(self.time)
        self.time = self.time + TIMEBASE # Zeitbasis in Sekunden für korrekten Plot

        print("\n", "POS X: ", self.position_x, "\n", "POS Y: ", self.position_y, "\n", "POS Z: ", self.position_z)


    def integrator(self, current_value, last_value, last_integrated):
        integrated_value = last_integrated + 0.5 * (last_value + current_value) * TIMEBASE
        return integrated_value


    def check_endstops(self):
        # Achse X
        if self.position_x > ENDSTOP_X_POS:
            self.position_x = ENDSTOP_X_POS
        elif self.position_x < ENDSTOP_X_NEG:
            self.position_x = ENDSTOP_X_NEG
        
        # Achse Y
        if self.position_y > ENDSTOP_Y_POS:
            self.position_y = ENDSTOP_Y_POS
        elif self.position_y < ENDSTOP_Y_NEG:
            self.position_y = ENDSTOP_Y_NEG
        
        # Achse Z
        if self.position_z > ENDSTOP_Z_POS:
            self.position_z = ENDSTOP_Z_POS
        elif self.position_z < ENDSTOP_Z_NEG:
            self.position_z = ENDSTOP_Z_NEG
    

    def plotallaxis(self):
        # Plot X
        plt.subplot(3, 1, 1)
        plt.title("X-Axis")
        plt.plot(self.time_array, self.pos_x_array, '-', color='black')
        plt.plot(self.time_array, self.pos_x_tcp_array, '-', color='blue')
        plt.ylabel("Position X")
        plt.axhline(ENDSTOP_X_POS, color = 'green', linestyle='--', label='Endstop Positive (Home)')
        plt.axhline(ENDSTOP_X_NEG, color = 'red', linestyle='--', label='Endstop Negative')
        plt.legend()
        plt.grid()

        # Plot Y
        plt.subplot(3, 1, 2)
        plt.title("Y-Axis")
        plt.plot(self.time_array, self.pos_y_array, '-', color='black')
        plt.plot(self.time_array, self.pos_y_tcp_array, '-', color='blue')
        plt.ylabel("Position Y")
        plt.axhline(ENDSTOP_Y_POS, color = 'green', linestyle='--', label='Endstop Positive (Home)')
        plt.axhline(ENDSTOP_Y_NEG, color = 'red', linestyle='--', label='Endstop Negative')
        plt.legend()
        plt.grid()

        # Plot Z
        plt.subplot(3, 1, 3)
        plt.title("Z-Axis")
        plt.plot(self.time_array, self.pos_z_array, '-', color='black')
        plt.plot(self.time_array, self.pos_z_tcp_array, '-', color='blue')
        plt.gca().invert_yaxis()
        plt.ylabel("Position Z")
        plt.axhline(ENDSTOP_Z_NEG, color = 'green', linestyle='--', label='Endstop Negative (Home)')
        plt.axhline(ENDSTOP_Z_POS, color = 'red', linestyle='--', label='Endstop Positive (conveyor belt)')
        plt.legend()
        plt.grid()

        plt.tight_layout()
        plt.show()


def main():
    rclpy.init()
    simulator_node = simulator()
    try:
        multithread_executor = MultiThreadedExecutor()
        rclpy.spin(simulator_node, multithread_executor)
    finally:
        simulator_node.plotallaxis()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()