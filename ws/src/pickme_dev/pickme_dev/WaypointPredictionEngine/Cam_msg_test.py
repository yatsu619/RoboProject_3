import rclpy
from rclpy.node import Node
import time
from ro45_portalrobot_interfaces.msg import CamData
import random


class CamNodeTest(Node):
    def __init__(self):
        super().__init__('cam_msg_test')
        self.publisher = self.create_publisher(CamData, '/CamData', 10)

        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.start_x = 0.10      # erster negativer Wert in Meter
        self.speed = -0.01         # 1 cm/s = 0.01 m/s
        self.elapsed = 0.0
        self.cycle_time = 6.0

        self.get_logger().info('Cam_msg_test gestartet')

    def timer_callback(self):
        x = self.start_x + self.speed * self.elapsed
        y = 0.0

        msg = CamData()
        msg.obj_type = random.randint(1,2)
        msg.x = float(x)
        msg.y = float(y)
        msg.timestamp = float(time.time())

        self.publisher.publish(msg)
        self.get_logger().info(
            f'Publiziert: obj_type={msg.obj_type}, x={msg.x:.4f}, y={msg.y:.4f}'
        )

        self.elapsed += self.timer_period
        if self.elapsed >= self.cycle_time:
            self.elapsed = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = CamNodeTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()