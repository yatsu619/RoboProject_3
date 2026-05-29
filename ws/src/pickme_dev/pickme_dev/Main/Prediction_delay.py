
import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import PredictedPos,PredictedPosdelay


class DelayBufferNode(Node):
    def __init__(self):
        super().__init__('delay_buffer_node')

        self.delay_sec = 0.7  # Zeit bis zum Greifen,messesn 

        self.subscription = self.create_subscription(
            PredictedPosdelay,
            '/predicted_positiondelay',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            PredictedPos,
            '/predicted_position',
            10
        )

        self.message_buffer = []
        self.current_id = None
        self.pending_time = None

        self.get_logger().info('DelayBufferNode gestartet - verzögert um {:.2f}s'.format(self.delay_sec))

        self.timer = self.create_timer(0.05, self.timer_callback)

    def callback(self, msg):

        if self.current_id is None:
            self.current_id = msg.id

        if msg.id != self.current_id:

            self.get_logger().info(
            f'Neue ID erkannt: {msg.id} -> Buffer zurücksetzen'
            )

            self.message_buffer.clear()
            self.current_id = msg.id

            self.pending_time = self.get_clock().now()

        elif len(self.message_buffer) == 0:
            self.pending_time = self.get_clock().now()

        self.message_buffer.append(msg)

    def timer_callback(self):

        if len(self.message_buffer) > 0 and self.pending_time is not None:

            now = self.get_clock().now()
            delta = (now - self.pending_time).nanoseconds * 1e-9

            if delta >= self.delay_sec:

                for msg in self.message_buffer:
                    self.publisher.publish(msg)

                self.message_buffer.clear()
                self.pending_time = None


def main(args=None):
    rclpy.init(args=args)
    node = DelayBufferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()