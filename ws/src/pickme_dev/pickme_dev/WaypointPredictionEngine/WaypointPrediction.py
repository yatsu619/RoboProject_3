
import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from ro45_portalrobot_interfaces import RobotPosStamped, PredictedPos







class WaypointPreditionNode(Node):
    def __init__(self):
        super().__init__('WaypointPredition_node')
        

        self.subscriber_position = self.create_subscription(
            RobotPosStamped,
            '/robot_position',
            self.robot_pos_callback,
            10
        )

        self.publisher_prediction = self.create_publisher(
            PredictedPos,
            '/predicted_position',
            10
        )

        self.last_msg = None
        self.lookahead_sec = 1.0

        self.get_logger().info('WaypointPredictionNode gestartet.')

    def robot_pos_callback(self, msg: RobotPosStamped):
        if self.last_msg is None:
            self.last_msg = msg
            return

        dt = self.time_diff_sec(self.last_msg.stamp, msg.stamp)
        if dt <= 0.0:
            self.get_logger().warn('Ungültiger Zeitunterschied.')
            self.last_msg = msg
            return

        vx = (msg.x - self.last_msg.x) / dt
        vy = (msg.y - self.last_msg.y) / dt

        pred_x = msg.x + vx * self.lookahead_sec
        pred_y = msg.y + vy * self.lookahead_sec

        pred_msg = PredictedPos()
        pred_msg.stamp = msg.stamp
        pred_msg.x = pred_x
        pred_msg.y = pred_y
        pred_msg.vx = vx
        pred_msg.vy = vy

        self.publisher_prediction.publish(pred_msg)

        self.get_logger().info(
            f'Pos: ({msg.x:.2f}, {msg.y:.2f}) '
            f'v=({vx:.2f}, {vy:.2f}) '
            f'Pred: ({pred_x:.2f}, {pred_y:.2f})'
        )

        self.last_msg = msg

    def time_diff_sec(self, t1: Time, t2: Time) -> float:
        return (t2.sec - t1.sec) + (t2.nanosec - t1.nanosec) * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPreditionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()