
import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from ro45_portalrobot_interfaces.msg import RobotPosStamped, PredictedPos
from collections import deque








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
         
        self.x_buffer = deque(maxlen=5)
        self.y_buffer = deque(maxlen=5)
        self.z_buffer = deque(maxlen=5)

        self.last_msg= None
        self.last_msg_avg = None
        self.lookahead_sec = 1.0 # die zeit wo wir die kordinaten von dem objekt in der zukunft berechnen 
        self.z = 0.0  # das förderband ist laut kordinaten ursprung layer null in der z achse 
        self.get_logger().info('WaypointPredictionNode gestartet.')

    def robot_pos_callback(self, msg: RobotPosStamped):
        if self.last_msg_avg is None:
            self.last_msg_avg = self.moving_average(msg.x,msg.y)
            self.last_msg=msg
            return

        dt = self.time_diff_sec(self.last_msg.stamp, msg.stamp)
        if dt <= 0.0:
            self.get_logger().warn('Ungültiger Zeitunterschied.')
            self.last_msg = msg
            return
        avg_x, avg_y = self.moving_average(msg.x, msg.y)
        vx = (avg_x - self.last_msg_avg[0]) / dt
        vy = (avg_y - self.last_msg_avg[1]) / dt

        pred_x = msg.x + vx * self.lookahead_sec
        pred_y = msg.y + vy * self.lookahead_sec

        pred_msg = PredictedPos()
        
        pred_msg.x = pred_x
        pred_msg.y = pred_y
        pred_msg.z= self.z 
        pred_msg.obj_id = msg.obj_id 

        self.publisher_prediction.publish(pred_msg)

        self.get_logger().info(
            f'Pos: ({msg.x:.2f}, {msg.y:.2f}) '
            f'v=({vx:.2f}, {vy:.2f}) '
            f'Pred: ({pred_x:.2f}, {pred_y:.2f})'
        )

        self.last_msg = msg
        self.last_msg_avg = self.moving_average(msg.x,msg.y)

    def time_diff_sec(self, t1: Time, t2: Time) -> float:
        ''' funktion die den zeitunterschied von zwei zeitstempeln berechnet erster Teil sekunden zweiter Teil nanosekunden '''
        return (t2.sec - t1.sec) + (t2.nanosec - t1.nanosec) * 1e-9
    def moving_average(self, x, y):
        self.x_buffer.append(x)
        self.y_buffer.append(y)

        avg_x = sum(self.x_buffer) / len(self.x_buffer)
        avg_y = sum(self.y_buffer) / len(self.y_buffer)

        return avg_x, avg_y

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