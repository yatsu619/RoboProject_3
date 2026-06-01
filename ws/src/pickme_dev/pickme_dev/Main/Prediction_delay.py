
import rclpy
from rclpy.node import Node
from datetime import datetime
from ro45_portalrobot_interfaces.msg import PredictedPos,PredictedPosdelay


class DelayBufferNode(Node):
    def __init__(self):
        super().__init__('delay_buffer_node')

        
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

       

        self.get_logger().info('DelayBufferNode gestartet - verzögert um {:.2f}s'.format(self.delay_sec))

      
    def callback(self, msg):
       
        if datetime.now() >= msg.obj_zero :
            verfahren_zeit_seit_0 =self.time_diff_sec(msg.obj_zero, datetime.now())
            PredictedPosdelay= 0.0+ msg.vx *verfahren_zeit_seit_0 
            return PredictedPosdelay 
        
        pred_msg = PredictedPos()
        
    
        pred_msg.x = PredictedPosdelay
        pred_msg.y = msg.y
        pred_msg.z = msg.z 
    
        pred_msg.obj_id = msg.obj_type 

        self.publisher_prediction.publish(pred_msg)
        
    def time_diff_sec(self, t1, t2) :
        
        return (t2.sec - t1.sec)
    
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