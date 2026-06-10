
import rclpy
from rclpy.node import Node

from ro45_portalrobot_interfaces.msg import CamData, PredictedPosdelay
from collections import deque








class WaypointPreditionNode(Node):
    def __init__(self):
        super().__init__('WaypointPredition_node')
        

        self.subscriber_position = self.create_subscription(
            CamData,
            '/CamData',
            self.robot_pos_callback,
            10
        )

        self.publisher_prediction = self.create_publisher(
            PredictedPosdelay,
            '/predicted_positiondelay',
            10
        )
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.x_buffer = deque(maxlen=5)
        self.y_buffer = deque(maxlen=5)
        self.time_buffer= deque(maxlen=5)
        self.Föderband_layer = 0.0 
        self.last_msg= None
        self.obj_id=None
        
        
        self.get_logger().info('WaypointPredictionNode gestartet.')

    def robot_pos_callback(self, msg: CamData):
       
        self.x_buffer.append(msg.x)
        self.y_buffer.append(msg.y)
        self.time_buffer.append(msg.timestamp)
        self.obj_id=msg.obj_type 

    
    def timer_callback(self):
        if len(self.x_buffer) > 0 and self.last_msg==None :
            self.last_msg=self.moving_average
            self.x_buffer=None
            self.y_buffer=None
            self.time_buffer=None

        if self.last_msg!= None :
            self.aktuelle_Werte_msg =self.moving_average
            dt=self.time_diff_sec(self.aktuelle_Werte_msg[3],self.last_msg[3])
            vx=self.berechnung_Geschwindigkeit(self.aktuelle_Werte_msg[1],self.last_msg[1],dt )
        else :
            return


        pred_msg = PredictedPosdelay()
        
        pred_msg.vx = vx
        pred_msg.y = self.aktuelle_Werte_msg[2]
        pred_msg.z = self.Föderband_layer 
        pred_msg.x= self.aktuelle_Werte_msg[1]
        pred_msg.obj_id = self.obj_id

        self.publisher_prediction.publish(pred_msg)

        

        self.last_msg =None
    def berechnung_Geschwindigkeit(self,avg_x,last_avg_x,dt):
        vx=(avg_x-last_avg_x)/dt
        return vx 

    def time_diff_sec(self, t1, t2) -> float:
        
        return abs(t2 - t1)
    
    def moving_average(self):
       
        avg_x = sum(self.x_buffer) / len(self.x_buffer)
        avg_y = sum(self.y_buffer) / len(self.y_buffer)
        avg_time=sum(self.time_buffer) / len(self.time_buffer)
        return avg_x, avg_y, avg_time

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