
import rclpy
from rclpy.node import Node

from ro45_portalrobot_interfaces.msg import CamData, PredictedPosdelay
import statistics
from pickme_dev.WaypointPredictionEngine.Predic_logic import ConveyorSpeedEstimator

class WaypointPreditionNode(Node):
    def __init__(self):
        super().__init__('WaypointPredition_node')
        self.get_logger().info('WaypointPredictionNode gestartet.')
        self.speed_calculator= ConveyorSpeedEstimator()
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

        self.timer = self.create_timer(0.1, self.berechne_geschwindigkeit)        
        
        self.x_alt = None
        self.time_alt= None
        self.obj_type = None
        self.x_aktuell=None
        self.x_logged =None
        self.time_aktuell=None
        self.y_aktuell=None
        self.velocity_queue = []  
        self.queue_y= []
        self.median_vx=None


        self.Föderband_layer = 0.0        
        self.threshold = 0.06
        self.grenze = 0.04
        self.min_Elemente_queue= 10 
        self.rejekted_objekt_type=0


    def berechne_geschwindigkeit(self):

        # Block 1: Typ-Check
        if self.obj_type == self.rejekted_objekt_type or self.obj_type is None :
            self.get_logger().debug(f"Block 1: Objekt übersprungen | obj_type = {self.obj_type}")
        
            return
        if self.x_aktuell is None or self.time_aktuell is None:
            self.get_logger().debug(f"Block 1: Keine Daten vom Callback | x_aktuell = {self.x_aktuell} | time_aktuell = {self.time_aktuell}")
            return
        # Block 2: Initialisierung
        if self.x_alt is None or self.time_alt is None:
            self.get_logger().debug(f"Block 2: Initialisierung | x_alt gesetzt auf {self.x_aktuell:.4f}")
            self.aktualiesiere_Werte()
            return
        """ 
        # Block 3: Neues Objekt erkannt
        if abs(self.x_alt - self.x_aktuell) > self.threshold:
            self.x_logged=self.x_alt
            self.get_logger().info(f"Block 3: Neues Objekt erkannt | Sprung = {abs(self.x_alt - self.x_aktuell):.4f} > threshold {self.threshold}")
            if len(self.velocity_queue) >= self.min_Elemente_queue :
                self.median_vx = statistics.median(self.velocity_queue)
                self.median_y = statistics.median(self.queue_y)
                self.publish()
                self.get_logger().info(f"Block 3: Publish | median_vx = {self.median_vx:.4f} | median_y = {self.median_y:.4f} | Werte = {len(self.velocity_queue)}")
            else:
                self.get_logger().warning(f"Block 3: Queue zu klein ({len(self.velocity_queue)} < {self.min_Elemente_queue}) | kein Publish")
            self.velocity_queue = []          # list reset
            self.queue_y=[]
            self.aktualiesiere_Werte()
            return
"""
        if self.time_aktuell != self.time_alt:
            #dt=self.time_diff(self.time_aktuell,self.time_alt)
            #if dt<= 0:
                #self.get_logger().warning(f"Block 4: dt <= 0 ({dt}) | übersprungen")
                #return
            #vx = self.berechnung_Geschwindigkeit(self.x_aktuell,self.x_alt,dt)
            #self.velocity_queue.append(vx)        # list append
            self.queue_y.append(self.y_aktuell)
            #self.get_logger().debug(f"Block 4: vx = {vx:.4f} | dt = {dt:.4f} | queue_länge = {len(self.velocity_queue)}")
            self.aktualiesiere_Werte()
            
        self.median_vx=self.speed_calculator.update(self.x_aktuell,self.time_aktuell)
        self.x_logged=self.x_aktuell
        if self.median_vx is None:
            return
        # Block 5: Objekt verlässt Band
        if self.x_aktuell < self.grenze:
            self.x_logged=self.x_aktuell
            self.get_logger().info(f"Block 5: Objekt verlässt Band | x_aktuell = {self.x_aktuell:.4f} < grenze {self.grenze}")
            #if len(self.velocity_queue) >= self.min_Elemente_queue :
                #self.median_vx = statistics.median(self.velocity_queue)
            self.median_y=statistics.median(self.queue_y)
            self.publish()
            self.get_logger().info(f"Block 5: Publish | median_vx = {self.median_vx:.4f} | median_y = {self.median_y:.4f} | Werte = {len(self.velocity_queue)}")
            
            self.get_logger().warning(f"Block 5: Queue zu klein ({len(self.velocity_queue)} < {self.min_Elemente_queue}) | kein Publish")
            self.velocity_queue = []          # list reset
            self.queue_y=[]
            self.x_alt = None
            self.time_alt=None
        
        
        
    # if type == 0 --> skip 
    # else 
        # if x_alt == None 
            #x_alt = x_akjtuell 
            # return 
        # if abs(x_alt- x_aktuell) > threshold =0,02 
            # if length von quee > 10 
                # mache median auf vx und speichere 
                # publisch msg 
            # neue queee 
            # return 
        # else 
            # berechne vx und packe es in die quee 
            # delta_x = x_aktuell - x_alt
            #vx = delta_x / dt  # dt = Zeit seit letztem Frame

            #queue.append(vx)
        # setzte x_aktuell zu  x_alt 
        # if x_aktuell < grenze = 0,03 
        # bilde den median auf der quee 
        # publisch msg 
        # leree queue 


    def robot_pos_callback(self, msg: CamData):
       
        self.x_aktuell=msg.x
        self.y_aktuell=msg.y
        self.time_aktuell=msg.timestamp
        self.obj_type=msg.obj_type 

    
    
    def aktualiesiere_Werte(self):
        self.x_alt=self.x_aktuell
        self.time_alt=self.time_aktuell
    
    def publish(self):

        pred_msg = PredictedPosdelay()
        
        pred_msg.vx = self.median_vx
        pred_msg.y = self.median_y
        pred_msg.z = self.Föderband_layer 
        pred_msg.x= self.x_logged
        pred_msg.obj_id = self.obj_type

        self.publisher_prediction.publish(pred_msg)
        self.get_logger().info(f'Prediction publiziert | median_vx = {self.median_vx:.4f}| median_y = {self.median_y:.4f} | Aktuelles_X = {self.x_aktuell:.4f} | Obj_type = {self.obj_type:.4f}')
        
    def berechnung_Geschwindigkeit(self,akt_x,last_x,dt):
        vx=(akt_x-last_x)/dt
        return vx 

    def time_diff(self, t1, t2) -> float:
        
        return abs(t2 - t1)
        

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