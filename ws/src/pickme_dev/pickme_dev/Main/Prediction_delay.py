import rclpy
from rclpy.node import Node
from collections import deque
import time

from ro45_portalrobot_interfaces.msg import PredictedPos, PredictedPosdelay, RobotCmd 




class DelayBufferNode(Node):
    """
    ROS2-Knoten zur zeitbasierten Positionsvorhersage von Förderbandobjekten.
    Der Knoten puffert erkannte Objekte, berechnet deren aktuelle Position
    anhand der Förderbandgeschwindigkeit und veröffentlicht die
    vorhergesagte Greifposition zum geeigneten Zeitpunkt für den Roboter.
    """
    def __init__(self):
        """
        Initialisiert Publisher, Subscriber und interne Objektverwaltung.
        Der Knoten empfängt erkannte Objekte sowie den Status des Greifers und
        verwaltet einen FIFO-Puffer zur sequenziellen Verarbeitung der Objekte.
        """
        super().__init__("delay_buffer_node")
       
       
        self.create_subscription(
        PredictedPosdelay,
        "/predicted_positiondelay",
        self.pos_callback,
        10,
        ) 


        self.create_subscription(
        RobotCmd,
        '/robot_command',
        self.gripper_callback,
        10,
        )


        self.publisher = self.create_publisher(
            PredictedPos,
            "/predicted_position",
            10,
        )

        
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            f"DelayBufferNode gestartet ")
        

        self.obj_buffer = deque()

        self.active_obj= None
        self.obj_geholt=False
        self.activ_gripper=False
        self.obj_done=False
        self.last_x=None
        self.closest_grip_location= 0
        self.gripper_lock=False






    def pos_callback(self, msg: PredictedPosdelay):
        """
        Speichert neu erkannte Objekte im Verarbeitungspuffer.
        Objekte werden entsprechend ihrer Reihenfolge auf dem Förderband in
        einem FIFO-Puffer abgelegt und später einzeln verarbeitet.
        Args:msg: Vorverarbeitete Objektdaten aus der Waypoint-Prediction.
        """
        

        obj = {
            "vx": msg.vx,
            "y": msg.y,
            "x": msg.x,
            "z": msg.z,
            "obj_id": msg.obj_id,
            "zeitpunkt_logging": time.time(), 
        }
        
       
         # First received object initializes the ordering reference.
        if self.last_x is None:
            self.last_x = obj["x"]
            if obj["obj_id"] != 0 :
                self.obj_buffer.append(obj)
                self.get_logger().debug(f"Erstes Objekt gepuffert | x={obj['x']:.4f}")
            return
            
        if obj["x"] > self.last_x:
            if obj["obj_id"] is not 0 :
                self.obj_buffer.append(obj) 
                self.get_logger().info(
                    f"obj gefuffert  | vx = {obj['vx']:.4f}| y = {obj['y']:.4f} | Aktuelles_X = {obj['x']:.4f} | logging_time = {obj['zeitpunkt_logging']}"
                    f"Pufferlänge={len(self.obj_buffer)}"
                )
        self.last_x = obj["x"]

    def gripper_callback(self,msg:RobotCmd) :
        """
        Aktualisiert den aktuellen Status des Greifers.
        Args: msg: Nachricht mit dem aktuellen Greiferzustand.
        """
        self.activ_gripper=msg.activate_gripper
        self.get_logger().debug(
                    f"Greifer Status = {self.activ_gripper} ")



    def timer_callback(self):
        """
        Aktualisiert periodisch die vorhergesagte Objektposition.
        Für das aktuell aktive Objekt wird anhand der vergangenen Zeit die
        aktuelle Position berechnet. Sobald sich das Objekt innerhalb des
        Greifbereichs befindet, wird dessen Position veröffentlicht. Nach einem
        erfolgreichen Greifvorgang wird das nächste Objekt aus dem Puffer
        übernommen.
        """
    
        # Process only one active object at a time.
        if self.active_obj is None:
            if len(self.obj_buffer) > 0:
                self.active_obj = self.obj_buffer.popleft()
                self.get_logger().debug(
                    f"Neues Objekt aktiviert | obj_id={self.active_obj['obj_id']}"
                )
            else:
                self.get_logger().debug(f"Puffer leer | kein Objekt vorhanden")
                return

       
        x_zum_Startzeitpunkt = self.active_obj["x"]
        vx = self.active_obj["vx"]
        y = self.active_obj["y"]
        z = self.active_obj["z"]
        obj_id = self.active_obj["obj_id"]
        time_logged = self.active_obj["zeitpunkt_logging"]
        # Estimate the current object position from elapsed time.
        dt = abs(time.time() - time_logged)
        greifpunkt_x = vx * dt + x_zum_Startzeitpunkt

        if self.activ_gripper is False and self.gripper_lock is True:
            self.gripper_lock=False 

        
        if greifpunkt_x > self.closest_grip_location:
            self.obj_geholt=None
            self.get_logger().debug(
                f"Warten | obj_id={obj_id} | x={greifpunkt_x:.3f} < grenze={self.closest_grip_location}"
            )
            return

        
        if self.activ_gripper is True and self.gripper_lock is False :
            self.active_obj = None
            self.gripper_lock=True
            self.get_logger().info(
                f"Greifer aktiv | obj_id={obj_id} abgeschlossen | nächstes Objekt wird geholt"
            )
            return

        
        pred_msg = PredictedPos()
        pred_msg.x = greifpunkt_x
        pred_msg.y = y
        pred_msg.z = z
        pred_msg.obj_id = float(obj_id)
        self.publisher.publish(pred_msg)
        self.get_logger().debug(
            f"Publishe | obj_id={obj_id} | x={greifpunkt_x:.3f} m | dt={dt:.3f} s"
        )
    
        


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


if __name__ == "__main__":
    main()