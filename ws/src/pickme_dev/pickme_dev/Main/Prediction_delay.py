import rclpy
from rclpy.node import Node
from collections import deque
import time

from ro45_portalrobot_interfaces.msg import PredictedPos, PredictedPosdelay, RobotCmd 




class DelayBufferNode(Node):
    def __init__(self):
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

        # Timer für regelmäßige Positionsberechnung (hier 10 Hz)
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
        






    def pos_callback(self, msg: PredictedPosdelay):
        """
        Empfange PredictedPosdelay und puffere Objekte.
        """
        

        obj = {
            "vx": msg.vx,
            "y": msg.y,
            "x": msg.x,
            "z": msg.z,
            "obj_id": msg.obj_id,
            "zeitpunkt_logging": time.time(), 
        }
        
       
         # Erstes Objekt → direkt puffern und last_x setzen
        if self.last_x is None:
            self.last_x = obj["x"]
            self.obj_buffer.append(obj)
            self.get_logger().info(f"Erstes Objekt gepuffert | x={obj['x']:.4f}")
            return
            
        if obj["x"] > self.last_x:
            self.obj_buffer.append(obj) #Objekt erstes mal oder neues objekt  ->  Objekt puffern
            self.get_logger().info(
                    f"obj gefuffert  | vx = {obj['vx']:.4f}| y = {obj['y']:.4f} | Aktuelles_X = {obj['x']:.4f} | logging_time = {obj['zeitpunkt_logging']}"
                    f"Pufferlänge={len(self.obj_buffer)}"
                )
        self.last_x = obj["x"]

    def gripper_callback(self,msg:RobotCmd) :
        self.activ_gripper=msg.activate_gripper
        self.get_logger().info(
                    f"Greifer Status = {self.activ_gripper} ")



    def timer_callback(self):
       
      # Kein aktives Objekt → aus Puffer holen
        if self.active_obj is None:
            if len(self.obj_buffer) > 0:
                self.active_obj = self.obj_buffer.popleft()
                self.get_logger().info(
                    f"Neues Objekt aktiviert | obj_id={self.active_obj['obj_id']}"
                )
            else:
                self.get_logger().debug(f"Puffer leer | kein Objekt vorhanden")
                return

        # Werte aus aktivem Objekt
        x_zum_Startzeitpunkt = self.active_obj["x"]
        vx = self.active_obj["vx"]
        y = self.active_obj["y"]
        z = self.active_obj["z"]
        obj_id = self.active_obj["obj_id"]
        time_logged = self.active_obj["zeitpunkt_logging"]

        dt = abs(time.time() - time_logged)
        greifpunkt_x = vx * dt + x_zum_Startzeitpunkt

        # Noch zu weit weg → warten
        if greifpunkt_x < self.closest_grip_location:
            self.obj_geholt=None
            self.get_logger().debug(
                f"Warten | obj_id={obj_id} | x={greifpunkt_x:.3f} < grenze={self.closest_grip_location}"
            )
            return

        # Greifer hat gegriffen → reset und nächstes Objekt
        if self.activ_gripper == True:
            self.active_obj = None
            self.get_logger().info(
                f"Greifer aktiv | obj_id={obj_id} abgeschlossen | nächstes Objekt wird geholt"
            )
            return

        # In Reichweite + Greifer noch nicht aktiv → publishen
        pred_msg = PredictedPos()
        pred_msg.x = greifpunkt_x*-1
        pred_msg.y = y
        pred_msg.z = z
        pred_msg.obj_id = float(obj_id)
        self.publisher.publish(pred_msg)
        self.get_logger().info(
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