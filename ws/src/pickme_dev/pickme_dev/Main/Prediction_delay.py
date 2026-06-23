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
        
       
        
        self.obj_buffer.append(obj) #Objekt erstes mal oder neues objekt  ->  Objekt puffern
        self.get_logger().info(
                f"obj gefuffert  | vx = {obj['vx']:.4f}| y = {obj['y']:.4f} | Aktuelles_X = {obj['x']:.4f} | logging_time = {obj['zeitpunkt_logging']}"
                f"Pufferlänge={len(self.obj_buffer)}"
                )
        

    def gripper_callback(self,msg:RobotCmd) :
        self.activ_gripper=msg.activate_gripper
        self.get_logger().info(
                    f"Greifer Status = {self.activ_gripper} ")



    def timer_callback(self):
       
        if self.activ_gripper == True and self.obj_geholt == False :
            if len(self.obj_buffer) > 0:
                self.active_obj = self.obj_buffer.popleft()
                self.obj_done=False
                self.obj_geholt= True
                self.get_logger().info(
                    f"Nach Greifprozess: neues Objekt aus Puffer aktiviert "
                    f"obj| vx = {self.active_obj['vx']:.4f}| y = {self.active_obj['y']:.4f} | Aktuelles_X = {self.active_obj['x']:.4f} | logging_time = {self.active_obj['zeitpunkt_logging']}"
                )
                return
            
            if len(self.obj_buffer) <=0:
                self.active_obj= None
                self.obj_done= True
                self.get_logger().info(
                    f"Nach Greifprozess: kein objekt im puffer "
                )
            return
        
        if self.activ_gripper== False and self.obj_geholt== True:
            self.obj_geholt= False



        if self.active_obj== None and len(self.obj_buffer)>0: 
            self.active_obj = self.obj_buffer.popleft()
            self.obj_done=False
            self.get_logger().info(
                    f"inizial : erstes Objekt aus Puffer aktiviert "
                    f"obj| vx = {self.active_obj['vx']:.4f}| y = {self.active_obj['y']:.4f} | Aktuelles_X = {self.active_obj['x']:.4f} | logging_time = {self.active_obj['zeitpunkt_logging']}"
            )
        if self.active_obj== None and len(self.obj_buffer)<=0:
            self.obj_done=True
            self.get_logger().info(
                    f"leer: kein objet vorhanden  "
            )
            return
        
        x_zum_Startzeitpunkt= self.active_obj["x"]
        vx = self.active_obj["vx"]
        y = self.active_obj["y"]
        z = self.active_obj["z"]
        obj_id = self.active_obj["obj_id"]
        time_logged= self.active_obj["zeitpunkt_logging"]
        time_now=time.time()
        self.get_logger().info(
        f"obj (obj_id={obj_id}),  y={y:.3f} m,  "
        f"vx={vx:.3f} "
    )
        
       

        # Innerhalb des Greifprozesses -> Position berechnen und publizieren
        dt = abs(time_now-time_logged)
        greifpunkt_x = vx * dt + x_zum_Startzeitpunkt
        self.get_logger().info(
        f"Greifpunkt  (obj_id={obj_id}), "
        f"x={greifpunkt_x:.3f} m, y={y:.3f} m, z={z:.3f} m, "
        f"dt={dt:.3f} s"
    )
        if greifpunkt_x< self.closest_grip_location and self.obj_done is False:
            pred_msg = PredictedPos()
            pred_msg.x = greifpunkt_x
            pred_msg.y = y
            pred_msg.z = z
            pred_msg.obj_id = float(obj_id)
            self.get_logger().info(
        f"Greifpunkt publiziert (obj_id={obj_id}), "
        f"x={greifpunkt_x:.3f} m, y={y:.3f} m, z={z:.3f} m, "
        f"dt={dt:.3f} s"
    )
            self.publisher.publish(pred_msg)
        


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