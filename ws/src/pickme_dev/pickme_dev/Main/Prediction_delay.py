import rclpy
from rclpy.node import Node
from collections import deque
import time

from ro45_portalrobot_interfaces.msg import PredictedPos, PredictedPosdelay, RobotCmd 




class DelayBufferNode(Node):
    def __init__(self):
        super().__init__("delay_buffer_node")

        
       

        
        self.obj_buffer = deque()
        self.last_x_per_objtype={}
        self.active_obj= None
        self.is_gripping=False 
        self.activ_gripper=False
        self.last_x=None
        self.min_abstand_obj= 0.025

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
            f"DelayBufferNode gestartet "
        )

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
        obj_id=obj["obj_id"]
        x=obj["x"]
        last_x=self.last_x_perobjtype.get(obj_id,None)

        if self.last_x is None or abs(x - last_x) >= self.min_abstand_obj :
            self.obj_buffer.append(obj) #Objekt erstes mal oder neues objekt  ->  Objekt puffern
            self.get_logger().info(
                f"Neues Objekt gepuffert (obj_id={obj['obj_id']}), "
                f"Pufferlänge={len(self.obj_buffer)}"
                )
        self.last_x_per_objtype[obj_id]=x
    

    def gripper_callback(self,msg:RobotCmd) :
        new_gripper_state=msg.activate_gripper
        if new_gripper_state and not self.is_gripping:
                self.is_gripping = True
                self.activ_gripper = True
        if not new_gripper_state:
            self.is_gripping = False
            self.activ_gripper = False
            self.active_obj = None

    def timer_callback(self):
       
        if self.is_gripping == True and not self.is_gripping:
            if len(self.obj_buffer) > 0:
                self.active_obj = self.obj_buffer.popleft()
                self.is_gripping= True
                self.get_logger().info(
                    f"Nach Greifprozess: neues Objekt aus Puffer aktiviert "
                    f"(obj_id={self.active_obj['obj_id']})"
                )
            return
            if not self.activ_gripper:
                return
            if self.active_obj is None:
                if len(self.obj_buffer)>0 :
                    self.active_obj = self.obj_buffer.popleft()
                    self.is_gripping = True    
                else:
                    return
            if  len(self.obj_buffer) <=0:
                self.active_obj= None
                self.get_logger().info(
                    f"Nach Greifprozess: kein objekt im puffer "
                )
            return
        
        if self.active_obj is None:
            return
        
        x_zum_Startzeitpunkt= self.active_obj["x"]
        vx = self.active_obj["vx"]
        y = self.active_obj["y"]
        z = self.active_obj["z"]
        obj_id = self.active_obj["obj_id"]
        time_logged= self.active_obj["zeitpunkt_logging"]
        time_now=time.time()

        
       

        # Innerhalb des Greifprozesses -> Position berechnen und publizieren
        if self.is_gripping: 
            dt = abs(time_now-time_logged)
            greifpunkt_x = vx * dt + x_zum_Startzeitpunkt
        

            pred_msg = PredictedPos()
            pred_msg.x = greifpunkt_x
            pred_msg.y = y
            pred_msg.z = z
            pred_msg.obj_id = float(obj_id)

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