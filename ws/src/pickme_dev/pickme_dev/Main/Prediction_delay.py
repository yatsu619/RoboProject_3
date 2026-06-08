import rclpy
from rclpy.node import Node
from collections import deque

from ro45_portalrobot_interfaces.msg import PredictedPos, PredictedPosdelay


class DelayBufferNode(Node):
    def __init__(self):
        super().__init__("delay_buffer_node")

        
        self.greif_duration_sec = self.declare_parameter(
            "greif_duration_sec", 5.0
        )

        
        self.obj_buffer = deque()
        

    
        self.active_obj = None

        self.subscription = self.create_subscription(
            PredictedPosdelay,
            "/predicted_positiondelay",
            self.callback,
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
            f"DelayBufferNode gestartet - publiziert ab obj_zero bis "
            f"greif_duration_sec={self.greif_duration_sec:.2f}s"
        )

    def callback(self, msg: PredictedPosdelay):
        """
        Empfange PredictedPosdelay und puffere Objekte.
        Wenn kein aktives Objekt existiert, wird es sofort aktiv gesetzt.
        Andernfalls kommt es in den Puffer.
        """
       
        obj_zero_sec = float(msg.obj_zero.sec) + float(msg.obj_zero.nanosec) * 1e-9

        obj = {
            "vx": msg.vx,
            "y": msg.y,
            "z": msg.z,
            "obj_zero_sec": obj_zero_sec,
            "obj_id": msg.obj_id,
        }

        if self.active_obj is None:
            # Kein aktives Objekt → neues Objekt wird aktiv
            self.active_obj = obj
            self.get_logger().info(
                f"Neues Objekt aktiviert (obj_id={obj['obj_id']}), obj_zero_sec={obj['obj_zero_sec']:.3f}"
            )
        else:
            # Aktives Objekt existiert → neues Objekt puffern
            self.obj_buffer.append(obj)
            self.get_logger().info(
                f"Neues Objekt gepuffert (obj_id={obj['obj_id']}), "
                f"Pufferlänge={len(self.obj_buffer)}"
            )

    def timer_callback(self):
       
        if self.active_obj is None:
        
            if len(self.obj_buffer) > 0:
                self.active_obj = self.obj_buffer.popleft()
                self.get_logger().info(
                    f"Nach Greifprozess: neues Objekt aus Puffer aktiviert "
                    f"(obj_id={self.active_obj['obj_id']})"
                )
            return

        # Aktuelle Zeit in Sekunden
        now = self.get_clock().now()
        now_sec = float(now.nanoseconds) * 1e-9

        t_zero = self.active_obj["obj_zero_sec"]
        vx = self.active_obj["vx"]
        y = self.active_obj["y"]
        z = self.active_obj["z"]
        obj_id = self.active_obj["obj_id"]

        if now_sec < t_zero:
            # Noch vor dem Nullpunkt → nichts publizieren
            return

        dt = now_sec - t_zero

        if dt > self.greif_duration_sec:
            # Greifprozess vorbei → Objekt deaktivieren
            self.get_logger().info(
                f"Greifprozess vorbei (obj_id={obj_id}), dt={dt:.3f}s > "
                f"greif_duration_sec={self.greif_duration_sec:.3f}s"
            )
            self.active_obj = None
            return

        # Innerhalb des Greifprozesses → Position berechnen und publizieren
        x = vx * dt

        pred_msg = PredictedPos()
        pred_msg.x = x
        pred_msg.y = y
        pred_msg.z = z
        pred_msg.obj_id = obj_id

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