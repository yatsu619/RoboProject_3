import cv2
import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import CamData
from pickme_dev.Camera.cap_preprocessing import setup, process_frame
from pickme_dev.Camera.ml_detection import MLDetection

"""
ROS2-Knoten der Kamera-Pipeline.
 
Führt Positionsbestimmung (cap_preprocessing) und Klassifikation
(ml_detection) für jedes Kamerabild zusammen und veröffentlicht die
erkannten Objekte (Typ, Weltkoordinate, Zeitstempel) auf dem Topic
/CamData.
"""

class CamNode(Node):
    def __init__(self):
        """
        Initialisiert den Knoten: öffnet die Kamera, führt die
        Kalibrierung durch, lädt das Klassifikationsmodell und startet
        den Timer für die wiederkehrende Verarbeitung.
        """
        super().__init__('cam_node')
        self.publisher = self.create_publisher(CamData, '/CamData', 10)
        self.cap = cv2.VideoCapture(2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        for _ in range(10):
            self.cap.read()
        self.H = setup(self.cap)
        self.ml = MLDetection()
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('CamNode gestartet')

    def timer_callback(self):
        """
        Wird periodisch aufgerufen: liest ein neues Kamerabild ein,
        bestimmt Objektpositionen und -klassen, prüft, ob beide Anzahlen
        übereinstimmen, und veröffentlicht bei Übereinstimmung für jedes
        Objekt eine CamData-Nachricht.
        """
        ret, frame = self.cap.read()
        if not ret:
            return
        objekte = process_frame(frame, self.H)
        labels = self.ml.classify(frame)

        if len(objekte) != len(labels):
            self.get_logger().error(f'Anzahl Objekte ({len(objekte)}) != Anzahl Labels ({len(labels)})')
            return
        
        else:
            for i, (world_x, world_y, timestamp) in enumerate(objekte):
                if i >= len(labels):
                    break
                #if labels[i] == 0:
                 #   return
                msg = CamData()
                msg.obj_type = labels[i]
                msg.x = world_x
                msg.y = world_y
                msg.timestamp = timestamp
                self.publisher.publish(msg)
                self.get_logger().info(f'Publiziert: {msg.obj_type}, {msg.x:.4f}, {msg.y:.4f}')

def main(args=None):
    rclpy.init(args=args)
    node = CamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()