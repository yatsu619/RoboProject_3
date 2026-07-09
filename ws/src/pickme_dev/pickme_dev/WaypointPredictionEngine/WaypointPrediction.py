
import rclpy
from rclpy.node import Node
from collections import Counter, deque
from ro45_portalrobot_interfaces.msg import CamData, PredictedPosdelay
import statistics
from pickme_dev.WaypointPredictionEngine.Predic_logic import BeltVelocityTracker

class WaypointPreditionNode(Node):
    """
    Der Knoten ermittelt die Förderbandgeschwindigkeit, filtert die
    y-Position eines Objekts und bestimmt die wahrscheinlichste Objekt-ID.
    Sobald ein Objekt den Erfassungsbereich der Kamera verlässt, werden die
    berechneten Objektinformationen veröffentlicht.
    """
    def __init__(self):
        """
        Initialisiert Publisher, Subscriber und interne Verarbeitungsstrukturen.
        Außerdem werden alle Variablen und Puffer angelegt, die für die
        Geschwindigkeitsbestimmung, Positionsfilterung und Objektzuordnung
        benötigt werden.
        """
        super().__init__('WaypointPredition_node')
        self.get_logger().info('WaypointPredictionNode gestartet.')
        self.speed_calculator= BeltVelocityTracker()
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
        

        self.queue_y= []
        self.median_vx=None
        
        self.buffer = []             
          
        self.window_size = 32
        self.last_obj_type=None


        self.conveyor_belt_height = 0.0        
        self.threshold = 0.06
        self.grenze = 0.04
        self.min_Elemente_queue= 10 
        self.rejekted_objekt_type=0


    def berechne_geschwindigkeit(self):
        """
        Verarbeitet die zuletzt empfangenen Kameradaten.
        Die Methode bestimmt die Förderbandgeschwindigkeit, filtert die
        y-Position des Objekts und erkennt, wann ein Objekt den zuverlässigen
        Erfassungsbereich der Kamera verlassen hat. Anschließend werden die
        berechneten Daten veröffentlicht.
        """
        if self.x_aktuell is None or self.time_aktuell is None:
            self.get_logger().debug(f"Block 1: Keine Daten vom Callback | x_aktuell = {self.x_aktuell} | time_aktuell = {self.time_aktuell}")
            return
    
        if self.x_alt is None or self.time_alt is None:
            self.get_logger().debug(f"Block 2: Initialisierung | x_alt gesetzt auf {self.x_aktuell:.4f}")
            self.aktualiesiere_Werte()
            return
        

        self.queue_y.append(self.y_aktuell)
          
            
        self.median_vx=self.speed_calculator.feed(self.x_aktuell,self.time_aktuell)
        self.x_logged=self.x_aktuell
        if self.median_vx is None:
            return
        
        if self.x_aktuell < self.grenze:
            # Object is assumed to have left the camera's reliable detection area.
            self.x_logged = self.x_aktuell
            self.get_logger().debug(f"Block 5: Objekt verlässt Band | x_aktuell = {self.x_aktuell:.4f} < grenze {self.grenze}")

            self.median_y = statistics.median(self.queue_y)
            result = self.trigger()

            if result is None:
                self.obj_type = self.last_obj_type
            else:
                self.obj_type = result
                self.last_obj_type = self.obj_type

            self.get_logger().debug(f"Block 5: result={result} | obj_type={self.obj_type} | last_obj_type={self.last_obj_type}")

            try:
                self.publish()
            except Exception as e:
                self.get_logger().error(f"Publish fehlgeschlagen: {e}")

            self.get_logger().debug(f"Block 5: Publish | median_vx = {self.median_vx:.4f} | median_y = {self.median_y:.4f}")

            self.queue_y = []
            self.x_alt = None
            self.time_alt = None
    

    def robot_pos_callback(self, msg: CamData):
        """
        Speichert die aktuelle Kameramessung.
          Args:msg: Kameranachricht mit Objektposition, Zeitstempel und Objekt-Type.
        """
        self.x_aktuell=msg.x
        self.y_aktuell=msg.y
        self.time_aktuell=msg.timestamp
        self.add_id(msg.obj_type)
        
    
    
    def aktualiesiere_Werte(self):
        """Übernimmt die aktuelle Messung als Referenz für die nächste Berechnung."""
        self.x_alt=self.x_aktuell
        self.time_alt=self.time_aktuell
    
    def publish(self):
        """
        Veröffentlicht die berechneten Objektinformationen.
        Die Nachricht enthält die bestimmte Förderbandgeschwindigkeit,
        die gefilterte Objektposition sowie den validierte Objekt-Type.
        """

        pred_msg = PredictedPosdelay()
        
        pred_msg.vx = self.median_vx
        pred_msg.y = self.median_y
        pred_msg.z = self.conveyor_belt_height 
        pred_msg.x= self.x_logged
        pred_msg.obj_id = self.obj_type

        self.publisher_prediction.publish(pred_msg)
        self.get_logger().info(f'Prediction publiziert | median_vx = {self.median_vx:.4f}| median_y = {self.median_y:.4f} | Aktuelles_X = {self.x_aktuell:.4f} | Obj_type = {self.obj_type}')
        
    def berechnung_Geschwindigkeit(self,akt_x,last_x,dt):
        """
        Berechnet die Momentangeschwindigkeit aus zwei Positionsmessungen.
        Args:
            akt_x: Aktuelle x-Position.
            last_x: Vorherige x-Position.
            dt: Zeitdifferenz zwischen beiden Messungen.
        Returns:
            Berechnete Momentangeschwindigkeit.
        """
        vx=(akt_x-last_x)/dt
        return vx 

    def time_diff(self, t1, t2) -> float:
        """
    Berechnet die absolute Zeitdifferenz zweier Zeitstempel.
    Args:
        t1: Erster Zeitstempel.
        t2: Zweiter Zeitstempel.
    Returns:
        Absolute Zeitdifferenz.
    """
        
        return abs(t2 - t1)
        
    def add_id(self, obj_id):
        """
    Fügt eine erkannte Objekt-ID dem Beobachtungspuffer hinzu.
    Der Puffer besitzt eine feste maximale Größe. Ältere Einträge werden
    automatisch entfernt, sobald die maximale Anzahl erreicht ist.
    Args:
        obj_id: Erkannten Objekt-type.
    """
        self.buffer.append(obj_id)
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)

    def trigger(self):
        """
        Ermittelt die wahrscheinlichste Objekt-ID.
        Da mehrere Objekte gleichzeitig erkannt werden können und die Kamera
        deren IDs in wechselnder Reihenfolge liefert, wird die am häufigsten
        erkannte ID als wahrscheinlichstes Objekt bestimmt. Anschließend werden
        alle Vorkommen dieser ID aus dem Puffer entfernt.
        Returns:
            Den validierte Objekt-Type oder ``None``, falls keine Beobachtungen
            vorhanden sind.
        """
        if not self.buffer:
            return None

        counter = Counter(self.buffer)
        best_id, _ = counter.most_common(1)[0]
        # The most frequently observed Type most likely belongs to the object that remained in the camera view the longest.
        self.buffer = [x for x in self.buffer if x != best_id]
        return best_id
        

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