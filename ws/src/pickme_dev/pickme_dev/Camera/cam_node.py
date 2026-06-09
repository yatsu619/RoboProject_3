import cv2
import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import CamData
from pickme_dev.Camera.cap_preprocessing import setup, process_frame
from pickme_dev.Camera.ml_detection import MLDetection


class CamNode(Node):

    #ros2 run pickme_dev cam_node
    def __init__(self):
        super().__init__('cam_node')
        self.publisher = self.create_publisher(CamData, '/CamData', 10)

        self.cap = cv2.VideoCapture(3)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        for _ in range(10):
            self.cap.read()

        self.H, self.ROI_X_MIN, self.ROI_X_MAX, self.ROI_Y_MIN, self.ROI_Y_MAX = setup(self.cap)
        self.ml = MLDetection()

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('CamNode gestartet')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        world_x, world_y, timestamp, _, _, _, _, _, _, _ = process_frame(
            frame, self.ROI_X_MIN, self.ROI_X_MAX, self.ROI_Y_MIN, self.ROI_Y_MAX, self.H)

        if world_x is not None:
            msg = CamData()
            label = self.ml.classify(frame)
            if label is None:
                return
            msg.obj_type = label
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