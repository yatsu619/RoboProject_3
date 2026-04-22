import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CamNode(Node):
    def __init__(self):
        super().__init__('cam_node')
        self.publisher = self.create_publisher(String, '/CamData', 10)
        self.get_logger().info('CamNode gestartet')



def main(args=None):
    rclpy.init(args = args)
    node = CamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
