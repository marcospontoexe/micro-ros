import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PongNode(Node):
    def __init__(self):
        super().__init__('pong_node')
        self.subscription = self.create_subscription(String, '/ping', self.ping_callback, 10)
        self.publisher = self.create_publisher(String, '/pong', 10)
        self.pong_count = 0
        self.get_logger().info(f'ROS2 PongNode initialized')

    def ping_callback(self, msg):
        self.pong_count += 1
        self.get_logger().info(f'Received: {msg.data}')
        response = String()
        response.data = f'pong  #{self.pong_count} to \"{msg.data}\""'
        self.publisher.publish(response)

def main(args=None):
    rclpy.init(args=args)
    node = PongNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()