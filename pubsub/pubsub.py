import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

class PubsubNode(Node):
    def __init__(self):
        super().__init__("pubsub")
        self.get_logger().info("Pubsub Node")
        self._tmr = self.create_timer(0.15, self.timer_callback)
        self._pub = self.create_publisher(Twist, "cmd_vel", 10)

    def timer_callback(self):
        self.get_logger().debug("Timer!")
        self._pub.publish(Twist(linear=Vector3(x=0.2)))


def main(args=None):
    rclpy.init(args=args)
    pubsub_node = PubsubNode()
    rclpy.spin(pubsub_node)
    pubsub_node.destroy_node()
    rclpy.shutdown()
