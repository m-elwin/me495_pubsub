import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose

class PubsubNode(Node):
    """
    Simple node for testing publishing and subscribing.

    Publishes
    ---------
    cmd_vel : geometry_msgs/msg/Twist - velocity to control the turtle

    Subscribes
    ----------
    pose : turtlesim/msg/Pose - the pose of the turtle robot

    """

    def __init__(self):
        super().__init__("pubsub")
        self.get_logger().info("Pubsub Node")
        self._tmr = self.create_timer(0.15, self.timer_callback)
        self._pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._sub = self.create_subscription(Pose, "pose", self.pose_callback, 10)
        self._velocity = 2.0

    def timer_callback(self):
        self.get_logger().debug("Timer!")
        self._pub.publish(Twist(linear=Vector3(x=self._velocity)))

    def pose_callback(self, pose):
        """Update the pose of the robot"""
        if pose.x > 7 and self._velocity > 0:
            self._velocity *= -1
        elif pose.x < 2 and self._velocity < 0:
            self._velocity *= -1


def main(args=None):
    rclpy.init(args=args)
    pubsub_node = PubsubNode()
    rclpy.spin(pubsub_node)
    pubsub_node.destroy_node()
    rclpy.shutdown()
