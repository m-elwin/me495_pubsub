"""Solutions to https://nu-msr.github.io/ros_notes/ros2/activity/pubsub_activity.html."""
from geometry_msgs.msg import Twist, Vector3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Spawn


class PubsubNode(Node):
    """
    Simple node for testing publishing and subscribing.

    Publishes
    ---------
    cmd_vel : geometry_msgs/msg/Twist - velocity to control the turtle

    Subscribes
    ----------
    pose : turtlesim/msg/Pose - the pose of the turtle robot

    Parameters
    ----------
    xmin : float64 - the minimum x coordinate
    xmax : float64 - the maximum x coordinate

    """

    def __init__(self):
        super().__init__('pubsub')
        self.get_logger().info('Pubsub Node')
        self._tmr = self.create_timer(0.15, self.timer_callback)
        self._pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._sub = self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self._velocity = 2.0
        self.declare_parameter('xmin', 2.0)
        self._xmin = self.get_parameter('xmin').value
        self.declare_parameter('xmax', 7.0)
        self._xmax = self.get_parameter('xmax').value

        self.get_logger().debug(f'Xmin: {self._xmin} Xmax: {self._xmax}')

        spawn_client = self.create_client(Spawn, 'spawn')
        if not spawn_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('Failed to find spawn service.')
        rclpy.spin_until_future_complete(self,
                                         spawn_client.call_async(
                                             Spawn.Request(x=self._xmin, y=5.54)))
        rclpy.spin_until_future_complete(self,
                                         spawn_client.call_async(
                                             Spawn.Request(x=self._xmax, y=5.54)))

    def timer_callback(self):
        """Publish the twist."""
        self.get_logger().debug('Timer!')
        self._pub.publish(Twist(linear=Vector3(x=self._velocity)))

    def pose_callback(self, pose):
        """Update the pose of the robot."""
        if pose.x > self._xmax and self._velocity > 0:
            self._velocity *= -1.0
        elif pose.x < self._xmin and self._velocity < 0:
            self._velocity *= -1.0


def main(args=None):
    rclpy.init(args=args)
    pubsub_node = PubsubNode()
    rclpy.spin(pubsub_node)
    pubsub_node.destroy_node()
    rclpy.shutdown()
