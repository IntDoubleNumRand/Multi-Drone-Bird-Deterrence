# Relay /birds/raw → /birds/positions and /obstacles/static → /obstacles/positions.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        self.declare_parameter('birds_input_topic', '/birds/raw')
        self.declare_parameter('birds_output_topic', '/birds/positions')
        self.declare_parameter('obstacles_input_topic', '/obstacles/static')
        self.declare_parameter('obstacles_output_topic', '/obstacles/positions')

        birds_in = self.get_parameter('birds_input_topic').get_parameter_value().string_value
        birds_out = self.get_parameter('birds_output_topic').get_parameter_value().string_value
        obs_in = self.get_parameter('obstacles_input_topic').get_parameter_value().string_value
        obs_out = self.get_parameter('obstacles_output_topic').get_parameter_value().string_value

        self.birds_pub = self.create_publisher(PoseArray, birds_out, 10)
        self.obstacles_pub = self.create_publisher(PoseArray, obs_out, 10)
        self.create_subscription(PoseArray, birds_in, self.birds_cb, 10)
        self.create_subscription(PoseArray, obs_in, self.obstacles_cb, 10)

    def birds_cb(self, msg):
        self.birds_pub.publish(msg)

    def obstacles_cb(self, msg):
        self.obstacles_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
