# Static obstacles from field_layout.yaml on /obstacles/static.
# pose.position.z is the cylinder radius (meters).

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point

from drone_system.field_layout import obstacle_list


class ObstaclesNode(Node):
    def __init__(self):
        super().__init__('obstacles_node')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate_hz', 1.0)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.obstacles = []
        for o in obstacle_list():
            self.obstacles.append({
                'x': float(o['x']),
                'y': float(o['y']),
                'radius': float(o.get('radius', 1.0)),
                'name': str(o.get('name', 'obstacle')),
            })

        self.get_logger().info(f'Publishing {len(self.obstacles)} static obstacle(s)')
        self.pub = self.create_publisher(PoseArray, '/obstacles/static', 10)

        rate = float(self.get_parameter('publish_rate_hz').get_parameter_value().double_value)
        if rate <= 0.0:
            rate = 1.0
        self.create_timer(1.0 / rate, self.publish)

    def publish(self):
        arr = PoseArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = self.frame_id
        for o in self.obstacles:
            p = Pose()
            p.position = Point(x=o['x'], y=o['y'], z=o['radius'])
            p.orientation.w = 1.0
            arr.poses.append(p)
        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ObstaclesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
