# Static obstacles from field_layout.yaml on /obstacles/static.
# Each obstacle is packed into a Pose:
#   position.x, position.y = center (m)
#   position.z            = top height above ground (m)
#   orientation.z         = radius (m), > 0 => cylinder
#   orientation.x         = half extent X (m), box only
#   orientation.y         = half extent Y (m), box only

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point

from drone_system.field_layout import obstacle_list


def _normalize(o):
    """Return (name, cx, cy, top_z, radius, half_x, half_y). Cylinder => radius>0."""
    name = str(o.get('name', 'obstacle'))
    height = float(o.get('height', 0.0))
    shape = str(o.get('shape', '')).strip().lower()
    is_box = shape == 'box' or ('x_min' in o and 'x_max' in o)
    if is_box:
        x_min = float(o['x_min'])
        x_max = float(o['x_max'])
        y_min = float(o['y_min'])
        y_max = float(o['y_max'])
        cx = 0.5 * (x_min + x_max)
        cy = 0.5 * (y_min + y_max)
        half_x = abs(x_max - x_min) / 2.0
        half_y = abs(y_max - y_min) / 2.0
        return name, cx, cy, height, 0.0, half_x, half_y
    cx = float(o['x'])
    cy = float(o['y'])
    radius = float(o.get('radius', 1.0))
    return name, cx, cy, height, radius, 0.0, 0.0


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
            name, cx, cy, top_z, radius, half_x, half_y = _normalize(o)
            self.obstacles.append({
                'name': name,
                'cx': cx,
                'cy': cy,
                'top_z': top_z,
                'radius': radius,
                'half_x': half_x,
                'half_y': half_y,
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
            p.position = Point(x=o['cx'], y=o['cy'], z=o['top_z'])
            p.orientation.w = 1.0
            p.orientation.z = o['radius']
            p.orientation.x = o['half_x']
            p.orientation.y = o['half_y']
            arr.poses.append(p)
        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ObstaclesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
