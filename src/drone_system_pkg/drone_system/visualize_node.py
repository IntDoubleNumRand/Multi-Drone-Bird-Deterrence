# RViz markers for drone, birds, and obstacles (/drone_marker, /bird_markers, …).

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Int32, Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from drone_system.field_layout import bird_count as layout_bird_count, obstacle_count as layout_obstacle_count

# Match birds_node STATE_*
_STATE_WANDER = 0
_STATE_FLEE = 1
_STATE_RECOVER = 2
_STATE_ENTER = 3

# Bird colors by state (wander / flee / recover / enter).
_BIRD_COLORS = {
    _STATE_WANDER: (0.95, 0.62, 0.12),
    _STATE_FLEE: (0.85, 0.15, 0.15),
    _STATE_RECOVER: (0.20, 0.70, 0.95),
    _STATE_ENTER: (0.30, 0.80, 0.30),
}
_UNKNOWN_BIRD_COLOR = (0.70, 0.70, 0.70)


class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('birds_topic', '/birds/positions')
        self.declare_parameter('obstacles_topic', '/obstacles/positions')
        self.declare_parameter('map_frame', 'map')

        self._max_birds = layout_bird_count()
        self._obstacles = None
        self._last_bird_marker_count = 0
        self._drone_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self._pose_valid = False

        self.x = 0.0
        self.y = 0.0
        self.z = 5.0
        self._birds = None
        self._bird_states = []
        self._target_index = -1

        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self._birds_topic = self.get_parameter('birds_topic').get_parameter_value().string_value

        self.create_subscription(
            PoseStamped,
            pose_topic,
            self.pose_cb,
            qos_profile_sensor_data,
        )
        self._obstacles_topic = self.get_parameter('obstacles_topic').get_parameter_value().string_value
        self.create_subscription(PoseArray, self._birds_topic, self.birds_cb, 10)
        self.create_subscription(PoseArray, self._obstacles_topic, self.obstacles_cb, 10)
        self.create_subscription(Int32MultiArray, '/birds/status', self.status_cb, 10)
        self.create_subscription(Int32, '/coordinator/target_index', self.target_cb, 10)
        self.create_subscription(Marker, '/drone/z_state', self.z_cb, 10)

        self.pub = self.create_publisher(Marker, '/drone_marker', 10)
        self.bird_pub = self.create_publisher(MarkerArray, '/bird_markers', 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)
        self.create_timer(0.1, self.update)

        self.get_logger().info(f'RViz bird markers capped at {self._max_birds} (field_layout.yaml)')

    def pose_cb(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        self._pose_valid = True
        if msg.header.frame_id:
            self._drone_frame = msg.header.frame_id

    def status_cb(self, msg):
        self._bird_states = list(msg.data)

    def obstacles_cb(self, msg):
        self._obstacles = msg

    def target_cb(self, msg):
        self._target_index = int(msg.data)

    def birds_cb(self, msg):
        self._birds = msg
        if len(msg.poses) > self._max_birds:
            self.get_logger().warn(
                f'Got {len(msg.poses)} birds on {self._birds_topic}; '
                f'showing {self._max_birds}. Run ./scripts/reset.sh if you see duplicates.',
                throttle_duration_sec=5.0,
            )

    def z_cb(self, msg):
        if msg.pose.position.z != 0.0:
            self.z = msg.pose.position.z

    def _delete_bird_marker(self, frame, stamp, marker_id):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = stamp
        m.ns = 'birds'
        m.id = marker_id
        m.action = Marker.DELETE
        return m

    def _bird_style(self, state, is_target):
        base_r, base_g, base_b = _BIRD_COLORS.get(state, _UNKNOWN_BIRD_COLOR)
        alpha = 0.95
        scale = 0.35

        if state == _STATE_RECOVER:
            alpha = 0.75
            scale = 0.30
        elif state == _STATE_ENTER:
            alpha = 0.80
            scale = 0.32

        if is_target:
            alpha = 1.0
            scale = 0.48
            base_r = min(1.0, base_r + 0.10)
            base_g = max(0.0, base_g - 0.05)
            base_b = max(0.0, base_b - 0.05)

        return base_r, base_g, base_b, alpha, scale

    def update(self):
        if not self._pose_valid:
            return

        now = self.get_clock().now().to_msg()
        default_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        drone_frame = self._drone_frame or default_frame

        marker = Marker()
        marker.header.frame_id = drone_frame
        marker.header.stamp = now
        marker.ns = 'drone'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = self.z
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 0.7
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.pub.publish(marker)

        arr = MarkerArray()
        n_show = 0
        if self._birds and self._birds.poses:
            n_show = min(len(self._birds.poses), self._max_birds)
            bird_frame = self._birds.header.frame_id or drone_frame
            for i in range(n_show):
                p = self._birds.poses[i]
                b = Marker()
                b.header.frame_id = bird_frame
                b.header.stamp = now
                b.ns = 'birds'
                b.id = i
                b.type = Marker.SPHERE
                b.action = Marker.ADD
                b.pose.position = p.position
                b.pose.orientation.w = 1.0
                st = self._bird_states[i] if i < len(self._bird_states) else 0
                is_target = (i == self._target_index)
                r, g, bl, a, scale = self._bird_style(st, is_target)
                b.scale.x = scale
                b.scale.y = scale
                b.scale.z = scale
                b.color.r = float(r)
                b.color.g = float(g)
                b.color.b = float(bl)
                b.color.a = float(a)
                arr.markers.append(b)

        # DELETE markers for bird IDs we no longer publish (stale RViz ghosts).
        for i in range(n_show, max(self._last_bird_marker_count, self._max_birds)):
            arr.markers.append(self._delete_bird_marker(drone_frame, now, i))

        self._last_bird_marker_count = n_show
        self.bird_pub.publish(arr)

        obs_arr = MarkerArray()
        n_obs = layout_obstacle_count()
        if self._obstacles and self._obstacles.poses:
            obs_frame = self._obstacles.header.frame_id or drone_frame
            for i, p in enumerate(self._obstacles.poses[:n_obs]):
                radius = max(float(p.position.z), 0.5)
                o = Marker()
                o.header.frame_id = obs_frame
                o.header.stamp = now
                o.ns = 'obstacles'
                o.id = i
                o.type = Marker.CYLINDER
                o.action = Marker.ADD
                o.pose.position.x = p.position.x
                o.pose.position.y = p.position.y
                o.pose.position.z = 1.0
                o.pose.orientation.w = 1.0
                o.scale.x = radius * 2.0
                o.scale.y = radius * 2.0
                o.scale.z = 2.0
                o.color.r = 0.45
                o.color.g = 0.45
                o.color.b = 0.45
                o.color.a = 0.85
                obs_arr.markers.append(o)
        self.obstacle_pub.publish(obs_arr)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
