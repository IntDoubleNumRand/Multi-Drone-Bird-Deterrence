# RViz markers for drone, birds, and obstacles (/drone_marker, /bird_markers, …).

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point, PoseStamped, PoseArray
from std_msgs.msg import Int32, Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from drone_system.field_layout import (
    bird_count as layout_bird_count,
    load_layout,
    obstacle_count as layout_obstacle_count,
)
from drone_system.launch_params import DRONE_POSE_TOPICS

# Match birds_node STATE_*
_STATE_WANDER = 0
_STATE_FLEE = 1
_STATE_RECOVER = 2
_STATE_ENTER = 3

# Marker palette (RGB).
_DRONE_COLOR = (1.0, 0.55, 0.0)      # orange
_BIRD_GREEN = (0.20, 0.80, 0.25)     # roaming / outside walls
_BIRD_YELLOW = (1.0, 0.85, 0.10)     # selected, not chased off yet
_BIRD_RED = (0.90, 0.12, 0.12)       # chased off + selected
_BIRD_PURPLE = (0.62, 0.20, 0.85)    # chased off, not selected
_BIRD_GRAY = (0.45, 0.45, 0.45)       # completed / crossed off


class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        self.declare_parameter('pose_topic', DRONE_POSE_TOPICS[0])
        self.declare_parameter(
            'drone_ids',
            ['drone_1'],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        )
        self.declare_parameter(
            'drone_pose_topics',
            DRONE_POSE_TOPICS,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        )
        self.declare_parameter(
            'target_topics',
            ['/coordinator/target_index'],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        )
        self.declare_parameter(
            'z_state_topics',
            ['/drone/z_state'],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        )
        self.declare_parameter('birds_topic', '/birds/positions')
        self.declare_parameter('obstacles_topic', '/obstacles/positions')
        self.declare_parameter('map_frame', 'map')

        self._max_birds = layout_bird_count()
        self._obstacles = None
        self._last_bird_marker_count = 0
        self._drone_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.x = 0.0
        self.y = 0.0
        self.z = 5.0
        self._drone_pose = {}
        self._birds = None
        self._bird_states = []
        self._target_index = -1
        self._target_indices = {}
        self._selected = set()
        self._completed = set()
        layout = load_layout()
        self._field_xy = float(layout.get('field_xy', 30.0))
        self._world_xy = float(layout.get('limit_xy', self._field_xy))

        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        drone_ids = list(self.get_parameter('drone_ids').value)
        drone_pose_topics = list(self.get_parameter('drone_pose_topics').value)
        if not drone_pose_topics:
            drone_pose_topics = [pose_topic]
        drone_pose_topics = [str(t) for t in drone_pose_topics if str(t)]
        if not drone_pose_topics:
            drone_pose_topics = list(DRONE_POSE_TOPICS)
        if not drone_ids:
            drone_ids = [f'drone_{i+1}' for i in range(len(drone_pose_topics))]
        if len(drone_ids) < len(drone_pose_topics):
            for i in range(len(drone_ids), len(drone_pose_topics)):
                drone_ids.append(f'drone_{i+1}')
        self._drone_ids = drone_ids[:len(drone_pose_topics)]
        target_topics = [str(topic) for topic in self.get_parameter('target_topics').value if str(topic)]
        if not target_topics:
            target_topics = ['/coordinator/target_index']
        z_state_topics = [str(topic) for topic in self.get_parameter('z_state_topics').value if str(topic)]
        if not z_state_topics:
            z_state_topics = ['/drone/z_state']
        self._birds_topic = self.get_parameter('birds_topic').get_parameter_value().string_value

        for drone_id, topic in zip(self._drone_ids, drone_pose_topics):
            self._drone_pose[drone_id] = (0.0, 0.0, 5.0, False)
            self.create_subscription(
                PoseStamped,
                topic,
                self._make_pose_cb(drone_id),
                qos_profile_sensor_data,
            )
        self._obstacles_topic = self.get_parameter('obstacles_topic').get_parameter_value().string_value
        self.create_subscription(PoseArray, self._birds_topic, self.birds_cb, 10)
        self.create_subscription(PoseArray, self._obstacles_topic, self.obstacles_cb, 10)
        self.create_subscription(Int32MultiArray, '/birds/status', self.status_cb, 10)
        self.create_subscription(Int32MultiArray, '/central/assignments', self.assignments_cb, 10)
        self.create_subscription(Int32MultiArray, '/central/completed_targets', self.completed_cb, 10)
        for topic in target_topics:
            self.create_subscription(Int32, topic, self._make_target_cb(topic), 10)
        for topic in z_state_topics:
            self.create_subscription(Marker, topic, self._make_z_cb(topic), 10)

        self.pub = self.create_publisher(Marker, '/drone_marker', 10)
        self.drone_pub = self.create_publisher(MarkerArray, '/drone_markers', 10)
        self.bird_pub = self.create_publisher(MarkerArray, '/bird_markers', 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)
        self.boundary_pub = self.create_publisher(MarkerArray, '/boundary_markers', 10)
        self.create_timer(0.1, self.update)

        self.get_logger().info(f'RViz bird markers capped at {self._max_birds} (field_layout.yaml)')
        self.get_logger().info(f'RViz drone markers from topics: {drone_pose_topics}')
        self.get_logger().info(
            f'RViz boundaries: field ±{self._field_xy:.1f} m, bird world ±{self._world_xy:.1f} m'
        )

    def _refresh_target_index(self):
        active = [idx for idx in self._target_indices.values() if idx >= 0]
        self._target_index = active[0] if active else -1

    def _make_pose_cb(self, drone_id):
        def _cb(msg):
            x = float(msg.pose.position.x)
            y = float(msg.pose.position.y)
            z = float(msg.pose.position.z)
            self._drone_pose[drone_id] = (x, y, z, True)
            if drone_id == self._drone_ids[0]:
                self.x, self.y, self.z = x, y, z
            if msg.header.frame_id:
                self._drone_frame = msg.header.frame_id

        return _cb

    def status_cb(self, msg):
        self._bird_states = list(msg.data)

    def obstacles_cb(self, msg):
        self._obstacles = msg

    def _make_target_cb(self, topic):
        def _cb(msg):
            self._target_indices[topic] = int(msg.data)
            self._refresh_target_index()

        return _cb

    def assignments_cb(self, msg):
        # Flat [drone_idx, bird_idx, drone_idx, bird_idx, ...] from the dispatcher.
        sel = set()
        data = list(msg.data)
        for k in range(1, len(data), 2):
            if data[k] >= 0:
                sel.add(int(data[k]))
        self._selected = sel

    def completed_cb(self, msg):
        self._completed = {int(v) for v in msg.data if int(v) >= 0}

    def birds_cb(self, msg):
        self._birds = msg
        if len(msg.poses) > self._max_birds:
            self.get_logger().warn(
                f'Got {len(msg.poses)} birds on {self._birds_topic}; '
                f'showing {self._max_birds}. Run ./scripts/reset.sh if you see duplicates.',
                throttle_duration_sec=5.0,
            )

    def _make_z_cb(self, topic):
        def _cb(msg):
            if msg.pose.position.z == 0.0:
                return
            if topic.endswith('/drone_1/z_state') or topic.endswith('/drone/z_state'):
                self.z = msg.pose.position.z

        return _cb

    def _delete_bird_marker(self, frame, stamp, marker_id):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = stamp
        m.ns = 'birds'
        m.id = marker_id
        m.action = Marker.DELETE
        return m

    def _bird_style(self, i, state, x, y):
        #   outside walls / roaming        -> green
        #   inside + selected + not fleeing -> yellow
        #   fleeing + selected              -> red (actively chased off)
        #   fleeing + not selected          -> purple (chased off, idle)
        chased = state == _STATE_FLEE
        selected = i in self._selected or i == self._target_index
        inside = abs(x) <= self._field_xy and abs(y) <= self._field_xy
        completed = i in self._completed or not inside
        if completed:
            color = _BIRD_GRAY
        elif chased and selected:
            color = _BIRD_RED
        elif chased:
            color = _BIRD_PURPLE
        elif selected and inside:
            color = _BIRD_YELLOW
        else:
            color = _BIRD_GREEN
        scale = 0.55 if completed else (0.9 if selected else 0.8)
        return color[0], color[1], color[2], 1.0, scale

    def _completed_marker(self, frame, stamp, marker_id, position):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = stamp
        m.ns = 'completed_targets'
        m.id = marker_id
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = float(position.x)
        m.pose.position.y = float(position.y)
        m.pose.position.z = float(position.z) + 0.9
        m.pose.orientation.w = 1.0
        m.scale.z = 1.2
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0
        m.text = 'X'
        return m

    def _square_line_marker(self, frame, stamp, marker_id, ns, half_xy, z, rgba, width):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = stamp
        m.ns = ns
        m.id = marker_id
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = float(width)
        m.color.r = float(rgba[0])
        m.color.g = float(rgba[1])
        m.color.b = float(rgba[2])
        m.color.a = float(rgba[3])
        h = float(half_xy)
        zz = float(z)
        m.points = [
            Point(x=-h, y=-h, z=zz),
            Point(x=h, y=-h, z=zz),
            Point(x=h, y=h, z=zz),
            Point(x=-h, y=h, z=zz),
            Point(x=-h, y=-h, z=zz),
        ]
        return m

    def _boundary_label(self, frame, stamp, marker_id, text, x, y, z, rgba):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = stamp
        m.ns = 'boundary_labels'
        m.id = marker_id
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.pose.orientation.w = 1.0
        m.scale.z = 1.0
        m.color.r = float(rgba[0])
        m.color.g = float(rgba[1])
        m.color.b = float(rgba[2])
        m.color.a = float(rgba[3])
        m.text = text
        return m

    def _publish_boundaries(self, frame, stamp):
        arr = MarkerArray()
        arr.markers.append(
            self._square_line_marker(
                frame,
                stamp,
                0,
                'field_boundary',
                self._field_xy,
                0.15,
                (0.10, 0.70, 1.0, 1.0),
                0.18,
            )
        )
        if self._world_xy > self._field_xy:
            arr.markers.append(
                self._square_line_marker(
                    frame,
                    stamp,
                    1,
                    'world_boundary',
                    self._world_xy,
                    0.1,
                    (0.55, 0.55, 0.55, 0.45),
                    0.08,
                )
            )
        arr.markers.append(
            self._boundary_label(
                frame,
                stamp,
                0,
                'field fence',
                -self._field_xy,
                self._field_xy,
                1.0,
                (0.10, 0.70, 1.0, 1.0),
            )
        )
        self.boundary_pub.publish(arr)

    def update(self):
        now = self.get_clock().now().to_msg()
        default_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        drone_frame = self._drone_frame or default_frame
        self._publish_boundaries(default_frame, now)

        # Backward-compatible Marker topic: publish one marker per drone.
        # For multi-drone RViz, prefer /drone_markers (MarkerArray).
        for idx, drone_id in enumerate(self._drone_ids):
            px, py, pz, valid = self._drone_pose.get(drone_id, (0.0, 0.0, 5.0, False))
            if not valid:
                continue
            marker = Marker()
            marker.header.frame_id = drone_frame
            marker.header.stamp = now
            marker.ns = 'drone'
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(px)
            marker.pose.position.y = float(py)
            marker.pose.position.z = float(pz)
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 0.4
            marker.color.r = _DRONE_COLOR[0]
            marker.color.g = _DRONE_COLOR[1]
            marker.color.b = _DRONE_COLOR[2]
            marker.color.a = 1.0
            self.pub.publish(marker)

        drone_arr = MarkerArray()
        for idx, drone_id in enumerate(self._drone_ids):
            px, py, pz, valid = self._drone_pose.get(drone_id, (0.0, 0.0, 5.0, False))
            if not valid:
                continue
            d = Marker()
            d.header.frame_id = drone_frame
            d.header.stamp = now
            d.ns = 'drones'
            d.id = idx
            d.type = Marker.SPHERE
            d.action = Marker.ADD
            d.pose.position.x = float(px)
            d.pose.position.y = float(py)
            d.pose.position.z = float(pz)
            d.scale.x = 1.0
            d.scale.y = 1.0
            d.scale.z = 0.4
            d.color.r = _DRONE_COLOR[0]
            d.color.g = _DRONE_COLOR[1]
            d.color.b = _DRONE_COLOR[2]
            d.color.a = 1.0
            drone_arr.markers.append(d)
        self.drone_pub.publish(drone_arr)

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
                r, g, bl, a, scale = self._bird_style(i, st, p.position.x, p.position.y)
                b.scale.x = scale
                b.scale.y = scale
                b.scale.z = scale
                b.color.r = float(r)
                b.color.g = float(g)
                b.color.b = float(bl)
                b.color.a = float(a)
                arr.markers.append(b)
                inside = abs(p.position.x) <= self._field_xy and abs(p.position.y) <= self._field_xy
                if i in self._completed or not inside:
                    arr.markers.append(self._completed_marker(bird_frame, now, i, p.position))

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
                # position.z = top height; orientation.z = radius (>0 cylinder);
                # orientation.x/y = box half extents.
                height = max(float(p.position.z), 2.0)
                radius = float(p.orientation.z)
                half_x = float(p.orientation.x)
                half_y = float(p.orientation.y)
                o = Marker()
                o.header.frame_id = obs_frame
                o.header.stamp = now
                o.ns = 'obstacles'
                o.id = i
                o.action = Marker.ADD
                o.pose.position.x = p.position.x
                o.pose.position.y = p.position.y
                o.pose.position.z = height / 2.0
                o.pose.orientation.w = 1.0
                if radius > 0.0:
                    o.type = Marker.CYLINDER
                    o.scale.x = radius * 2.0
                    o.scale.y = radius * 2.0
                else:
                    o.type = Marker.CUBE
                    o.scale.x = max(half_x * 2.0, 0.5)
                    o.scale.y = max(half_y * 2.0, 0.5)
                o.scale.z = height
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
