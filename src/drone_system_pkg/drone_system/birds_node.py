# Simulated birds on /birds/raw (PoseArray) plus /birds/status.

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseArray, Pose, Point, PoseStamped
from std_msgs.msg import Bool, Int32MultiArray

from drone_system.bird_behavior import BirdAgent, BirdStepContext
from drone_system.bird_behavior import STATE_RECOVER
from drone_system.bird_behavior import PROFILE_EASY, PROFILE_HARD
from drone_system.field_layout import load_layout, bird_count as layout_bird_count
from drone_system.launch_params import DRONE_POSE_TOPICS


class BirdsNode(Node):
    def __init__(self):
        super().__init__('birds_node')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        layout = load_layout()
        default_n = layout_bird_count()
        self.declare_parameter('bird_count', default_n)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('pose_topic', DRONE_POSE_TOPICS[0])
        self.declare_parameter(
            'drone_pose_topics',
            DRONE_POSE_TOPICS,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        )

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        n = int(self.get_parameter('bird_count').get_parameter_value().integer_value)
        max_n = layout_bird_count()
        if n > max_n:
            self.get_logger().warn(f'bird_count={n} > layout ({max_n}); clamping to layout')
            n = max_n
        if n < 1:
            n = 1

        self.world_xy = float(layout.get('limit_xy', 15.0))
        self.field_xy = float(layout.get('field_xy', layout.get('bird_detection_xy', 10.0)))
        self.cruise_speed = float(layout.get('bird_cruise_speed', 0.55))
        self.escape_speed = float(layout.get('bird_escape_speed', 0.85))
        self.recover_s = float(layout.get('bird_recover_s', 4.0))
        self.hard_reentry_s = float(layout.get('hard_bird_reentry_s', 25.0))
        self.wander_margin = float(layout.get('bird_wander_margin', 1.5))
        self.wall_margin_easy = float(
            layout.get('bird_world_wall_margin_easy', self.wander_margin)
        )
        self.wall_margin_hard = float(layout.get('bird_world_wall_margin_hard', 0.35))
        self.hard_wall_ring = float(layout.get('bird_hard_wall_ring_m', 1.2))
        self.wander_arrival = float(layout.get('bird_wander_arrival_m', 1.2))
        self.spawn_margin = float(layout.get('bird_spawn_margin', 0.5))
        self.demo_intro_s = float(layout.get('bird_demo_intro_s', 0.0))
        self.fixed_z = float(layout.get('bird_altitude_m', 4.0))

        self._chased_mask = [0] * n
        self._prev_chased_mask = [0] * n
        self._drone_positions = {}

        self.birds = []
        layout_birds = layout.get('birds', [])
        default_profiles = [PROFILE_EASY, PROFILE_HARD]
        for i in range(n):
            if i < len(layout_birds):
                b = layout_birds[i]
                sx = float(b['x'])
                sy = float(b['y'])
                sz = float(b.get('z', self.fixed_z))
                profile = str(b.get('profile', default_profiles[i % len(default_profiles)]))
            else:
                b = layout_birds[-1]
                sx, sy = float(b['x']), float(b['y'])
                sz = float(b.get('z', self.fixed_z))
                profile = str(b.get('profile', PROFILE_EASY))
            ox, oy = sx, sy
            sx, sy = self._clamp_spawn(sx, sy)
            if abs(ox) > self.field_xy or abs(oy) > self.field_xy:
                self.get_logger().warn(
                    f'Bird {i} layout spawn ({ox:.1f}, {oy:.1f}) outside field; using ({sx:.1f}, {sy:.1f})'
                )
            self.birds.append(BirdAgent(sx, sy, sz, profile=profile, intro_recover_s=self.demo_intro_s))

        intro_msg = f', intro_recover={self.demo_intro_s:.0f}s' if self.demo_intro_s > 0 else ''
        self.get_logger().info(
            f'{len(self.birds)} birds, field_xy=±{self.field_xy}, world_xy=±{self.world_xy}{intro_msg}'
        )

        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        pose_topics = list(self.get_parameter('drone_pose_topics').value)
        if not pose_topics:
            pose_topics = [pose_topic]
        pose_topics = [str(topic) for topic in pose_topics if str(topic)]
        if not pose_topics:
            pose_topics = list(DRONE_POSE_TOPICS)
        self.pub = self.create_publisher(PoseArray, '/birds/raw', 10)
        self.status_pub = self.create_publisher(Int32MultiArray, '/birds/status', 10)
        self.recovery_pub = self.create_publisher(Bool, '/birds/in_recovery', 10)
        self.create_subscription(Int32MultiArray, '/birds/chased_mask', self._chased_cb, 10)
        for topic in pose_topics:
            self._drone_positions[topic] = (0.0, 0.0, False)
            self.create_subscription(
                PoseStamped,
                topic,
                self._make_drone_pose_cb(topic),
                qos_profile_sensor_data,
            )
        self.get_logger().info(f'Bird threats tracked from drone poses: {pose_topics}')

        rate = float(self.get_parameter('publish_rate_hz').get_parameter_value().double_value)
        if rate <= 0.0:
            rate = 10.0
        self.dt = 1.0 / rate
        self.create_timer(1.0 / rate, self.update)

    def _clamp_spawn(self, x, y):
        lim = max(0.5, self.field_xy - self.spawn_margin)
        return (
            max(-lim, min(lim, x)),
            max(-lim, min(lim, y)),
        )

    def _chased_cb(self, msg):
        self._chased_mask = [1 if int(v) > 0 else 0 for v in msg.data]

    def _make_drone_pose_cb(self, topic):
        def _cb(msg):
            self._drone_positions[topic] = (
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                True,
            )

        return _cb

    def _nearest_drone_pose(self, bird_x, bird_y):
        best = None
        best_d2 = float("inf")
        for x, y, valid in self._drone_positions.values():
            if not valid:
                continue
            d2 = (x - bird_x) ** 2 + (y - bird_y) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best = (x, y)
        if best is None:
            return False, 0.0, 0.0
        return True, best[0], best[1]

    def update(self):
        arr = PoseArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = self.frame_id

        any_recovery = False
        status_msg = Int32MultiArray()
        for i, b in enumerate(self.birds):
            wall_margin = (
                self.wall_margin_hard if b.profile == PROFILE_HARD else self.wall_margin_easy
            )
            chased = bool(self._chased_mask[i]) if i < len(self._chased_mask) else False
            was_chased = bool(self._prev_chased_mask[i]) if i < len(self._prev_chased_mask) else False
            drone_pose_valid, drone_x, drone_y = self._nearest_drone_pose(b.x, b.y)
            ctx = BirdStepContext(
                dt=self.dt,
                chased=chased,
                just_unchased=was_chased and not chased,
                field_xy=self.field_xy,
                world_xy=self.world_xy,
                wall_margin=wall_margin,
                hard_wall_ring=self.hard_wall_ring,
                cruise_speed=self.cruise_speed,
                escape_speed=self.escape_speed,
                recover_default_s=self.recover_s,
                hard_reentry_s=self.hard_reentry_s,
                wander_margin=self.wander_margin,
                wander_arrival=self.wander_arrival,
                drone_pose_valid=drone_pose_valid,
                drone_x=drone_x,
                drone_y=drone_y,
            )
            state = b.step(ctx)
            status_msg.data.append(state)
            if state == STATE_RECOVER:
                any_recovery = True

            p = Pose()
            p.position = Point(x=b.x, y=b.y, z=b.z)
            p.orientation.w = 1.0
            arr.poses.append(p)

        self._prev_chased_mask = list(self._chased_mask)
        self.pub.publish(arr)
        self.status_pub.publish(status_msg)
        self.recovery_pub.publish(Bool(data=any_recovery))


def main(args=None):
    rclpy.init(args=args)
    node = BirdsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
