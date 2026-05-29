# Simulated birds on /birds/raw (PoseArray) plus /birds/status.

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseArray, Pose, Point, PoseStamped
from std_msgs.msg import Bool, Int32MultiArray

from drone_system.bird_behavior import BirdAgent, BirdStepContext
from drone_system.bird_behavior import STATE_RECOVER
from drone_system.bird_behavior import PROFILE_EASY, PROFILE_HARD
from drone_system.field_layout import load_layout, bird_count as layout_bird_count


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
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')

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
        self.wander_margin = float(layout.get('bird_wander_margin', 1.5))
        self.wander_arrival = float(layout.get('bird_wander_arrival_m', 1.2))
        self.spawn_margin = float(layout.get('bird_spawn_margin', 0.5))
        self.demo_intro_s = float(layout.get('bird_demo_intro_s', 0.0))
        self.fixed_z = float(layout.get('bird_altitude_m', 4.0))

        self.chased = False
        self._was_chased = False
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_pose_valid = False

        self.birds = []
        layout_birds = layout.get('birds', [])
        default_profiles = [PROFILE_EASY, PROFILE_HARD]
        for i in range(n):
            if i < len(layout_birds):
                b = layout_birds[i]
                sx = float(b['x'])
                sy = float(b['y'])
                profile = str(b.get('profile', default_profiles[i % len(default_profiles)]))
            else:
                b = layout_birds[-1]
                sx, sy = float(b['x']), float(b['y'])
                profile = str(b.get('profile', PROFILE_EASY))
            ox, oy = sx, sy
            sx, sy = self._clamp_spawn(sx, sy)
            if abs(ox) > self.field_xy or abs(oy) > self.field_xy:
                self.get_logger().warn(
                    f'Bird {i} layout spawn ({ox:.1f}, {oy:.1f}) outside field; using ({sx:.1f}, {sy:.1f})'
                )
            self.birds.append(BirdAgent(sx, sy, self.fixed_z, profile=profile, intro_recover_s=self.demo_intro_s))

        intro_msg = f', intro_recover={self.demo_intro_s:.0f}s' if self.demo_intro_s > 0 else ''
        self.get_logger().info(
            f'{len(self.birds)} birds, field_xy=±{self.field_xy}, world_xy=±{self.world_xy}{intro_msg}'
        )

        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.pub = self.create_publisher(PoseArray, '/birds/raw', 10)
        self.status_pub = self.create_publisher(Int32MultiArray, '/birds/status', 10)
        self.recovery_pub = self.create_publisher(Bool, '/birds/in_recovery', 10)
        self.create_subscription(Bool, '/bird/chased', self._chased_cb, 10)
        self.create_subscription(
            PoseStamped,
            pose_topic,
            self._drone_pose_cb,
            qos_profile_sensor_data,
        )

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
        self.chased = msg.data

    def _drone_pose_cb(self, msg):
        self.drone_x = msg.pose.position.x
        self.drone_y = msg.pose.position.y
        self.drone_pose_valid = True

    def update(self):
        arr = PoseArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = self.frame_id

        any_recovery = False
        status_msg = Int32MultiArray()
        just_unchased = self._was_chased and not self.chased
        for b in self.birds:
            ctx = BirdStepContext(
                dt=self.dt,
                chased=self.chased,
                just_unchased=just_unchased,
                field_xy=self.field_xy,
                world_xy=self.world_xy,
                cruise_speed=self.cruise_speed,
                escape_speed=self.escape_speed,
                recover_default_s=self.recover_s,
                wander_margin=self.wander_margin,
                wander_arrival=self.wander_arrival,
                drone_pose_valid=self.drone_pose_valid,
                drone_x=self.drone_x,
                drone_y=self.drone_y,
            )
            state = b.step(ctx)
            status_msg.data.append(state)
            if state == STATE_RECOVER:
                any_recovery = True

            p = Pose()
            p.position = Point(x=b.x, y=b.y, z=b.z)
            p.orientation.w = 1.0
            arr.poses.append(p)

        self._was_chased = self.chased
        self.pub.publish(arr)
        self.status_pub.publish(status_msg)
        self.recovery_pub.publish(Bool(data=any_recovery))


def main(args=None):
    rclpy.init(args=args)
    node = BirdsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
