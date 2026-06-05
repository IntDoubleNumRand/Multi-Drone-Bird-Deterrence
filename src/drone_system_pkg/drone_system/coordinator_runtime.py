# Per-drone patrol / chase / return controller; reads /central/assignment/<id>.

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from mavros_msgs.msg import State as MavrosState
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int32, Int32MultiArray
from visualization_msgs.msg import Marker

from drone_system.coordination.assignment import (
    STATE_FLEE,
    STATE_WANDER,
    assign_nearest_in_field,
    is_assignable_bird,
    is_proximate_bird,
)
from drone_system.coordination.behavior_tree import Action, Condition, Selector, Sequence
from drone_system.coordination.models import BirdSnapshot, DroneSnapshot
from drone_system.field_layout import bird_detection_box_half, load_layout
from drone_system.launch_params import DRONE_POSE_TOPICS

MODE_PATROL = "patrol"
MODE_CHASE = "chase"
MODE_RETURN = "return"
MODE_HOLD = "hold"
MODE_REST = "rest"


@dataclass
class DecisionContext:
    low_battery: bool
    has_target: bool
    hold_active: bool
    rest_active: bool
    force_home: bool
    return_home_when_no_targets: bool
    mode: str = MODE_PATROL


class PatrolState:
    def execute(self, node):
        px, py = node.patrol[node.idx]
        if math.hypot(node.x - px, node.y - py) < node.patrol_advance_m:
            node.idx = (node.idx + 1) % len(node.patrol)
        node.send_goal(px, py, node.patrol_z)


class ChaseState:
    def execute(self, node):
        if node.target_index < 0 or node.target_index >= len(node._birds):
            node.send_goal(node.home_x, node.home_y, node.patrol_z)
            return
        p = node._birds[node.target_index].position
        bx, by = float(p.x), float(p.y)
        # If the bird has already crossed the field boundary, aim for the wall
        # instead of telling the drone to leave the allowed area.
        if abs(bx) > node.field_xy or abs(by) > node.field_xy:
            tx, ty = node._clamp_to_field(bx, by)
        else:
            tx, ty = node.chase_goal_xy(bx, by)
        node.send_goal(tx, ty, node.chase_z)


class ReturnState:
    def execute(self, node):
        node.send_goal(node.home_x, node.home_y, node.return_z)


class RestState:
    def execute(self, node):
        node.send_goal(node.home_x, node.home_y, node.rest_z, enforce_min_z=False)


class HoldState:
    def execute(self, node):
        # Stay near the handoff point for a moment before heading home.
        node.send_goal(node._hold_x, node._hold_y, node.return_z)


class CoordinatorNode(Node):
    def __init__(self):
        super().__init__("coordinator_node")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        layout = load_layout()
        home = layout.get("home", {})
        self.declare_parameter("pose_topic", DRONE_POSE_TOPICS[0])
        self.declare_parameter("setpoint_topic", "/drone_1/setpoint_position/local")
        self.declare_parameter("birds_topic", "/birds/positions")
        self.declare_parameter("obstacles_topic", "/obstacles/positions")
        self.declare_parameter("bird_status_topic", "/birds/status")
        self.declare_parameter("drone_id", "drone_1")
        self.declare_parameter("assignment_topic", "")
        self.declare_parameter("chased_mask_topic", "/birds/chased_mask")
        self.declare_parameter("target_topic", "/coordinator/target_index")
        self.declare_parameter("z_state_topic", "/drone/z_state")
        self.declare_parameter("mavros_state_topic", "/drone_1/state")
        self.declare_parameter("battery_topic", "/drone_1/battery")
        self.declare_parameter("mavros_arm_service", "/drone_1/cmd/arming")
        self.declare_parameter("mavros_set_mode_service", "/drone_1/set_mode")
        self.declare_parameter("assignment_timeout_s", 1.5)
        self.declare_parameter("use_local_assignment_fallback", False)

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("limit_xy", float(layout.get("limit_xy", 15.0)))
        self.declare_parameter("field_xy", float(layout.get("field_xy", 10.0)))
        self.declare_parameter("patrol_z", float(layout.get("patrol_z", 7.0)))
        self.declare_parameter("chase_z", float(layout.get("chase_z", 8.0)))
        self.declare_parameter("return_z", float(layout.get("return_z", 7.0)))
        self.declare_parameter("rest_z", float(layout.get("rest_z", 0.3)))
        self.declare_parameter("min_flight_z", float(layout.get("min_flight_z", 7.0)))
        self.declare_parameter("target_lock_s", float(layout.get("target_lock_s", 2.5)))
        self.declare_parameter("home_x", float(home.get("x", 0.0)))
        self.declare_parameter("home_y", float(home.get("y", 0.0)))
        self.declare_parameter("setpoint_rate_hz", 20.0)
        self.declare_parameter("logic_rate_hz", 10.0)
        self.declare_parameter("auto_offboard", True)
        self.declare_parameter("arm_retry_s", 2.0)
        self.declare_parameter("mode_retry_s", 2.0)
        self.declare_parameter("setpoint_warmup_cycles", 40)
        self.declare_parameter(
            "chase_standoff_m",
            float(layout.get("chase_standoff_m", 1.5)),
        )
        self.declare_parameter("setpoint_max_step_m", 0.0)
        self.declare_parameter("setpoint_max_z_step_m", 0.0)
        self.declare_parameter("patrol_advance_m", 0.5)
        self.declare_parameter(
            "return_home_when_no_targets",
            bool(layout.get("return_home_when_no_targets", True)),
        )
        self.declare_parameter("boundary_hold_s", float(layout.get("boundary_hold_s", 5.0)))
        self.declare_parameter("demo_post_chase_home_s", 0.0)
        self.declare_parameter("post_assignment_rest_s", float(layout.get("post_assignment_rest_s", 4.0)))
        self.declare_parameter("enable_low_battery_return", False)
        self.declare_parameter("battery_drain_per_tick", 0.0)
        self.declare_parameter("low_battery_threshold", 25.0)
        self.declare_parameter("battery_topic_timeout_s", 2.0)
        self.declare_parameter("bird_timeout_s", 1.0)
        self.declare_parameter("obstacle_clearance_m", float(layout.get("obstacle_clearance_m", 2.0)))
        self.declare_parameter("obstacle_repulsion_gain", float(layout.get("obstacle_repulsion_gain", 1.0)))
        self.declare_parameter(
            "obstacle_flyover_clearance_m",
            float(layout.get("obstacle_flyover_clearance_m", layout.get("house_flyover_clearance_m", 1.0))),
        )
        self.declare_parameter(
            "obstacle_max_flyover_z",
            float(layout.get("obstacle_max_flyover_z", 7.0)),
        )

        self.limit = float(self.get_parameter("limit_xy").value)
        self.field_xy = float(self.get_parameter("field_xy").value)
        self.det_half_x, self.det_half_y, self.det_half_z = bird_detection_box_half()
        self.frame_id = self.get_parameter("map_frame").value
        self.patrol_z = float(self.get_parameter("patrol_z").value)
        self.chase_z = float(self.get_parameter("chase_z").value)
        self.return_z = float(self.get_parameter("return_z").value)
        self.rest_z = float(self.get_parameter("rest_z").value)
        self.min_flight_z = float(self.get_parameter("min_flight_z").value)
        self.target_lock_s = float(self.get_parameter("target_lock_s").value)
        self.home_x = float(self.get_parameter("home_x").value)
        self.home_y = float(self.get_parameter("home_y").value)
        self.auto_offboard = bool(self.get_parameter("auto_offboard").value)
        self.arm_retry_s = float(self.get_parameter("arm_retry_s").value)
        self.mode_retry_s = float(self.get_parameter("mode_retry_s").value)
        self.setpoint_warmup_cycles = int(self.get_parameter("setpoint_warmup_cycles").value)
        self.chase_standoff_m = float(self.get_parameter("chase_standoff_m").value)
        self.setpoint_max_step_m = float(self.get_parameter("setpoint_max_step_m").value)
        self.setpoint_max_z_step_m = float(self.get_parameter("setpoint_max_z_step_m").value)
        self.patrol_advance_m = float(self.get_parameter("patrol_advance_m").value)
        self.return_home_when_no_targets = bool(self.get_parameter("return_home_when_no_targets").value)
        self.boundary_hold_s = float(self.get_parameter("boundary_hold_s").value)
        self.demo_post_chase_home_s = float(self.get_parameter("demo_post_chase_home_s").value)
        self.post_assignment_rest_s = float(self.get_parameter("post_assignment_rest_s").value)
        self.enable_low_battery_return = bool(self.get_parameter("enable_low_battery_return").value)
        self.battery_drain_per_tick = float(self.get_parameter("battery_drain_per_tick").value)
        self.low_battery_threshold = float(self.get_parameter("low_battery_threshold").value)
        self.battery_topic_timeout_s = float(self.get_parameter("battery_topic_timeout_s").value)
        self.bird_timeout_s = float(self.get_parameter("bird_timeout_s").value)
        self.obstacle_clearance = float(self.get_parameter("obstacle_clearance_m").value)
        self.obstacle_repulsion_gain = float(self.get_parameter("obstacle_repulsion_gain").value)
        self.obstacle_flyover_clearance_m = float(self.get_parameter("obstacle_flyover_clearance_m").value)
        self.obstacle_max_flyover_z = float(self.get_parameter("obstacle_max_flyover_z").value)
        self.use_local_assignment_fallback = bool(self.get_parameter("use_local_assignment_fallback").value)
        self.assignment_timeout_s = float(self.get_parameter("assignment_timeout_s").value)

        self.drone_id = self.get_parameter("drone_id").value
        assignment_topic = self.get_parameter("assignment_topic").value
        if not assignment_topic:
            assignment_topic = f"/central/assignment/{self.drone_id}"
        self.assignment_topic = assignment_topic
        battery_topic = self.get_parameter("battery_topic").value
        if not battery_topic:
            battery_topic = f"/{self.drone_id}/battery"
        self.battery_topic = battery_topic

        wps = layout.get("patrol_waypoints", [])
        self.patrol = [(float(w[0]), float(w[1])) for w in wps] if wps else [
            (-8.0, -8.0), (-8.0, 8.0), (8.0, 8.0), (8.0, -8.0),
        ]
        self.idx = 0
        self.battery = 100.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.pose_valid = False
        self._birds = []
        self._bird_states = []
        self._obstacles = []
        self._last_bird_msg_ns = 0
        self._prev_has_target = False
        self._home_until_ns = 0
        self._rest_until_ns = 0
        self._hold_until_ns = 0
        self._hold_x = 0.0
        self._hold_y = 0.0
        self._must_return_home = False
        self._blocked_target = -1
        self._completed_targets = set()
        self._active_target = -1
        self.target_index = -1
        self._assigned_target = -1
        self._last_assignment_ns = 0
        self._locked_target = -1
        self._lock_until_ns = 0
        self._last_mode_name = ""

        self._setpoint = PoseStamped()
        self._setpoint.pose.orientation.w = 1.0
        self._setpoint_count = 0
        self._mavros_state = MavrosState()
        self._arm_req_inflight = False
        self._mode_req_inflight = False
        self._last_arm_req_ns = 0
        self._last_mode_req_ns = 0
        self._last_battery_msg_ns = 0

        pose_topic = self.get_parameter("pose_topic").value
        setpoint_topic = self.get_parameter("setpoint_topic").value
        birds_topic = self.get_parameter("birds_topic").value
        obstacles_topic = self.get_parameter("obstacles_topic").value
        status_topic = self.get_parameter("bird_status_topic").value
        chased_mask_topic = self.get_parameter("chased_mask_topic").value
        target_topic = self.get_parameter("target_topic").value
        z_state_topic = self.get_parameter("z_state_topic").value
        mavros_state_topic = self.get_parameter("mavros_state_topic").value
        mavros_state_topic_fallback = self._alternate_state_topic(mavros_state_topic)
        battery_topic = self.battery_topic
        mavros_arm_service = self.get_parameter("mavros_arm_service").value
        mavros_set_mode_service = self.get_parameter("mavros_set_mode_service").value

        self.setpoint_pub = self.create_publisher(PoseStamped, setpoint_topic, 10)
        self.chased_pub = self.create_publisher(Int32MultiArray, chased_mask_topic, 10)
        self.target_pub = self.create_publisher(Int32, target_topic, 10)
        self.z_pub = self.create_publisher(Marker, z_state_topic, 10)

        self.create_subscription(PoseArray, birds_topic, self.birds_cb, 10)
        self.create_subscription(PoseArray, obstacles_topic, self.obstacles_cb, 10)
        self.create_subscription(Int32MultiArray, status_topic, self.status_cb, 10)
        self.create_subscription(Int32, self.assignment_topic, self.assignment_cb, 10)
        self.create_subscription(PoseStamped, pose_topic, self.pose_cb, qos_profile_sensor_data)
        self.create_subscription(BatteryState, battery_topic, self.battery_cb, qos_profile_sensor_data)
        self.create_subscription(
            MavrosState, mavros_state_topic, self.mavros_state_cb, qos_profile_sensor_data
        )
        if mavros_state_topic_fallback and mavros_state_topic_fallback != mavros_state_topic:
            self.create_subscription(
                MavrosState, mavros_state_topic_fallback, self.mavros_state_cb, qos_profile_sensor_data
            )

        self.arm_client = self.create_client(CommandBool, mavros_arm_service)
        self.mode_client = self.create_client(SetMode, mavros_set_mode_service)
        self._patrol_state = PatrolState()
        self._chase_state = ChaseState()
        self._return_state = ReturnState()
        self._rest_state = RestState()
        self._hold_state = HoldState()

        self._decision_tree = Selector([
            Sequence([Condition(lambda c: c.low_battery), Action(lambda c: setattr(c, "mode", MODE_RETURN))]),
            Sequence([Condition(lambda c: c.force_home), Action(lambda c: setattr(c, "mode", MODE_RETURN))]),
            # After a chase ends near the wall, pause there briefly before returning.
            Sequence([Condition(lambda c: c.hold_active), Action(lambda c: setattr(c, "mode", MODE_HOLD))]),
            Sequence([Condition(lambda c: c.rest_active), Action(lambda c: setattr(c, "mode", MODE_REST))]),
            Sequence([Condition(lambda c: c.has_target), Action(lambda c: setattr(c, "mode", MODE_CHASE))]),
            Sequence([
                Condition(lambda c: c.return_home_when_no_targets),
                Action(lambda c: setattr(c, "mode", MODE_RETURN)),
            ]),
            Action(lambda c: setattr(c, "mode", MODE_PATROL)),
        ])

        logic_hz = float(self.get_parameter("logic_rate_hz").value) or 10.0
        self.create_timer(1.0 / logic_hz, self.update)
        sp_hz = float(self.get_parameter("setpoint_rate_hz").value) or 20.0
        self.create_timer(1.0 / sp_hz, self.publish_setpoint)

    def _now_ns(self):
        return self.get_clock().now().nanoseconds

    def pose_cb(self, msg):
        self.x = float(msg.pose.position.x)
        self.y = float(msg.pose.position.y)
        self.z = float(msg.pose.position.z)
        self.pose_valid = True

    def birds_cb(self, msg):
        self._birds = list(msg.poses)
        self._last_bird_msg_ns = self._now_ns()

    def status_cb(self, msg):
        self._bird_states = list(msg.data)

    def obstacles_cb(self, msg):
        # Obstacles are encoded into Pose messages by obstacles_node.
        # position stores the center and top height.
        # orientation.z is the cylinder radius when non-zero.
        # orientation.x and orientation.y store box half extents.
        obs = []
        for p in msg.poses:
            obs.append((
                float(p.position.x),       # cx
                float(p.position.y),       # cy
                float(p.position.z),       # top_z
                float(p.orientation.z),    # radius (0 => box)
                float(p.orientation.x),    # half_x
                float(p.orientation.y),    # half_y
            ))
        self._obstacles = obs

    def assignment_cb(self, msg):
        self._assigned_target = int(msg.data)
        self._last_assignment_ns = self._now_ns()

    def mavros_state_cb(self, msg):
        self._mavros_state = msg

    def battery_cb(self, msg):
        percent = float(msg.percentage)
        if math.isfinite(percent) and percent >= 0.0:
            # MAVROS usually reports percentage on a 0..1 scale, but keeping
            # the >1 path makes this tolerant of already-percent values.
            self.battery = percent * 100.0 if percent <= 1.0 else percent
            self._last_battery_msg_ns = self._now_ns()

    @staticmethod
    def _alternate_state_topic(primary_topic):
        """Common MAVROS state-topic fallback across naming styles."""
        topic = str(primary_topic or "").strip()
        if not topic:
            return ""
        if topic.endswith("/mavros/state"):
            return topic.replace("/mavros/state", "/state")
        if topic.endswith("/state"):
            return topic[:-len("/state")] + "/mavros/state"
        return ""

    def chase_goal_xy(self, bx, by):
        # Walk from the drone toward the bird, then stop short by the configured
        # standoff distance so the drone does not overshoot the target.
        dx = bx - self.x
        dy = by - self.y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return bx, by
        reach = max(0.0, dist - self.chase_standoff_m)
        t = reach / dist
        return self.x + t * dx, self.y + t * dy

    def _bird_state(self, i):
        return int(self._bird_states[i]) if i < len(self._bird_states) else STATE_WANDER

    def _bird_snapshot(self, i):
        p = self._birds[i].position
        return BirdSnapshot(
            index=i,
            x=float(p.x),
            y=float(p.y),
            z=float(p.z),
            state=self._bird_state(i),
        )

    def _is_assignable(self, i):
        if i < 0 or i >= len(self._birds):
            return False
        if not self._bird_outside_field(i):
            self._completed_targets.discard(i)
        if i in self._completed_targets:
            return False
        # Once a bird has been chased out, ignore it until it comes back inside.
        if i == self._blocked_target:
            b = self._bird_snapshot(i)
            if abs(b.x) <= self.field_xy and abs(b.y) <= self.field_xy:
                self._blocked_target = -1
            else:
                return False
        return is_assignable_bird(self._bird_snapshot(i), self.field_xy, self.limit)

    def _is_proximate_to_bird(self, bird_index):
        if bird_index < 0 or bird_index >= len(self._birds):
            return False
        bird = self._bird_snapshot(bird_index)
        drone = DroneSnapshot(
            drone_id=self.drone_id, x=self.x, y=self.y, z=self.z, valid=self.pose_valid
        )
        return is_proximate_bird(
            drone,
            bird,
            self.det_half_x,
            self.det_half_y,
            self.det_half_z,
            self.field_xy,
            self.limit,
            check_z=True,
            z_above_only=True,
            z_above_max=self.det_half_z * 2.0,
        )

    def _clamp_to_field(self, x, y):
        return (
            max(-self.field_xy, min(self.field_xy, float(x))),
            max(-self.field_xy, min(self.field_xy, float(y))),
        )

    def _target_outside_field(self):
        if self.target_index < 0 or self.target_index >= len(self._birds):
            return False
        p = self._birds[self.target_index].position
        return abs(float(p.x)) > self.field_xy or abs(float(p.y)) > self.field_xy

    def _bird_outside_field(self, bird_index):
        if bird_index < 0 or bird_index >= len(self._birds):
            return False
        p = self._birds[bird_index].position
        return abs(float(p.x)) > self.field_xy or abs(float(p.y)) > self.field_xy

    def _refresh_completed_targets(self):
        for i in range(len(self._birds)):
            if self._bird_outside_field(i):
                self._completed_targets.add(i)
            else:
                self._completed_targets.discard(i)

    def _start_return_home_cycle(self, now_ns, target_index=-1):
        if target_index >= 0:
            self._completed_targets.add(int(target_index))
            self._blocked_target = int(target_index)
        self._must_return_home = True
        self._locked_target = -1
        self._active_target = -1
        self.target_index = -1
        self._hold_until_ns = 0

    def _start_boundary_hold(self, now_ns):
        if self.boundary_hold_s > 0.0:
            self._hold_until_ns = now_ns + int(self.boundary_hold_s * 1e9)
            self._hold_x, self._hold_y = self._clamp_to_field(self.x, self.y)

    def _local_fallback_target(self):
        birds = [
            self._bird_snapshot(i)
            for i in range(len(self._birds))
            if i not in self._completed_targets
        ]
        d = DroneSnapshot(
            drone_id=self.drone_id, x=self.x, y=self.y, z=self.z, valid=self.pose_valid
        )
        return int(
            assign_nearest_in_field([d], birds, self.field_xy, self.limit).get(
                self.drone_id, -1
            )
        )

    def _central_assignment_is_fresh(self, now_ns):
        return (
            self._last_assignment_ns > 0
            and (now_ns - self._last_assignment_ns) <= int(self.assignment_timeout_s * 1e9)
        )

    def _candidate_target_index(self):
        now_ns = self._now_ns()
        if self._central_assignment_is_fresh(now_ns):
            if self._assigned_target >= 0:
                return self._assigned_target
            if self.use_local_assignment_fallback:
                return self._local_fallback_target()
            return -1
        if self.use_local_assignment_fallback:
            return self._local_fallback_target()
        return -1

    def _resolve_target_index(self):
        now_ns = self._now_ns()
        central_fresh = self._central_assignment_is_fresh(now_ns)
        idx = self._candidate_target_index()
        if 0 <= idx < len(self._birds) and self._is_assignable(idx):
            self._locked_target = idx
            self._lock_until_ns = now_ns + int(self.target_lock_s * 1e9)
            return idx
        if central_fresh:
            # Central assignment is authoritative in multi-drone mode. If this
            # drone was unassigned or assigned an invalid target, do not keep
            # chasing a stale locked target that another drone may now own.
            self._locked_target = -1
            return -1
        if self._locked_target >= 0 and now_ns < self._lock_until_ns:
            if self._is_assignable(self._locked_target):
                return self._locked_target
        self._locked_target = -1
        return -1

    def _rate_limit_xy(self, x, y):
        # Limit the XY jump per update so OFFBOARD setpoints move like a smooth
        # path instead of teleporting across the field.
        if self.setpoint_max_step_m <= 0.0:
            return x, y
        sx, sy = float(self._setpoint.pose.position.x), float(self._setpoint.pose.position.y)
        if self.pose_valid and self._setpoint_count < 2:
            sx, sy = self.x, self.y
        dx, dy = x - sx, y - sy
        dist = math.hypot(dx, dy)
        if dist > self.setpoint_max_step_m:
            # Normalize the delta vector, then scale it to the allowed step size.
            x = sx + self.setpoint_max_step_m * dx / dist
            y = sy + self.setpoint_max_step_m * dy / dist
        return x, y

    def _rate_limit_z(self, z):
        if self.setpoint_max_z_step_m <= 0.0:
            return z
        sz = float(self._setpoint.pose.position.z)
        dz = z - sz
        if abs(dz) > self.setpoint_max_z_step_m:
            z = sz + math.copysign(self.setpoint_max_z_step_m, dz)
        return z

    def _keepout_gap(self, px, py, ob):
        # Signed distance to the inflated obstacle boundary.
        # Negative means the point has already entered the keep-out zone.
        cx, cy, _top, radius, hx, hy = ob
        cl = self.obstacle_clearance
        if radius > 0.0:  # cylinder
            return math.hypot(px - cx, py - cy) - (radius + cl)
        # For boxes, compare against an inflated axis-aligned rectangle.
        return max(abs(px - cx) - (hx + cl), abs(py - cy) - (hy + cl))

    def _near_obstacle(self, ob, gx, gy):
        # Sample the straight-line path. If any sample lands inside keep-out,
        # treat the route as blocked and let avoidance adjust it.
        if self._keepout_gap(gx, gy, ob) < 0.0 or self._keepout_gap(self.x, self.y, ob) < 0.0:
            return True
        steps = 8
        for k in range(1, steps):
            t = k / steps
            sx = self.x + t * (gx - self.x)
            sy = self.y + t * (gy - self.y)
            if self._keepout_gap(sx, sy, ob) < 0.0:
                return True
        return False

    def _push_out(self, x, y, ob):
        # Push the goal sideways until it sits just outside the keep-out region.
        cx, cy, _top, radius, hx, hy = ob
        cl = self.obstacle_clearance
        if radius > 0.0:  # cylinder
            dx, dy = x - cx, y - cy
            dist = math.hypot(dx, dy)
            shell = radius + cl
            if dist < 1e-6:
                dx, dy, dist = 1.0, 0.0, 1.0
            if dist < shell:
                # Move along the outward radial direction by the missing margin.
                push = (shell - dist) * self.obstacle_repulsion_gain
                x += push * dx / dist
                y += push * dy / dist
            return x, y
        # For boxes, move to whichever inflated face is closer.
        pen_x = (hx + cl) - abs(x - cx)
        pen_y = (hy + cl) - abs(y - cy)
        if pen_x > 0.0 and pen_y > 0.0:
            if pen_x <= pen_y:
                x = cx + math.copysign(hx + cl, x - cx if x != cx else 1.0)
            else:
                y = cy + math.copysign(hy + cl, y - cy if y != cy else 1.0)
        return x, y

    def _apply_obstacle_avoidance(self, gx, gy, gz):
        # Short obstacles are handled by climbing over them.
        # Tall obstacles are handled by nudging the XY goal around them.
        if not self._obstacles:
            return gx, gy, gz
        x, y, z = float(gx), float(gy), float(gz)
        for ob in self._obstacles:
            top_z = ob[2]
            if not self._near_obstacle(ob, x, y):
                continue
            if top_z <= self.obstacle_max_flyover_z:
                z = max(z, top_z + self.obstacle_flyover_clearance_m)
            else:
                x, y = self._push_out(x, y, ob)
        return x, y, z

    def send_goal(self, x, y, z, *, enforce_min_z=True):
        # Keep the commanded point inside the field even if the bird has already
        # escaped into the larger world box used by the bird simulation.
        x = max(-self.field_xy, min(self.field_xy, x))
        y = max(-self.field_xy, min(self.field_xy, y))
        x, y, z = self._apply_obstacle_avoidance(x, y, z)
        x = max(-self.field_xy, min(self.field_xy, x))
        y = max(-self.field_xy, min(self.field_xy, y))
        x, y = self._rate_limit_xy(x, y)
        z = max(float(z), self.min_flight_z) if enforce_min_z else max(float(z), 0.0)
        z = self._rate_limit_z(z)
        self._setpoint.header.frame_id = self.frame_id
        self._setpoint.header.stamp = self.get_clock().now().to_msg()
        self._setpoint.pose.position.x = float(x)
        self._setpoint.pose.position.y = float(y)
        self._setpoint.pose.position.z = float(z)
        self._setpoint.pose.orientation.w = 1.0
        m = Marker()
        m.pose.position.z = z
        self.z_pub.publish(m)

    def publish_setpoint(self):
        self._setpoint.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_pub.publish(self._setpoint)
        self._setpoint_count += 1

    def _maybe_request_arm(self):
        if self._arm_req_inflight or self._mavros_state.armed or not self.arm_client.service_is_ready():
            return
        now = self._now_ns()
        if now - self._last_arm_req_ns < int(self.arm_retry_s * 1e9):
            return
        req = CommandBool.Request()
        req.value = True
        self._arm_req_inflight = True
        self._last_arm_req_ns = now
        self.arm_client.call_async(req).add_done_callback(self._arm_done)

    def _arm_done(self, fut):
        self._arm_req_inflight = False
        try:
            _ = fut.result()
        except Exception as exc:
            self.get_logger().warn(f"Arm request failed: {exc}")

    def _maybe_request_offboard_mode(self):
        if self._mode_req_inflight or self._mavros_state.mode == "OFFBOARD" or not self.mode_client.service_is_ready():
            return
        now = self._now_ns()
        if now - self._last_mode_req_ns < int(self.mode_retry_s * 1e9):
            return
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = "OFFBOARD"
        self._mode_req_inflight = True
        self._last_mode_req_ns = now
        self.mode_client.call_async(req).add_done_callback(self._mode_done)

    def _mode_done(self, fut):
        self._mode_req_inflight = False
        try:
            _ = fut.result()
        except Exception as exc:
            self.get_logger().warn(f"SetMode request failed: {exc}")

    def _handle_offboard_bootstrap(self):
        if not self.auto_offboard:
            return True
        if not self._mavros_state.connected:
            self.send_goal(self.home_x, self.home_y, self.patrol_z)
            return False
        if self._setpoint_count < self.setpoint_warmup_cycles:
            self.send_goal(self.home_x, self.home_y, self.patrol_z)
            return False
        self._maybe_request_arm()
        if not self._mavros_state.armed:
            self.send_goal(self.home_x, self.home_y, self.patrol_z)
            return False
        self._maybe_request_offboard_mode()
        if self._mavros_state.mode != "OFFBOARD":
            self.send_goal(self.home_x, self.home_y, self.patrol_z)
            return False
        return True

    def _run_decision_tree(self, has_target, hold_active, rest_active):
        ctx = DecisionContext(
            low_battery=self.enable_low_battery_return and self.battery < self.low_battery_threshold,
            has_target=has_target,
            hold_active=hold_active,
            rest_active=rest_active,
            force_home=(self._now_ns() < self._home_until_ns) or self._must_return_home,
            return_home_when_no_targets=self.return_home_when_no_targets,
        )
        self._decision_tree.run(ctx)
        return ctx.mode

    def _select_state(self, mode):
        if mode == MODE_CHASE:
            return self._chase_state
        if mode == MODE_HOLD:
            return self._hold_state
        if mode == MODE_REST:
            return self._rest_state
        if mode == MODE_RETURN:
            return self._return_state
        return self._patrol_state

    def update(self):
        battery_topic_fresh = False
        if self._last_battery_msg_ns > 0 and self.battery_topic_timeout_s > 0.0:
            battery_topic_fresh = ((self._now_ns() - self._last_battery_msg_ns) / 1e9) <= self.battery_topic_timeout_s
        if not battery_topic_fresh and self.battery_drain_per_tick > 0.0:
            self.battery = max(0.0, self.battery - self.battery_drain_per_tick)
        if self._last_bird_msg_ns > 0 and self.bird_timeout_s > 0.0:
            if (self._now_ns() - self._last_bird_msg_ns) / 1e9 > self.bird_timeout_s:
                self._birds = []
        if not self._handle_offboard_bootstrap():
            self.chased_pub.publish(Int32MultiArray(data=[]))
            return
        if not self.pose_valid:
            self.send_goal(self.home_x, self.home_y, self.patrol_z)
            self.chased_pub.publish(Int32MultiArray(data=[]))
            return

        now_ns = self._now_ns()
        self._refresh_completed_targets()
        if self._active_target in self._completed_targets:
            self._start_return_home_cycle(now_ns, self._active_target)

        if self._must_return_home and math.hypot(self.x - self.home_x, self.y - self.home_y) <= 0.8:
            # Once home, wait a bit before taking another assignment.
            self._must_return_home = False
            if self.post_assignment_rest_s > 0.0:
                self._rest_until_ns = now_ns + int(self.post_assignment_rest_s * 1e9)

        if self._must_return_home or now_ns < self._rest_until_ns:
            self.target_index = -1
        else:
            self.target_index = self._resolve_target_index()
        has_target = self.target_index >= 0
        if has_target and self._target_outside_field():
            # Once the drone reaches the wall for this chase, stop there, then
            # hand the drone back to the return-home flow.
            tx, ty = self._clamp_to_field(
                self._birds[self.target_index].position.x,
                self._birds[self.target_index].position.y,
            )
            if math.hypot(self.x - tx, self.y - ty) <= 0.8:
                self._start_return_home_cycle(now_ns, self.target_index)
                has_target = False
        if self._must_return_home:
            # Do not resume chasing until the return-home cycle is finished.
            self.target_index = -1
            has_target = False
        rest_active = (not self._must_return_home) and now_ns < self._rest_until_ns
        if rest_active:
            self.target_index = -1
            has_target = False
        if has_target:
            self._active_target = int(self.target_index)
        if self._prev_has_target and not has_target:
            # Cache the current spot so the hold state stays put instead of drifting.
            self._start_boundary_hold(now_ns)
            if self.demo_post_chase_home_s > 0.0:
                self._home_until_ns = now_ns + int(self.demo_post_chase_home_s * 1e9)
        self._prev_has_target = has_target

        hold_active = (not has_target) and (not rest_active) and now_ns < self._hold_until_ns
        mode = self._run_decision_tree(has_target, hold_active, rest_active)
        # Publish one bit per bird showing whether this drone is currently close
        # enough to be considered an active deterrent.
        chased_mask = []
        scare_enabled = mode in (MODE_CHASE, MODE_HOLD)
        for i in range(len(self._birds)):
            bit = 1 if (scare_enabled and self._is_proximate_to_bird(i)) else 0
            chased_mask.append(bit)
        self.chased_pub.publish(Int32MultiArray(data=chased_mask))
        self.target_pub.publish(Int32(data=self.target_index if has_target else -1))

        if mode != self._last_mode_name:
            self._last_mode_name = mode
            self.get_logger().info(
                f"{self.drone_id} FSM -> {mode}, target={self.target_index if has_target else -1}"
            )
        self._select_state(mode).execute(self)


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

