# Per-drone patrol / chase / return controller; reads /central/assignment/<id>.

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from mavros_msgs.msg import State as MavrosState
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Int32, Int32MultiArray
from visualization_msgs.msg import Marker

from drone_system.coordination.assignment import (
    STATE_FLEE,
    STATE_WANDER,
    assign_nearest_unique,
)
from drone_system.coordination.behavior_tree import Action, Condition, Selector, Sequence
from drone_system.coordination.models import BirdSnapshot, DroneSnapshot
from drone_system.field_layout import load_layout

MODE_PATROL = "patrol"
MODE_CHASE = "chase"
MODE_RETURN = "return"


@dataclass
class DecisionContext:
    low_battery: bool
    has_target: bool
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
        dx = p.x - node.x
        dy = p.y - node.y
        dist = math.hypot(dx, dy) or 1.0
        # Lead point: bird position + chase_lead_m along (bird - drone) unit vector.
        tx = p.x + node.chase_lead_m * dx / dist
        ty = p.y + node.chase_lead_m * dy / dist
        node.send_goal(tx, ty, node.chase_z)


class ReturnState:
    def execute(self, node):
        node.send_goal(node.home_x, node.home_y, node.return_z)


class CoordinatorNode(Node):
    def __init__(self):
        super().__init__("coordinator_node")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        layout = load_layout()
        home = layout.get("home", {})
        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("setpoint_topic", "/mavros/setpoint_position/local")
        self.declare_parameter("birds_topic", "/birds/positions")
        self.declare_parameter("obstacles_topic", "/obstacles/positions")
        self.declare_parameter("bird_status_topic", "/birds/status")
        self.declare_parameter("drone_id", "drone_1")
        self.declare_parameter("assignment_topic", "")
        self.declare_parameter("assignment_timeout_s", 1.5)
        self.declare_parameter("use_local_assignment_fallback", True)

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("limit_xy", float(layout.get("limit_xy", 15.0)))
        self.declare_parameter("field_xy", float(layout.get("field_xy", 10.0)))
        self.declare_parameter("patrol_z", float(layout.get("patrol_z", 5.0)))
        self.declare_parameter("chase_z", float(layout.get("chase_z", 8.0)))
        self.declare_parameter("return_z", float(layout.get("return_z", 3.0)))
        self.declare_parameter("home_x", float(home.get("x", 0.0)))
        self.declare_parameter("home_y", float(home.get("y", 0.0)))
        self.declare_parameter("setpoint_rate_hz", 20.0)
        self.declare_parameter("logic_rate_hz", 10.0)
        self.declare_parameter("auto_offboard", True)
        self.declare_parameter("arm_retry_s", 2.0)
        self.declare_parameter("mode_retry_s", 2.0)
        self.declare_parameter("setpoint_warmup_cycles", 40)
        self.declare_parameter("chase_lead_m", 2.0)
        self.declare_parameter("setpoint_max_step_m", 0.0)
        self.declare_parameter("setpoint_max_z_step_m", 0.0)
        self.declare_parameter("patrol_advance_m", 0.5)
        self.declare_parameter("return_home_when_no_targets", True)
        self.declare_parameter("demo_post_chase_home_s", 0.0)
        self.declare_parameter("enable_low_battery_return", False)
        self.declare_parameter("battery_drain_per_tick", 0.0)
        self.declare_parameter("low_battery_threshold", 25.0)
        self.declare_parameter("bird_timeout_s", 1.0)
        self.declare_parameter("obstacle_clearance_m", float(layout.get("obstacle_clearance_m", 2.0)))
        self.declare_parameter("obstacle_repulsion_gain", float(layout.get("obstacle_repulsion_gain", 1.0)))

        self.limit = float(self.get_parameter("limit_xy").value)
        self.field_xy = float(self.get_parameter("field_xy").value)
        self.frame_id = self.get_parameter("map_frame").value
        self.patrol_z = float(self.get_parameter("patrol_z").value)
        self.chase_z = float(self.get_parameter("chase_z").value)
        self.return_z = float(self.get_parameter("return_z").value)
        self.home_x = float(self.get_parameter("home_x").value)
        self.home_y = float(self.get_parameter("home_y").value)
        self.auto_offboard = bool(self.get_parameter("auto_offboard").value)
        self.arm_retry_s = float(self.get_parameter("arm_retry_s").value)
        self.mode_retry_s = float(self.get_parameter("mode_retry_s").value)
        self.setpoint_warmup_cycles = int(self.get_parameter("setpoint_warmup_cycles").value)
        self.chase_lead_m = float(self.get_parameter("chase_lead_m").value)
        self.setpoint_max_step_m = float(self.get_parameter("setpoint_max_step_m").value)
        self.setpoint_max_z_step_m = float(self.get_parameter("setpoint_max_z_step_m").value)
        self.patrol_advance_m = float(self.get_parameter("patrol_advance_m").value)
        self.return_home_when_no_targets = bool(self.get_parameter("return_home_when_no_targets").value)
        self.demo_post_chase_home_s = float(self.get_parameter("demo_post_chase_home_s").value)
        self.enable_low_battery_return = bool(self.get_parameter("enable_low_battery_return").value)
        self.battery_drain_per_tick = float(self.get_parameter("battery_drain_per_tick").value)
        self.low_battery_threshold = float(self.get_parameter("low_battery_threshold").value)
        self.bird_timeout_s = float(self.get_parameter("bird_timeout_s").value)
        self.obstacle_clearance = float(self.get_parameter("obstacle_clearance_m").value)
        self.obstacle_repulsion_gain = float(self.get_parameter("obstacle_repulsion_gain").value)
        self.use_local_assignment_fallback = bool(self.get_parameter("use_local_assignment_fallback").value)
        self.assignment_timeout_s = float(self.get_parameter("assignment_timeout_s").value)

        self.drone_id = self.get_parameter("drone_id").value
        assignment_topic = self.get_parameter("assignment_topic").value
        if not assignment_topic:
            assignment_topic = f"/central/assignment/{self.drone_id}"
        self.assignment_topic = assignment_topic

        wps = layout.get("patrol_waypoints", [])
        self.patrol = [(float(w[0]), float(w[1])) for w in wps] if wps else [
            (-8.0, -8.0), (-8.0, 8.0), (8.0, 8.0), (8.0, -8.0),
        ]
        self.idx = 0
        self.battery = 100.0
        self.x = 0.0
        self.y = 0.0
        self.pose_valid = False
        self._birds = []
        self._bird_states = []
        self._obstacle_poses = []
        self._last_bird_msg_ns = 0
        self._prev_has_target = False
        self._home_until_ns = 0
        self.target_index = -1
        self._assigned_target = -1
        self._last_assignment_ns = 0
        self._last_mode_name = ""

        self._setpoint = PoseStamped()
        self._setpoint.pose.orientation.w = 1.0
        self._setpoint_count = 0
        self._mavros_state = MavrosState()
        self._arm_req_inflight = False
        self._mode_req_inflight = False
        self._last_arm_req_ns = 0
        self._last_mode_req_ns = 0

        pose_topic = self.get_parameter("pose_topic").value
        setpoint_topic = self.get_parameter("setpoint_topic").value
        birds_topic = self.get_parameter("birds_topic").value
        obstacles_topic = self.get_parameter("obstacles_topic").value
        status_topic = self.get_parameter("bird_status_topic").value

        self.setpoint_pub = self.create_publisher(PoseStamped, setpoint_topic, 10)
        self.chased_pub = self.create_publisher(Bool, "/bird/chased", 10)
        self.target_pub = self.create_publisher(Int32, "/coordinator/target_index", 10)
        self.z_pub = self.create_publisher(Marker, "/drone/z_state", 10)

        self.create_subscription(PoseArray, birds_topic, self.birds_cb, 10)
        self.create_subscription(PoseArray, obstacles_topic, self.obstacles_cb, 10)
        self.create_subscription(Int32MultiArray, status_topic, self.status_cb, 10)
        self.create_subscription(Int32, self.assignment_topic, self.assignment_cb, 10)
        self.create_subscription(PoseStamped, pose_topic, self.pose_cb, qos_profile_sensor_data)
        self.create_subscription(MavrosState, "/mavros/state", self.mavros_state_cb, qos_profile_sensor_data)

        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self._patrol_state = PatrolState()
        self._chase_state = ChaseState()
        self._return_state = ReturnState()

        self._decision_tree = Selector([
            Sequence([Condition(lambda c: c.low_battery), Action(lambda c: setattr(c, "mode", MODE_RETURN))]),
            Sequence([Condition(lambda c: c.has_target), Action(lambda c: setattr(c, "mode", MODE_CHASE))]),
            Sequence([Condition(lambda c: c.force_home), Action(lambda c: setattr(c, "mode", MODE_RETURN))]),
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
        self.pose_valid = True

    def birds_cb(self, msg):
        self._birds = list(msg.poses)
        self._last_bird_msg_ns = self._now_ns()

    def status_cb(self, msg):
        self._bird_states = list(msg.data)

    def obstacles_cb(self, msg):
        self._obstacle_poses = list(msg.poses)

    def assignment_cb(self, msg):
        self._assigned_target = int(msg.data)
        self._last_assignment_ns = self._now_ns()

    def mavros_state_cb(self, msg):
        self._mavros_state = msg

    def _bird_state(self, i):
        return int(self._bird_states[i]) if i < len(self._bird_states) else STATE_WANDER

    def _is_active_bird(self, i, x, y):
        return abs(x) <= self.field_xy and abs(y) <= self.field_xy and self._bird_state(i) in (STATE_WANDER, STATE_FLEE)

    def _local_fallback_target(self):
        birds = [
            BirdSnapshot(index=i, x=float(p.position.x), y=float(p.position.y), state=self._bird_state(i))
            for i, p in enumerate(self._birds)
        ]
        d = DroneSnapshot(drone_id=self.drone_id, x=self.x, y=self.y, valid=self.pose_valid)
        return int(assign_nearest_unique([d], birds, self.field_xy).get(self.drone_id, -1))

    def _resolve_target_index(self):
        now_ns = self._now_ns()
        fresh = self._last_assignment_ns > 0 and (now_ns - self._last_assignment_ns) <= int(self.assignment_timeout_s * 1e9)
        if fresh:
            # If centralized assignment is currently "-1", still allow local
            # nearest-active fallback to avoid getting stuck in RETURN.
            if self._assigned_target >= 0:
                idx = self._assigned_target
            else:
                idx = self._local_fallback_target() if self.use_local_assignment_fallback else -1
        else:
            idx = self._local_fallback_target() if self.use_local_assignment_fallback else -1
        if idx < 0 or idx >= len(self._birds):
            return -1
        p = self._birds[idx].position
        return idx if self._is_active_bird(idx, p.x, p.y) else -1

    def _rate_limit_xy(self, x, y):
        # Cap how far each setpoint step can move in XY (smooth OFFBOARD commands).
        if self.setpoint_max_step_m <= 0.0:
            return x, y
        sx, sy = float(self._setpoint.pose.position.x), float(self._setpoint.pose.position.y)
        if self.pose_valid and self._setpoint_count < 2:
            sx, sy = self.x, self.y
        dx, dy = x - sx, y - sy
        dist = math.hypot(dx, dy)
        if dist > self.setpoint_max_step_m:
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

    def _apply_obstacle_avoidance(self, gx, gy):
        # Cylindrical obstacles: repel when distance < (radius + clearance).
        # Push strength grows linearly as we penetrate deeper into the shell.
        if not self._obstacle_poses:
            return gx, gy
        x, y = float(gx), float(gy)
        for qx, qy in ((self.x, self.y), (x, y)):
            for p in self._obstacle_poses:
                cx, cy, radius = p.position.x, p.position.y, p.position.z
                shell = radius + self.obstacle_clearance
                dx, dy = qx - cx, qy - cy
                dist = math.hypot(dx, dy)
                if dist >= shell or dist < 1e-6:
                    continue
                push = (shell - dist) * self.obstacle_repulsion_gain
                ux, uy = dx / dist, dy / dist
                if qx != self.x or qy != self.y:
                    x += push * ux
                    y += push * uy
        return x, y

    def send_goal(self, x, y, z):
        x = max(-self.limit, min(self.limit, x))
        y = max(-self.limit, min(self.limit, y))
        x, y = self._apply_obstacle_avoidance(x, y)
        x = max(-self.limit, min(self.limit, x))
        y = max(-self.limit, min(self.limit, y))
        x, y = self._rate_limit_xy(x, y)
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

    def _run_decision_tree(self, has_target):
        ctx = DecisionContext(
            low_battery=self.enable_low_battery_return and self.battery < self.low_battery_threshold,
            has_target=has_target,
            force_home=self._now_ns() < self._home_until_ns,
            return_home_when_no_targets=self.return_home_when_no_targets,
        )
        self._decision_tree.run(ctx)
        return ctx.mode

    def _select_state(self, mode):
        if mode == MODE_CHASE:
            return self._chase_state
        if mode == MODE_RETURN:
            return self._return_state
        return self._patrol_state

    def update(self):
        if self.battery_drain_per_tick > 0.0:
            self.battery = max(0.0, self.battery - self.battery_drain_per_tick)
        if self._last_bird_msg_ns > 0 and self.bird_timeout_s > 0.0:
            if (self._now_ns() - self._last_bird_msg_ns) / 1e9 > self.bird_timeout_s:
                self._birds = []
        if not self._handle_offboard_bootstrap():
            self.chased_pub.publish(Bool(data=False))
            return
        if not self.pose_valid:
            self.send_goal(self.home_x, self.home_y, self.patrol_z)
            self.chased_pub.publish(Bool(data=False))
            return

        self.target_index = self._resolve_target_index()
        has_target = self.target_index >= 0
        if self._prev_has_target and not has_target and self.demo_post_chase_home_s > 0.0:
            self._home_until_ns = self._now_ns() + int(self.demo_post_chase_home_s * 1e9)
        self._prev_has_target = has_target

        mode = self._run_decision_tree(has_target)
        chased = mode == MODE_CHASE and has_target
        self.chased_pub.publish(Bool(data=chased))
        self.target_pub.publish(Int32(data=self.target_index if chased else -1))

        if mode != self._last_mode_name:
            self._last_mode_name = mode
            self.get_logger().info(f"{self.drone_id} FSM -> {mode}, target={self.target_index if chased else -1}")
        self._select_state(mode).execute(self)


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

