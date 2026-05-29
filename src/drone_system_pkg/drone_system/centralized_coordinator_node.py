# Picks which bird each drone should chase; publishes /central/assignment/<drone_id>.

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32, Int32MultiArray

from drone_system.coordination.assignment import STATE_WANDER, assign_nearest_unique
from drone_system.coordination.models import BirdSnapshot, DroneSnapshot
from drone_system.field_layout import load_layout


class CentralizedCoordinatorNode(Node):
    def __init__(self):
        super().__init__("centralized_coordinator_node")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        layout = load_layout()
        self.declare_parameter("birds_topic", "/birds/positions")
        self.declare_parameter("bird_status_topic", "/birds/status")
        self.declare_parameter("field_xy", float(layout.get("field_xy", 10.0)))
        self.declare_parameter("drone_ids", ["drone_1"])
        self.declare_parameter("drone_pose_topics", ["/mavros/local_position/pose"])
        self.declare_parameter("assignment_rate_hz", 10.0)

        self.field_xy = self.get_parameter("field_xy").get_parameter_value().double_value
        self.drone_ids = list(self.get_parameter("drone_ids").get_parameter_value().string_array_value)
        self.pose_topics = list(
            self.get_parameter("drone_pose_topics").get_parameter_value().string_array_value
        )
        if not self.drone_ids:
            self.drone_ids = ["drone_1"]
        if len(self.pose_topics) < len(self.drone_ids):
            fallback = self.pose_topics[-1] if self.pose_topics else "/mavros/local_position/pose"
            self.pose_topics.extend([fallback] * (len(self.drone_ids) - len(self.pose_topics)))

        self._birds = []
        self._states = []
        self._drone_snapshots = {
            drone_id: DroneSnapshot(drone_id=drone_id, x=0.0, y=0.0, valid=False)
            for drone_id in self.drone_ids
        }
        self._assignment_pubs = {
            drone_id: self.create_publisher(Int32, f"/central/assignment/{drone_id}", 10)
            for drone_id in self.drone_ids
        }
        self._assignment_debug_pub = self.create_publisher(Int32MultiArray, "/central/assignments", 10)

        self.create_subscription(PoseArray, self.get_parameter("birds_topic").value, self.birds_cb, 10)
        self.create_subscription(
            Int32MultiArray,
            self.get_parameter("bird_status_topic").value,
            self.status_cb,
            10,
        )

        for i, drone_id in enumerate(self.drone_ids):
            topic = self.pose_topics[i]
            self.create_subscription(
                PoseStamped,
                topic,
                self._make_pose_cb(drone_id),
                qos_profile_sensor_data,
            )
            self.get_logger().info(f"Tracking pose for {drone_id} on {topic}")

        rate = float(self.get_parameter("assignment_rate_hz").value)
        if rate <= 0.0:
            rate = 10.0
        self.create_timer(1.0 / rate, self.update)

    def _make_pose_cb(self, drone_id):
        def _cb(msg):
            self._drone_snapshots[drone_id] = DroneSnapshot(
                drone_id=drone_id,
                x=float(msg.pose.position.x),
                y=float(msg.pose.position.y),
                valid=True,
            )

        return _cb

    def birds_cb(self, msg):
        self._birds = list(msg.poses)

    def status_cb(self, msg):
        self._states = list(msg.data)

    def _bird_snapshots(self):
        snapshots = []
        for i, p in enumerate(self._birds):
            st = self._states[i] if i < len(self._states) else STATE_WANDER
            snapshots.append(
                BirdSnapshot(
                    index=i,
                    x=float(p.position.x),
                    y=float(p.position.y),
                    state=int(st),
                )
            )
        return snapshots

    def update(self):
        drones = [self._drone_snapshots[did] for did in self.drone_ids]
        birds = self._bird_snapshots()
        assigned = assign_nearest_unique(drones, birds, self.field_xy)

        debug = Int32MultiArray()
        for did in self.drone_ids:
            idx = int(assigned.get(did, -1))
            msg = Int32()
            msg.data = idx
            self._assignment_pubs[did].publish(msg)
            debug.data.extend([self.drone_ids.index(did), idx])
        self._assignment_debug_pub.publish(debug)


def main(args=None):
    rclpy.init(args=args)
    node = CentralizedCoordinatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
