import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class ChasedMaskAggregatorNode(Node):
    """Merge per-drone chased masks into one OR-ed mask for birds_node."""

    def __init__(self):
        super().__init__("chased_mask_aggregator_node")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)
        self.declare_parameter(
            "input_topics",
            ["/birds/chased_mask/drone_1", "/birds/chased_mask/drone_2"],
        )
        self.declare_parameter("bird_status_topic", "/birds/status")
        self.declare_parameter("output_topic", "/birds/chased_mask")
        self.declare_parameter("publish_rate_hz", 20.0)

        self._input_topics = list(self.get_parameter("input_topics").value)
        self._status_topic = str(self.get_parameter("bird_status_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._pub = self.create_publisher(Int32MultiArray, output_topic, 10)

        self._latest_by_topic = {topic: [] for topic in self._input_topics}
        self._bird_count = 0

        self.create_subscription(Int32MultiArray, self._status_topic, self._status_cb, 10)
        for topic in self._input_topics:
            self.create_subscription(Int32MultiArray, topic, self._make_mask_cb(topic), 10)

        hz = float(self.get_parameter("publish_rate_hz").value)
        if hz <= 0.0:
            hz = 20.0
        self.create_timer(1.0 / hz, self._publish_merged)
        self.get_logger().info(
            f"Merging chased masks from {len(self._input_topics)} topics into {output_topic}"
        )

    def _make_mask_cb(self, topic):
        def _cb(msg):
            self._latest_by_topic[topic] = [1 if int(v) > 0 else 0 for v in msg.data]

        return _cb

    def _status_cb(self, msg):
        self._bird_count = len(msg.data)

    def _publish_merged(self):
        target_len = self._bird_count
        if target_len <= 0:
            target_len = max((len(v) for v in self._latest_by_topic.values()), default=0)
        if target_len <= 0:
            return

        merged = [0] * target_len
        for mask in self._latest_by_topic.values():
            for i, value in enumerate(mask[:target_len]):
                if value > merged[i]:
                    merged[i] = value
        self._pub.publish(Int32MultiArray(data=merged))


def main(args=None):
    rclpy.init(args=args)
    node = ChasedMaskAggregatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
