import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.cb, qos)
        print("Subscribed. Waiting for messages...")

    def cb(self, msg):
        print(f"Received msg: x={msg.x}, y={msg.y}, z={msg.z}")

rclpy.init()
node = TestNode()
rclpy.spin(node)

