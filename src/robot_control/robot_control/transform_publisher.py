import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler


class TransformPublisher(Node):
	def __init__(self):
		super().__init__(node_name="transform_publisher")
		self._broadcaster = StaticTransformBroadcaster(self)
		self._timer = self.create_timer(1.0, self.timer_callback)

	def timer_callback(self):
		t = TransformStamped()
		t.header.stamp = self.get_clock().now().to_msg()
		t.header.frame_id = "world"
		t.child_frame_id = "base"
		t.transform.translation.x = 1.0
		t.transform.translation.y = 0.0
		t.transform.translation.z = 0.0

		x, y, z, w = quaternion_from_euler(0, 0, 0)
		t.transform.rotation.x = x
		t.transform.rotation.y = y
		t.transform.rotation.z = z
		t.transform.rotation.w = w

		self._broadcaster.sendTransform(t)


def main(args=None):
	rclpy.init(args=args)
	node = TransformPublisher()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()