import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



class TrajectoryTest(Node):
	def __init__(self):
		super().__init__(node_name="trajectory_test")
		topic_name = "/joint_trajectory_controller/joint_trajectory"
		self._trajectory_publisher = self.create_publisher(JointTrajectory,
													 topic=topic_name,
													 qos_profile=10)
		self._timer = self.create_timer(1.0, self.timer_callback)
		self._joint_names = ["joint_1", "joint_2", "joint_3",
							 "joint_4", "joint_5", "joint_6",
							 "joint_7"
							]
		self._goal_positions = [0, 0, 0, 
						  		0, 0, 0, 
								0]

	def timer_callback(self):
		point = JointTrajectoryPoint(positions=self._goal_positions)

		trajectory_msg = JointTrajectory(joint_names=self._joint_names,
								   		 time_from_start=Duration(sec=2))
		trajectory_msg.points.append(point)
		
		for i in range(10):
			print(JointTrajectoryPoint(positions=[0.2 * i for _ in range(7)]))
			trajectory_msg.points.append(JointTrajectoryPoint(positions=[0.2 * i for _ in range(7)]))
		

		self._trajectory_publisher.publish(trajectory_msg)
		


def main(args=None):
	rclpy.init(args=args)
	node = TrajectoryTest()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()