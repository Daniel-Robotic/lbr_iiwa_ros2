import sys
import rclpy
import numpy as np
import roboticstoolbox as rtb

from rclpy.node import Node
from spatialmath import SE3
from roboticstoolbox import DHRobot
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ros_gz_interfaces.msg import Contacts


class KukaIIWA7(DHRobot):
	def __init__(self):
		
		mm = 1e-3

		dh = [
			rtb.RevoluteDH(a=0.0, alpha=-np.pi/2, d=0.340, qlim=np.array([-170 * np.pi / 180, 170 * np.pi / 180])),
			rtb.RevoluteDH(a=0.0, alpha=np.pi/2, d=0, qlim=np.array([-120 * np.pi / 180, 120 * np.pi / 180])),
			rtb.RevoluteDH(a=0.0, alpha=np.pi/2, d=0.4, qlim=np.array([-170 * np.pi / 180, 170 * np.pi / 180])),
			rtb.RevoluteDH(a=0.0, alpha=-np.pi/2, d=0, qlim=np.array([-120 * np.pi / 180, 120 * np.pi / 180])),
			rtb.RevoluteDH(a=0.0, alpha=-np.pi/2, d=0.4, qlim=np.array([-170 * np.pi / 180, 170 * np.pi / 180])),
			rtb.RevoluteDH(a=0.0, alpha=np.pi/2, d=0, qlim=np.array([-120 * np.pi / 180, 120 * np.pi / 180])),
			rtb.RevoluteDH(a=0.0, alpha=0, d=0.126, qlim=np.array([-175 * np.pi / 180, 175 * np.pi / 180])),
		]

		super().__init__(dh, name="IIWA7", manufacturer="KUKA")

		self.qr = np.zeros(7)
		self.qz = np.zeros(7)

		self.addconfiguration("qr", self.qr)
		self.addconfiguration("qz", self.qz)


class KinematicSolver(Node):
	def __init__(self):
		
		super().__init__(node_name="kinematic_solver")

		self.declare_parameter('x', 0.0)
		self.declare_parameter('y', 0.0)
		self.declare_parameter('z', 0.0)
		self.declare_parameter('a', 0.0)
		self.declare_parameter('b', 0.0)
		self.declare_parameter('c', 0.0)

		topic_name = "/joint_trajectory_controller/follow_joint_trajectory"
		self._action_client = ActionClient(self, 
									 		FollowJointTrajectory, 
									 		topic_name)
		server_reached = self._action_client.wait_for_server(timeout_sec=60.0)
		if not server_reached:
			self.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
			sys.exit()

		self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 1)
		self.current_joint_state = None


		self._joint_names = ["joint_1", "joint_2", "joint_3",
							 "joint_4", "joint_5", "joint_6",
							 "joint_7"]
		
		self._robot = KukaIIWA7()
		self._pose = (self.get_parameter('x').value,
					  self.get_parameter('y').value,
					  self.get_parameter('z').value)
		self._rotation = (self.get_parameter('a').value,
						  self.get_parameter('b').value,
						  self.get_parameter('c').value)
		
		self.get_logger().info("Started...")

		self.create_timer(3.0, self.send_trajectory)

	def joint_states_callback(self, msg):
		self.current_joint_state = msg.position

	def create_joint_trajectory_goal(self):
		se3 = SE3(*self._pose) * SE3.RPY(self._rotation, order='xyz')
		ik_solver = self._robot.ikine_LM(se3)
		
		goal_msg = FollowJointTrajectory.Goal()

		point = JointTrajectoryPoint(positions=ik_solver.q,)
		trajectory = JointTrajectory(joint_names=self._joint_names,
									points=[point],
									time_from_start=Duration(sec=2))

		goal_msg.trajectory = trajectory

		return goal_msg
		
	def send_trajectory(self):
		goal_msg = self.create_joint_trajectory_goal()
		
		self.get_logger().info('Sending joint trajectory goal...')
		send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
		send_goal_future.add_done_callback(self.goal_response_callback)

	def feedback_callback(self, feedback_msg):
		self.get_logger().info(f"Feedback recive: {feedback_msg.feedback}")

	def goal_response_callback(self, future):
		goal_handler = future.result()

		if not goal_handler:
			self.get_logger().error("Goal was rejected :(")
			return
		
		self.get_logger().info("Goal accepted, waiting for result...")
		result_future = goal_handler.get_result_async()
		result_future.add_done_callback(self.get_result_callback)

	def get_result_callback(self, future):
		result = future.result().result

		if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
			self.get_logger().info('Trajectory execution succeeded!')
		else:
			self.get_logger().error(f'Trajectory execution failed with error code: {result.error_code}')


	def kinematic_solver(self):

		if self._goal_in_progress:
			self.get_logger().info("Goal in progress, skipping new goal")
			return

		se3 = SE3(*self._pose) * SE3.RPY(self._rotation, order='xyz')
		ik_solver = self._robot.ikine_LM(se3)

		if not ik_solver.success:
			self.get_logger().warn("The received coordinates are not achievable")
			return

		self.get_logger().info("Moving robot...")
		point = JointTrajectoryPoint(positions=ik_solver.q)
		trajectory_msg = JointTrajectory(joint_names=self._joint_names,
											points=[point])
		
		goal_msg = FollowJointTrajectory.Goal()
		goal_msg.trajectory = trajectory_msg
		goal_msg.goal_time_tolerance = Duration(sec=2)

		self._action_client.wait_for_server()
		self._send_goal_future = self._action_client.send_goal_async(goal_msg, 
																	 feedback_callback=self.feedback_callback)
		self._send_goal_future.add_done_callback(self.goal_response_callback)
		self._goal_in_progress = True


def main(args=None):
	rclpy.init(args=args)
	node = KinematicSolver()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()