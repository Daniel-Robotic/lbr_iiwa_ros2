import rclpy
import numpy as np
import pandas as pd # type: ignore
import rclpy.clock
from rclpy.node import Node
from std_msgs.msg import Header
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy


class PositionPublisher(Node):
	def __init__(self, csv_data: str, sep:str=";"):
		super().__init__("position_publisher")

		self.__df = pd.read_csv(csv_data, sep=sep)
		qos_setting = QoSProfile(
			reliability=QoSReliabilityPolicy.BEST_EFFORT,
			durability=QoSDurabilityPolicy.VOLATILE,
			history=QoSHistoryPolicy.KEEP_LAST,
			depth=1
		)

		self.__publisher = self.create_publisher(JointTrajectory, 'LBRiiwa7R800/cmd_positions', qos_setting)
		self.create_service(srv_type=Empty,
					 		srv_name="LBRiiwa7R800/publish_data",
							callback=self.__publish_data)
		

	def __publish_data(self, requset, response):
		

		msg = JointTrajectory()
		msg.header = Header()
		msg.header.stamp = rclpy.clock.Clock().now().to_msg()
		msg.header.frame_id = ""

		msg.joint_names = ["lbr_A1", "lbr_A2", "lbr_A3",
							"lbr_A4", "lbr_A5", "lbr_A6",
							"lbr_A7"]

		for index, row in self.__df.iterrows():
			# if index == 0:
			# 	continue
		
			msg.points.append(JointTrajectoryPoint(positions=row.to_list()))

		self.__publisher.publish(msg)

		return Empty.Response()



def main(args=None):
	rclpy.init(args=None)

	position_publisher = PositionPublisher(csv_data="~/ros2_ws/src/iiwa_controller/config/test_move.csv", 
											sep=";")
	rclpy.spin(position_publisher)
	position_publisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
