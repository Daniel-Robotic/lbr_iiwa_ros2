import os
import rclpy
import numpy as np
import roboticstoolbox as rtb
import matplotlib.pyplot as plt

from rclpy.node import Node
from std_msgs.msg import String

from spatialmath import SE3
from roboticstoolbox import DHRobot

class KukaIIWA7(DHRobot):
	def __init__(self):
		
		mm = 1e-3

		# Denavit-Hartenberg parameters
		# dh = [
		# 	# Link_1
		# 	rtb.RevoluteMDH(
		# 		a=0.0, 
		# 		alpha=-np.pi/2, 
		# 		d=0.340,
		# 		m=3.4525,
		# 		G=1,
		# 		qlim=np.array([-170 * np.pi / 180, 
		# 		   				170 * np.pi / 180]),
		# 		# r=np.array([0, -0.03, 0.12]),
		# 		# I=np.array([0.07465325, 0.057419, 0.02393725, 
		# 		# 			0.008542, 0, 0])
		# 	),

		# 	# Link_2
		# 	rtb.RevoluteMDH(
		# 		a=0.0, 
		# 		alpha=np.pi/2, 
		# 		d=0,
		# 		m=3.4821,
		# 		G=1,
		# 		qlim=np.array([-120 * np.pi / 180, 
		# 		   				120 * np.pi / 180]),
		# 		# r=np.array([0.0003, 0.059, 0.042]),
		# 		# I=np.array([0.0390236145, 0.027932737789, 0.019911503489,
		# 		# 			-0.0086286438, -0.00366987446, -6.163317e-05])
		# 	),

		# 	# Link_3
		# 	rtb.RevoluteMDH(
		# 		a=0.0, 
		# 		alpha=np.pi/2, 
		# 		d=0.4,
		# 		m=4.05623,
		# 		G=1,
		# 		qlim=np.array([-170 * np.pi / 180, 
		# 		   				170 * np.pi / 180]),
		# 		# r=np.array([0, 0.03, 0.13]),
		# 		# I=np.array([0.104240894, 0.078270287, 0.034070607, 
		# 		# 			-0.009592297, 0, 0])
		# 	),

		# 	# Link_4
		# 	rtb.RevoluteMDH(
		# 		a=0.0, 
		# 		alpha=-np.pi/2, 
		# 		d=0,
		# 		m=3.4822,
		# 		G=1,
		# 		qlim=np.array([-120 * np.pi / 180, 
		# 		   				120 * np.pi / 180]),
		# 		# r=np.array([0, 0.067, 0.034]),
		# 		# I=np.array([0.041437019, 0.0247754232, 0.0234165958,	
		# 		# 			-0.0115574516, 0, 0])
		# 	),

		# 	# Link_5
		# 	rtb.RevoluteMDH(
		# 		a=0.0, 
		# 		alpha=-np.pi/2, 
		# 		d=0.4,
		# 		m=2.1633,
		# 		G=1,
		# 		qlim=np.array([-170 * np.pi / 180, 
		# 		   				170 * np.pi / 180]),
		# 		# r=np.array([0.0001, 0.021, 0.076]),
		# 		# I=np.array([0.0263192361, 0.018203242433, 0.012074036933,
		# 		# 			0.0073986268, -1.644108e-05, -4.54293e-06])
		# 	),

		# 	# Link_6
		# 	rtb.RevoluteMDH(
		# 		a=0.0, 
		# 		alpha=np.pi/2, 
		# 		d=0,
		# 		m=2.3466,
		# 		G=1,
		# 		qlim=np.array([-120 * np.pi / 180, 
		# 		   				120 * np.pi / 180]),
		# 		# r=np.array([0, 0.0006, 0.0004]),
		# 		# I=np.array([0.006510220232,	0.006259375456,	0.004527844776,
		# 		# 			0.000318346816, 0, 0])
		# 	),

		# 	# Link_7
		# 	rtb.RevoluteMDH(
		# 		a=0.0, 
		# 		alpha=0, 
		# 		d=0.126,
		# 		m=3.129,
		# 		G=1,
		# 		qlim=np.array([-175 * np.pi / 180, 
		# 		   				175 * np.pi / 180]),
		# 		# r=np.array([0, 0, 0.02]),
		# 		# I=np.array([0.0158916, 0.0159016, 0.002872,
		# 		# 			0, 0, 0.0005912])
		# 	)
		# 
		# ]

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


class IKRobotics(Node):
	def __init__(self):
		super().__init__(node_name="ik_robotics")
		
		self._robot = None
		self.get_logger().info("Started...")

		kuka_robot = KukaIIWA7()
		
		point = SE3(2, 2, 2) * SE3.RPY([1.57, 0, 0], order='xyz')
		print(point)
		q = kuka_robot.ikine_LM(point)
		print(q)
		

def main(args=None):
	rclpy.init(args=args)
	node = IKRobotics()
	# rclpy.spin(node)
	# rclpy.shutdown()

if __name__ == "__main__":
	main()