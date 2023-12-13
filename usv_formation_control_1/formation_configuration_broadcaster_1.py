#!/usr/bin/env python

"""
Broadcaster of a fixed configuration of a few RAS vessels.
Initial developer: Bart Boogmans (bartboogmans@hotmail.com)
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TFMessage	
from geometry_msgs.msg import TransformStamped
import argparse

parser = argparse.ArgumentParser(description='get input of formation ID')
parser.add_argument('formation_id', type=str,help='Identifier of the formation')
parser.add_argument("-r") # ROS2 arguments
args, unknown = parser.parse_known_args()

FORMATION_ID = args.formation_id

class FormationConfigurationBroadcasterNode(Node):
	def __init__(self):
		super().__init__('formation_configuration_broadcaster_fixed')
		self.pub_tf = self.create_publisher(TFMessage, "/"+ FORMATION_ID +"/reference/tf_vessels", 10)

		self.timer_broadcast = self.create_timer(4.0, self.broadcast_configuration)

	def broadcast_configuration(self):
		msg = TFMessage()

		now = self.get_clock().now()

		# Add three TransformStamped objects to the message
		msg.transforms = [TransformStamped() for i in range(3)]

		# Dark-blue Tito Neri
		msg.transforms[0].header.stamp = now.to_msg()
		msg.transforms[0].header.frame_id = FORMATION_ID
		msg.transforms[0].child_frame_id = "RAS_TN_DB"
		msg.transforms[0].transform.translation.x = float(-1.5)
		msg.transforms[0].transform.translation.y = float(1.5)
		msg.transforms[0].transform.translation.z = float(0.0)
		msg.transforms[0].transform.rotation.x = 0.0
		msg.transforms[0].transform.rotation.y = 0.0
		msg.transforms[0].transform.rotation.z = 0.0
		msg.transforms[0].transform.rotation.w = 1.0

		# Green Tito Neri
		msg.transforms[1].header.stamp = now.to_msg()
		msg.transforms[1].header.frame_id = FORMATION_ID
		msg.transforms[1].child_frame_id = "RAS_TN_GR"
		msg.transforms[1].transform.translation.x = float(-1.5)
		msg.transforms[1].transform.translation.y = float(-1.5)
		msg.transforms[1].transform.translation.z = float(0.0)
		msg.transforms[1].transform.rotation.x = 0.0
		msg.transforms[1].transform.rotation.y = 0.0
		msg.transforms[1].transform.rotation.z = 0.0
		msg.transforms[1].transform.rotation.w = 1.0

		# Orange Tito Neri
		msg.transforms[2].header.stamp = now.to_msg()
		msg.transforms[2].header.frame_id = FORMATION_ID
		msg.transforms[2].child_frame_id = "RAS_TN_OR"
		msg.transforms[2].transform.translation.x = float(1.5)
		msg.transforms[2].transform.translation.y = float(0.0)
		msg.transforms[2].transform.translation.z = float(0.0)
		msg.transforms[1].transform.rotation.x = 0.0
		msg.transforms[1].transform.rotation.y = 0.0
		msg.transforms[1].transform.rotation.z = 0.0
		msg.transforms[1].transform.rotation.w = 1.0

		# Publish message
		self.pub_tf.publish(msg)

def main(args=None):
	rclpy.init(args=args)

	node = FormationConfigurationBroadcasterNode()

	# Start the nodes processing thread
	rclpy.spin(node)

	# at termination of the code (generally with ctrl-c) Destroy the node explicitly
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
	