#!/usr/bin/env python

"""
Broadcaster of a fixed configuration of a few RAS vessels.
Initial developer: Bart Boogmans (bartboogmans@hotmail.com)
"""

import rospy
import tf
from geometry_msgs.msg import TransformStamped
import argparse

parser = argparse.ArgumentParser(description='get input of formation ID')
parser.add_argument('formation_id', type=str, nargs=1,help='Identifier of the formation')
args, unknown = parser.parse_known_args()

class FormationConfigurationBroadcaster():
	def __init__(self,formation_id):
		self.formation_id = formation_id
		self.node = rospy.init_node(self.formation_id + '_configuration_broadcaster_fixed', anonymous=False)
		self.pub_tf = rospy.Publisher("/"+ self.formation_id +"/reference/tf_vessels", TransformStamped, queue_size=100, latch=True)

	def run(self):
		rate = rospy.Rate(1.0)  # Hz
		while not rospy.is_shutdown():
			static_transformStamped = TransformStamped()
			static_transformStamped.header.stamp = rospy.Time.now()
			static_transformStamped.header.frame_id = "formation1"

			## Triangle configuration

			# Dark-blue Tito Neri
			static_transformStamped.child_frame_id = "RAS_TN_DB"
			static_transformStamped.transform.translation.x = float(0.0)
			static_transformStamped.transform.translation.y = float(2.0)
			static_transformStamped.transform.translation.z = float(0.0)
			quat = tf.transformations.quaternion_from_euler(
						float(0.0),float(0.0),float(0.0))
			static_transformStamped.transform.rotation.x = quat[0]
			static_transformStamped.transform.rotation.y = quat[1]
			static_transformStamped.transform.rotation.z = quat[2]
			static_transformStamped.transform.rotation.w = quat[3]
			self.pub_tf.publish(static_transformStamped)

			# Green Tito Neri
			static_transformStamped.child_frame_id = "RAS_TN_GR"
			static_transformStamped.transform.translation.x = float(-1.0)
			static_transformStamped.transform.translation.y = float(-0.5)
			static_transformStamped.transform.translation.z = float(0.0)
			quat = tf.transformations.quaternion_from_euler(
						float(0.0),float(0.0),float(0.0))
			static_transformStamped.transform.rotation.x = quat[0]
			static_transformStamped.transform.rotation.y = quat[1]
			static_transformStamped.transform.rotation.z = quat[2]
			static_transformStamped.transform.rotation.w = quat[3]
			self.pub_tf.publish(static_transformStamped)

			# Orange Tito Neri
			static_transformStamped.child_frame_id = "RAS_TN_OR"
			static_transformStamped.transform.translation.x = float(0.0)
			static_transformStamped.transform.translation.y = float(0.5)
			static_transformStamped.transform.translation.z = float(0.0)
			quat = tf.transformations.quaternion_from_euler(
						float(0.0),float(0.0),float(0.0))
			static_transformStamped.transform.rotation.x = quat[0]
			static_transformStamped.transform.rotation.y = quat[1]
			static_transformStamped.transform.rotation.z = quat[2]
			static_transformStamped.transform.rotation.w = quat[3]
			self.pub_tf.publish(static_transformStamped)

			rate.sleep()

if __name__ == '__main__':
	formationConfigurationBroadcaster1 = FormationConfigurationBroadcaster(args.formation_id[0])
	formationConfigurationBroadcaster1.run()