#!/usr/bin/env python

"""
Estimator of the state of a formation of vessels, by combining the known position of multiple vessels and their pose w.r.t. the centre origin of the formation.
Initial developer: Bart Boogmans (bartboogmans@hotmail.com)


Notes on functionality:
- This current version does not support removal of vessels from a formation.
- Does not function properly around earth's prime meredian as no unwrap is implemented there. 


"""

import rospy
import argparse
import time
import math
import numpy as np
from geometry_msgs.msg import TransformStamped
from geographic_msgs.msg import GeoPose
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32


parser = argparse.ArgumentParser(description='get input of formation ID')
parser.add_argument('formation_id', type=str, nargs=1,help='Identifier of the formation')
args, unknown = parser.parse_known_args()

MAX_BROADCAST_PERIOD = 0.20 # seconds
VESSEL_POSE_TIMEOUT = 5.0 # seconds

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def r_surf(yaw):
	return  np.array(((np.cos(yaw),-np.sin(yaw)),(np.sin(yaw),np.cos(yaw))))

def delta_northeast_to_delta_longlat(dn_de,lat):
	"""
	Calculates displacement in degrees (longitude & latitude) if the object would move a certain amount of meters 
	tangent to earth surface north and east.

	Args:
	dn_de (float[2]): vector with 2 elements pertaining movement north and east respectively
	lat (float): latitude on which the linearization takes place. Any near point. 

	Returns:
	[dlat,dlong] (float[2]) change in latitude and longitude respectively
	"""
	earth_radius = 6371.0e3  # Earth radius in meters
	r_lat = earth_radius * np.cos(np.radians(lat))
	dlat = np.degrees(dn_de[0]/earth_radius)
	dlong = np.degrees(dn_de[1]/r_lat)
	return np.array([dlat,dlong])

def geo_transform_surface(pose_init,transform):
	"""
	Transforms pose_init across surface of earth (linearized over a short distance)
	Initial pose is transformed by first doing translation and secondly rotation. 
	The transformation operation is expressed in initial coordinate system
	
	Args:
	pose_init (float[3]): The initial pose of the object [latitude (deg), longitude (deg), yaw(radians)]
	transform (float[3]): The transformation from object to new coordinate system [dx (meters), dy (meter), d_yaw (radians)]

	Returns:
	pose_out (float[3])
	"""

	yaw_formation = pose_init[2] - transform[2]
	R_formation = r_surf(yaw_formation)
	dn_de = -np.matmul(R_formation,transform[0:2])

	dlat_dlong = delta_northeast_to_delta_longlat(dn_de,pose_init[0])

	lat_long_new = np.array(pose_init[0:2]) + dlat_dlong
	return np.concatenate((lat_long_new, np.array([yaw_formation])), axis=0)

class Vessel():
	def __init__(self,name,parent):
		self.name = name
		self.parent = parent
		self.lastPose = [0.0,0.0,0.0]
		self.lastPosUpdateTimestamp = 0
		self.lastYawUpdateTimestamp = 0
		self.formationPose = TransformStamped
		self.formationPose_x_y_yaw = [0.0,0.0,0.0]
		self.geopos_subscriber = rospy.Subscriber('/'+self.name+'/state/geopos',NavSatFix,self.pose_callback)
		self.heading_subscriber = rospy.Subscriber('/'+self.name+'/state/yaw',Float32,self.yaw_callback)

	def pose_callback(self,msg):
		self.lastPose[0:2] = [msg.latitude,msg.longitude]
		self.lastPosUpdateTimestamp = time.time()
		self.parent.broadcast_callback()

	def yaw_callback(self,msg):
		self.lastPose[2] = msg.data
		self.lastYawUpdateTimestamp = time.time()

	def calculate_formation_pose(self):
		quat = self.formationPose.transform.rotation
		trans = self.formationPose.transform.translation
		dx, dy, dz = [trans.x, trans.y, trans.z]
		roll, pitch, yaw = euler_from_quaternion(quat.x,quat.y,quat.z,quat.w)
		transform_surf = [dx,dy,roll]
		return geo_transform_surface(self.lastPose,transform_surf)


class FormationStateEstimatorNode():
	def __init__(self,formation_id):
		self.formation_id = formation_id
		self.vessels = []
		self.tlast = time.time()
		self.node = rospy.init_node('formation_controller', anonymous=True)
		
		# Collect information on the shape of the formation.
		# This fills an array of Vessel objects (self.vessels[]) that is used to manage subscribers and store data for each vessel in the configuration. 
		self.formation_subscriber = rospy.Subscriber('/'+self.formation_id+'/reference/tf_vessels',TransformStamped,self.configuration_callback)
		
		# Make broadcasters for estimated state
		self.geopos_publisher = rospy.Publisher('/'+self.formation_id+'/state/geopos', NavSatFix, queue_size=1)
		self.yaw_publisher = rospy.Publisher('/'+self.formation_id+'/state/yaw', Float32, queue_size=1)
		
	def configuration_callback(self, msg):
		#check if the vessel is already registered in the formation
		match = False
		for vessel in self.vessels:
			if vessel.name == msg.child_frame_id:
				# vessel is already registered
				match = True
				vessel.formationPose = msg

		if not match:
			# Vessel is not yet registered
			# Add new vessel to list
			print('["'+self.formation_id+'" formation state estimator] Adding new vessel to formation: ' + msg.child_frame_id)

			newvessel = Vessel(msg.child_frame_id,self)
			newvessel.formationPose = msg 
			self.vessels.append(newvessel)



	def broadcast_callback(self):

		# Rate management
		now = time.time()
		if now > self.tlast + MAX_BROADCAST_PERIOD:

			# Beginning of estimation of formation state
			total = [0,0,0]
			num_valid_ships = 0

			# check for pose timeout
			for vessel in self.vessels:
				if now - vessel.lastPosUpdateTimestamp < VESSEL_POSE_TIMEOUT and now - vessel.lastYawUpdateTimestamp < VESSEL_POSE_TIMEOUT:
					formationposition = vessel.calculate_formation_pose()
					total = np.array(total) + formationposition
					num_valid_ships+=1
			
			if num_valid_ships>0:
				output = total / num_valid_ships
				msg = NavSatFix()
				msg.latitude = output[0]
				msg.longitude = output[1]
				self.geopos_publisher.publish(msg)

				msg = Float32()
				msg.data = output[2]
				self.yaw_publisher.publish(msg)

			# set timestamp for rate management
			self.tlast = now

	def run(self):
		rate = rospy.Rate(2)  # Hz
		while not rospy.is_shutdown():

			rate.sleep()

if __name__ == '__main__':
	pid_controller_node = FormationStateEstimatorNode(args.formation_id[0])
	pid_controller_node.run()
