#!/usr/bin/env python
"""
Plans trajectory for a formation based on fixed speed and predefined hardcoded waypoints. 
- Smoothes waypoints with B-spline approach
- Moves the formation CO reference along the smoothed path at specified speed. 
- Listens for registration of ships in the formation. Keeps track of vessels in the formation.
- Publishes reference point of registered vessels

Researchlab Autonomous Shipping (RAS) Delft

Feel free to contact any contributors if you have any questions about this software:
Bart Boogmans (bartboogmans@hotmail.com https://github.com/bartboogmans)
"""

import rospy
import numpy as np
import argparse
import time
import math
from sensor_msgs.msg import NavSatFix
from scipy import interpolate
import pyproj
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped

import ras_tf_lib.ras_tf_lib1  as rtf
from ras_tf_lib.ras_tf_lib1 import rascolors

parser = argparse.ArgumentParser(description='get input of formation ID')
parser.add_argument('name', type=str, nargs=1,help='Identifier of the formation')
args, unknown = parser.parse_known_args()

PERIOD_RECALCULATE_PATH = 40.0 # seconds
PERIOD_MOVE_REF = 0.10 # seconds
PERIOD_BROADCAST_STATUS = 10.0 # seconds
geodesic = pyproj.Geod(ellps='WGS84')


class Vessel():
	"""
	Container representing a vessel that is registered within a formation.
	"""

	def __init__(self,object_identifier:str,parent,formationPose_:TransformStamped=TransformStamped):
		self.name = object_identifier
		self.parent = parent
		self.publisher_geo_reference = rospy.Publisher('/'+self.name+'/reference/geopos', NavSatFix, queue_size=1)
		self.formationPose = formationPose_

	def calculate_ref(self):
		"""
		Calculates and publishes the reference geograpical poin (lat/long) of the vessel within a formation, 
		based on formation reference and vessels pose within the formation

		param: none
		returns: latitude,longitude of vessel
		"""

		quat = self.formationPose.transform.rotation
		dx, dy, dz = [self.formationPose.transform.translation.x, self.formationPose.transform.translation.y, self.formationPose.transform.translation.z]

		# Transform from quaternions to euler angles
		droll, dpitch, dyaw = rtf.euler_from_quaternion(quat.x,quat.y,quat.z,quat.w)

		pose_formation = self.parent.formation_ref_current

		# Find yaw of the vessel
		yaw_= pose_formation[2] + dyaw

		# Calculate the displacement [North,East] in meters
		dNdE = np.matmul(rtf.R_surf(yaw_),np.array([dx,dy]))
		
		# Calculate displacement lat/long (degrees) with respect to the formation CO
		dlat, dlong = rtf.d_northeast_to_d_latlong(dNdE[0],dNdE[1],pose_formation[0])

		# Calculate the position of the reference by adding displacement to the formation CO
		lat = pose_formation[0] + dlat
		long = pose_formation[1] + dlong
		
		return lat,long
		
		
class FormationTrajectoryPlanner():
	"""
	Class that represents a formation with n vessels
	Broadcasts formation CO and vessel references, moving along the specified path.
	"""
	def __init__(self,object_id:str):
		self.name = object_id
		
		# The waypoint array has latitude longitude and velocity reference, describing the rough outline of profile to follow.
		self.waypoints = np.array([		[52.0014827166619,  4.37188818059471,   0.35],\
										[52.001507485826,   4.37200619779137,   0.35],\
										[52.0015768394125,  4.37204106650856,   0.35],\
										[52.0016527979791,  4.37200083337334,   0.35],\
										[52.0017171975326,  4.3719230493119,	0.35],\
										[52.0016907772142,  4.37178357444313,   0.35],\
										[52.00162472635,	4.37174334130791,   0.35],\
										[52.0015355575287,  4.37179162107018,   0.35],\
		])
		
		self.node = rospy.init_node(self.name + '_trajectory_planner', anonymous=False)
		self.timestamp_recalculate_path = 0
		self.timestamp_move_reference = 0
		self.timestamp_broadcast_status = time.time()
		
		# Collect information on the shape of the formation.
		# This fills an array of Vessel objects (self.vessels[]) that is used to manage subscribers and store data for each vessel in the configuration. 
		self.vessels = []
		self.formation_subscriber = rospy.Subscriber('/'+self.name+'/reference/tf_vessels',TransformStamped,self.configuration_callback)
		

		# State tracking of the formation
		self.formation_ref_current = self.waypoints[0]
		self.waypoints_smoothed = []
		self.publisher_formation_ref_pos = rospy.Publisher('/'+self.name+'/reference/geopos', NavSatFix, queue_size=1)
		self.yaw_publisher = rospy.Publisher('/'+self.name+'/reference/yaw', Float32, queue_size=1)
		self.i_refpos = 1
		self.current_formation_velocity = 0.35




		# diagnostics
		self.tracker_moveref_callback = 0
		self.tracker_num_mainloop_callback = 0
				
	def replan_trajectory(self):
		"""
		Plans the trajectory of the formation along the specified waypoints. 
		Smoothes the path along the waypoints.
		"""
		self.timestamp_recalculate_path = time.time()
		self.formation_ref_current = self.waypoints[0]
		# Select the first two columns (lat/long) of the waypoints array
		waypoints = self.waypoints[:, [0, 1]]
		#self.waypoints_smoothed = np.array(rtf.B_spline(waypoints))

		# Smooth with chaikins corner cutting algorithm
		self.waypoints_smoothed = rtf.chaikins_corner_cutting(waypoints,5)

		# set the first waypoint as the current reference
		# concatenate the waypoint array [lat,long] with the angle 0 reference
		self.formation_ref_current = np.concatenate((self.waypoints_smoothed[0], [0]))
		
	
	def moveref(self,dt:float):
		"""
		Moves the formation reference a little step forward
		
		Args:
			dt [float] timestep in seconds

		Returns:
			none
	
		"""		
		## Find the distance to be travvelled during this timestep according to reference velocity
		movelength = dt*self.current_formation_velocity
		if dt >PERIOD_MOVE_REF*100: # in case of severe delay or startup behaviour
			movelength = 0.0

		## Find azimuth and distance of target waypoint
		current = self.formation_ref_current[[0,1]]
		target = self.waypoints_smoothed[self.i_refpos,[0,1]]
		fwd_azimuth,back_azimuth,distance = geodesic.inv(current[1], current[0],target[1],target[0])

		# Check if the distance travelled is larger than the distance to the next waypoint. If so, switch target to the next waypoint
		# Do this with a maximum of 5 attempts to prevent infinite loops
		attempts = 0
		while movelength>= distance and attempts < 5:
			attempts += 1
			# if so, go towards the next waypoint
			self.i_refpos+=1
			if self.i_refpos >= len(self.waypoints_smoothed):
				self.i_refpos =0 # Start again if at the end of the waypoint array

			# Find distance of target waypoint
			target = self.waypoints_smoothed[self.i_refpos]
			fwd_azimuth,back_azimuth,distance = geodesic.inv(current[1], current[0],target[1],target[0])

		## Move the formation reference 
		# calculate distance travelled north/east
		dN = np.cos(np.radians(fwd_azimuth)) * movelength
		dE = np.sin(np.radians(fwd_azimuth)) * movelength

		# transform to difference in geo coordinate
		dlat, dlong = rtf.d_northeast_to_d_latlong(dN,dE,self.formation_ref_current[0])
		
		# move the formation by adding the difference this timestep
		self.formation_ref_current[0] += dlat
		self.formation_ref_current[1] += dlong
		self.formation_ref_current[2] = math.radians(fwd_azimuth)


		# send 
		msg = NavSatFix()
		msg.latitude = self.formation_ref_current[0]
		msg.longitude = self.formation_ref_current[1]
		self.publisher_formation_ref_pos.publish(msg)

		msg = Float32()
		msg.data = self.formation_ref_current[2]
		self.yaw_publisher.publish(msg)

		self.tracker_moveref_callback += 1
	
	def broadcast_vessel_references(self):
		for vessel in self.vessels:
			# calculate reference
			lat,long = vessel.calculate_ref()

			# Publish reference
			msg = NavSatFix()
			msg.latitude = lat
			msg.longitude = long
			vessel.publisher_geo_reference.publish(msg)


	def run(self):
		# Plan the trajectory of the formation by smoothing the waypoints 
		self.replan_trajectory()
		# Set the formation reference to the first waypoint
		#self.formation_ref_current = np.concatenate((self.waypoints_smoothed[:,self.i_refpos], [0]), axis=0)

		rate = rospy.Rate(1000)  # Hz
		while not rospy.is_shutdown():
			now = time.time()
			self.tracker_num_mainloop_callback += 1
			
			if now - self.timestamp_recalculate_path > PERIOD_RECALCULATE_PATH:
				self.timestamp_recalculate_path = now
				pass # periodic replanning of path currently not implemented
			
			if now - self.timestamp_move_reference > PERIOD_MOVE_REF:
				self.moveref(now - self.timestamp_move_reference)
				self.timestamp_move_reference = now
				self.broadcast_vessel_references()
			
			if now - self.timestamp_broadcast_status > PERIOD_BROADCAST_STATUS:
				self.timestamp_broadcast_status = now

				# Determine system frequencies rounded to two decimals
				freq_moveref = round(self.tracker_moveref_callback / PERIOD_BROADCAST_STATUS, 2)
				freq_mainloop = round(self.tracker_num_mainloop_callback / PERIOD_BROADCAST_STATUS, 2)

				# Make strings of numbers in white if nonzero and red if zero
				freq_moveref_str = str(freq_moveref) if freq_moveref > 0 else rascolors.FAIL + str(freq_moveref) + rascolors.NORMAL
				freq_mainloop_str = str(freq_mainloop) if freq_mainloop > 0 else rascolors.FAIL + str(freq_mainloop) + rascolors.NORMAL

				# Print system state
				print('[' + self.name + '][trajectory planner] ' + 'Mainloop: ' + freq_mainloop_str + ' Hz, moveref_callback: ' + freq_moveref_str + ' Hz')

				# Reset counters
				self.tracker_moveref_callback = 0
				self.tracker_num_mainloop_callback = 0

		
			rate.sleep()
			
	def configuration_callback(self, msg:TransformStamped):
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
			print('['+self.name+'][trajectory planner] Adding new vessel to formation: ' + msg.child_frame_id)
			self.vessels.append(Vessel(msg.child_frame_id,self,formationPose_=msg))

if __name__ == '__main__':
	formationTrajectoryPlanner1 = FormationTrajectoryPlanner(args.name[0])
	formationTrajectoryPlanner1.run()