#!/usr/bin/env python


"""
Generates a trajectory for the formation to follow. 
Current version does the following:
- listens to ships registered in this formation and save their configuration.
- at specified rate A: plan a smoothed path along input waypoints
- at specified rate B: first action: move reference formation position and broadcast the new one
- at specified rate B: second action: for each registered vessel: translate formation reference to specific vessel position reference, and broadcast

Initial developer: Bart Boogmans (bartboogmans@hotmail.com)

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

parser = argparse.ArgumentParser(description='get input of formation ID')
parser.add_argument('name', type=str, nargs=1,help='Identifier of the formation')
args, unknown = parser.parse_known_args()

PERIOD_RECALCULATE_PATH = 40.0 # seconds
PERIOD_MOVE_REF = 0.10 # seconds
PERIOD_BROADCAST_STATUS = 4.0 # seconds
geodesic = pyproj.Geod(ellps='WGS84')


class geo_euclidean_transforms():	
	def d_northeast_to_d_latlong(dN,dE,lat_linearization):
		"""
		Transforms a small step in northeast tangent to earth surface in the difference in latitude longitude.
		Note: 
		- This method approximates earth locally flat and may misbehave around earths prime meredian or poles. 
		- Over long distances this method starts losing accuracy, but it works great and simple for small displacements. 
		- Approximates earth as a sphere

		Args:
			dN, dE float: movement north and east respectively in meters
			lat_linearization (float): latitude on which the linearization takes place. Usually taking the latitude of any near point suffices. 

		Returns:
			dlat,dlong float change in latitude and longitude respectively
	
		"""
		earth_radius = 6371.0e3  # Earth radius in meters
		r_lat = earth_radius * np.cos(np.radians(lat_linearization))
		dlat = np.degrees(dN/earth_radius)
		dlong = np.degrees(dE/r_lat)
		return dlat, dlong
	
	def d_latlong_to_d_northeast(dlat,dlong,lat_linearization):
		"""
		Transforms a small step in latitude longitude to northeast motion tangent to earth surface.
		Note: 
		- This method approximates earth locally flat and may misbehave near earths prime meredian or poles. 
		- Over long distances this method starts losing accuracy, but it works great and simple for small displacements. 
		- Approximates earth as a sphere

		Args:
			dlat, dlong float: change in latitude and longitude respectively
			lat_linearization (float): latitude on which the linearization takes place. Usually taking the latitude of any near point suffices. 

		Returns:
			dN, dE float: movement north and east respectively in meters
	
		"""
		earth_radius = 6371.0e3  # Earth radius in meters
		r_lat = earth_radius * np.cos(np.radians(lat_linearization))
		dN = np.radians(dlat)*earth_radius
		dE = np.radians(dlong)*r_lat
		return dN, dE
	
	def r_surf(yaw):
		return  np.array(((np.cos(yaw),-np.sin(yaw)),(np.sin(yaw),np.cos(yaw))))

def B_spline(waypoints):
	# Made according to guide at:
	# https://www.youtube.com/watch?v=ueUgHvUT2Z0

	x = []
	y = []
	
	for point in waypoints:
		x.append(point[0])
		y.append(point[1])
		
	# order k= 3 and smoothness s=0
	tck, *rest = interpolate.splprep([x,y],k=2,s=0)
	# tck is a tuple containing ths spline's knots, coefficients and degree defining our smoothed path

	# make the smooth spline into a set of waypoints
	u = np.linspace(0,1,num=400) # vector stating how far we are into our path
	smooth = interpolate.splev(u,tck)
	return smooth

def r_surf(yaw):
    return  np.array(((np.cos(yaw),-np.sin(yaw)),(np.sin(yaw),np.cos(yaw))))

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


class Vessel():
	def __init__(self,object_identifier,parent):
		self.name = object_identifier
		self.parent = parent
		self.publisher_geo_reference = rospy.Publisher('/'+self.name+'/reference/geopos', NavSatFix, queue_size=1)
		self.publisher_yaw_reference = rospy.Publisher('/'+self.name+'/reference/yaw', Float32, queue_size=1)
		self.formationPose = TransformStamped

	def calculate_own_pose_from_formation_pose(self):
		quat = self.formationPose.transform.rotation
		trans = self.formationPose.transform.translation
		dx, dy, dz = [trans.x, trans.y, trans.z]
		droll, dpitch, dyaw = euler_from_quaternion(quat.x,quat.y,quat.z,quat.w)
		# rotate placement vector of ship within formation coordinate system to global
		pose_formation = self.parent.formation_ref_current

		yaw_= pose_formation[2] + dyaw

		dNdE = np.matmul(r_surf(yaw_),np.array([dx,dy]))
		print('["'+self.name+'"] dNdE=' + str(dNdE) + 'meters north/east, yaw = '+str(yaw_) + 'dyaw='+str(dyaw) +' pose2='+str(pose_formation[2]))
		#transform displacement in meters to change in lat/long 
		dlat, dlong = geo_euclidean_transforms.d_northeast_to_d_latlong(dNdE[0],dNdE[1],pose_formation[0])
		lat = pose_formation[0] + dlat
		long = pose_formation[1] + dlong
		

		# send 
		msg = NavSatFix()
		msg.latitude = lat
		msg.longitude = long
		self.publisher_geo_reference.publish(msg)

		msg = Float32()
		msg.data = yaw_
		self.publisher_yaw_reference.publish(msg)
		
class FormationTrajectoryPlanner():
	def __init__(self,object_id):
		self.name = object_id
		
		# The waypoint array has latitude longitude and velocity reference, describing the rough outline of profile to follow.
		self.waypoints = np.array([		[52.0014827166619,  4.37188818059471,   0.41],\
										[52.001507485826,   4.37200619779137,   0.42],\
										[52.0015768394125,  4.37204106650856,   0.43],\
										[52.0016527979791,  4.37200083337334,   0.44],\
										[52.0017171975326,  4.3719230493119,	0.45],\
										[52.0016907772142,  4.37178357444313,   0.46],\
										[52.00162472635,	4.37174334130791,   0.47],\
										[52.0015355575287,  4.37179162107018,   0.48],\
										[52.0014827166619,  4.37188818059471,   0.41],\
		])
		
		self.node = rospy.init_node(self.name + '_trajectory_planner', anonymous=False)
		self.timestamp_recalculate_path = 0
		self.timestamp_move_reference = 0
		self.timestamp_broadcast_status = 0
		
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
		self.current_formation_velocity = 0.4
				
	def replan_trajectory(self):
		self.timestamp_recalculate_path = time.time()
		self.formation_ref_current = self.waypoints[0]
		waypoints = self.waypoints[:, [0, 1]]
		self.waypoints_smoothed = np.array(B_spline(waypoints))
		self.i_refpos = 1
	
	def moveref(self,dt):
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
		target = self.waypoints_smoothed[[0,1],self.i_refpos]
		fwd_azimuth,back_azimuth,distance = geodesic.inv(current[1], current[0],target[1],target[0])
		
		## Determine swiching to next waypoint
		if movelength>= distance:
			# if so, go towards the next waypoint
			self.i_refpos+=1
			if self.i_refpos >= len(self.waypoints_smoothed[0,:]):
				self.i_refpos =0 # Start again if at the end of the waypoint array
	
		## Move the formation reference 
		# calculate distance travelled north/east
		dN = np.cos(np.radians(fwd_azimuth)) * movelength
		dE = np.sin(np.radians(fwd_azimuth)) * movelength
		# transform to difference in geo coordinate
		dlat, dlong = geo_euclidean_transforms.d_northeast_to_d_latlong(dN,dE,self.formation_ref_current[0])
		
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
	
	def broadcast_vessel_locations(self):
		for vessel in self.vessels:
			vessel.calculate_own_pose_from_formation_pose()


	def run(self):
		
		self.replan_trajectory()
		rate = rospy.Rate(1000)  # Hz
		while not rospy.is_shutdown():
			now = time.time()
			
			if now - self.timestamp_recalculate_path > PERIOD_RECALCULATE_PATH:
				self.timestamp_recalculate_path = now
				pass # periodic replanning of path currently not implemented
			
			if now - self.timestamp_move_reference > PERIOD_MOVE_REF:
				self.moveref(now - self.timestamp_move_reference)
				self.timestamp_move_reference = now
				self.broadcast_vessel_locations()
			
			if now - self.timestamp_broadcast_status > PERIOD_BROADCAST_STATUS:
				#print('Broacasting system status')
				self.timestamp_broadcast_status = now
				print(self.formation_ref_current)
			
		
			rate.sleep()
			
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
			print('["'+rospy.get_name()+'"] Adding new vessel to formation: ' + msg.child_frame_id)
			newvessel = Vessel(msg.child_frame_id,self)
			newvessel.formationPose = msg 
			self.vessels.append(newvessel)

if __name__ == '__main__':
	formationTrajectoryPlanner1 = FormationTrajectoryPlanner(args.name[0])
	formationTrajectoryPlanner1.run()