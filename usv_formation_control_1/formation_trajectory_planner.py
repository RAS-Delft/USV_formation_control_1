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

import rclpy
import numpy as np
import argparse
import time
import math
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import pyproj
from std_msgs.msg import Float32
from tf2_ros import TFMessage
# Import transform listener
from tf2_ros import TransformListener
import ras_ros_core_control_modules.tools.geometry_tools as ras_geometry_tools
import ras_ros_core_control_modules.tools.display_tools as ras_display_tools



parser = argparse.ArgumentParser(description='Formation trajectory planner - plans trajectory for a formation based on fixed speed and predefined hardcoded waypoints. A pose of formation is recalculated and broadcasted periodically.')
parser.add_argument("objectID", type=str,help="set formation identifier")
parser.add_argument("-r") # ROS2 arguments
args, unknown = parser.parse_known_args()


OBJECT_ID = args.objectID
PERIOD_RECALCULATE_PATH = 40.0 # seconds
PERIOD_MOVE_REF = 0.10 # seconds
PERIOD_BROADCAST_STATUS = 5.0 # seconds
geodesic = pyproj.Geod(ellps='WGS84')

class Vessel():
	"""
	Container representing a vessel that is registered within a formation.
	"""

	def __init__(self,object_identifier:str,parent_,formationPose_:TFMessage):
		self.name = object_identifier
		self.parent = parent_
		self.publisher_geo_reference = self.parent.create_publisher(NavSatFix, '/'+self.name+'/reference/geopos', 10)
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
		droll, dpitch, dyaw = ras_geometry_tools.euler_from_quaternion(quat.x,quat.y,quat.z,quat.w)

		pose_formation = self.parent.formation_ref_current

		# Find yaw of the vessel
		yaw_= pose_formation[2] + dyaw

		# Calculate the displacement [North,East] in meters
		dNdE = np.matmul(ras_geometry_tools.R_surf(yaw_),np.array([dx,dy]))
		
		# Calculate displacement lat/long (degrees) with respect to the formation CO
		dlat, dlong = ras_geometry_tools.d_northeast_to_d_latlong(dNdE[0],dNdE[1],pose_formation[0])

		# Calculate the position of the reference by adding displacement to the formation CO
		lat = pose_formation[0] + dlat
		long = pose_formation[1] + dlong
		
		return lat,long
		
		
class FormationTrajectoryPlannerNode(Node):
	"""
	Class that represents a formation with n vessels
	Broadcasts formation CO and vessel references, moving along the specified path.
	"""
	def __init__(self):
		super().__init__('Formation_trajectory_planning_node')

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
		
		
		self.timestamp_recalculate_path = 0
		self.timestamp_move_reference = 0
		
		# Collect information on the shape of the formation.
		# This fills an array of Vessel objects (self.vessels[]) that is used to manage subscribers and store data for each vessel in the configuration. 
		self.vessels = []

	
		# Subscribe to the formation configuration
		self.subscription = self.create_subscription(TFMessage,'/'+OBJECT_ID+'/reference/tf_vessels',self.configuration_callback,10)

		# State tracking of the formation
		self.formation_ref_current = self.waypoints[0]
		self.waypoints_smoothed = []
		self.publisher_formation_ref_pos = self.create_publisher(NavSatFix, 'reference/geopos', 10)
		self.yaw_publisher = self.create_publisher(Float32, 'reference/heading', 10)
		self.i_refpos = 1
		self.current_formation_velocity = 0.35

		# Set up timer for main loop that moves the formation reference
		self.timestamp_move_reference = time.time()
		self.replan_trajectory()
		self.timer_moveref = self.create_timer(PERIOD_MOVE_REF, self.moveref)

		# Set up timer for main loop that recalculates the path
		#self.timer_recalculate_path = self.create_timer(PERIOD_RECALCULATE_PATH, self.replan_trajectory)

		# diagnostics
		self.timer_print_statistics = self.create_timer(PERIOD_BROADCAST_STATUS, self.print_statistics)
		self.timestamp_broadcast_status = self.get_clock().now().nanoseconds/1e9
		self.tracker_moveref_callback = 0
				
	def replan_trajectory(self):
		"""
		Plans the trajectory of the formation along the specified waypoints. 
		Smoothes the path along the waypoints.
		"""
		self.timestamp_recalculate_path = time.time()
		self.formation_ref_current = self.waypoints[0]
		# Select the first two columns (lat/long) of the waypoints array
		waypoints = self.waypoints[:, [0, 1]]

		# Smooth with chaikins corner cutting algorithm
		self.waypoints_smoothed = ras_geometry_tools.chaikins_corner_cutting(waypoints,5)

		# set the first waypoint as the current reference
		# concatenate the waypoint array [lat,long] with the angle 0 reference
		self.formation_ref_current = np.concatenate((self.waypoints_smoothed[0], [0]))
		
	def moveref(self):
		"""
		Moves the formation reference a little step forward
		
		Args:
			none

		Returns:
			none
	
		"""		
		# Calculate the time passed since the last call of this function
		now = time.time()
		dt = now - self.timestamp_move_reference

		## Find the distance to be travelled during this timestep according to reference velocity
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
		dlat, dlong = ras_geometry_tools.d_northeast_to_d_latlong(dN,dE,self.formation_ref_current[0])
		
		# move the formation by adding the difference this timestep
		self.formation_ref_current[0] += dlat
		self.formation_ref_current[1] += dlong
		self.formation_ref_current[2] = math.radians(fwd_azimuth)

		# Broadcast formation reference
		msg = NavSatFix()
		msg.latitude = self.formation_ref_current[0]
		msg.longitude = self.formation_ref_current[1]
		self.publisher_formation_ref_pos.publish(msg)

		msg = Float32()
		msg.data = self.formation_ref_current[2]
		self.yaw_publisher.publish(msg)

		# Broadcast individual vessel references
		self.broadcast_vessel_references()

		# Finishing up: Set trackers and timesteps
		self.tracker_moveref_callback += 1
		self.timestamp_move_reference = now
	
	def broadcast_vessel_references(self):
		for vessel in self.vessels:
			# calculate reference
			lat,long = vessel.calculate_ref()

			# Publish reference
			msg = NavSatFix()
			msg.latitude = lat
			msg.longitude = long
			vessel.publisher_geo_reference.publish(msg)
			
	def configuration_callback(self, msg:TFMessage):
		#check if the vessel is already registered in the formation
		match = False

		
		for transform in msg.transforms:
			match = False
			for vessel in self.vessels:
				if vessel.name == transform.child_frame_id:
					match = True

					# Overwrite the formationPose field of the vessel
					vessel.formationPose = transform 
			
			if not match:
				# Vessel is not yet registered
				# Add new vessel to list
				print('['+OBJECT_ID+'][trajectory planner] Adding new vessel to formation: ' + transform.child_frame_id)
				self.vessels.append(Vessel(transform.child_frame_id,self,formationPose_=transform))

	def print_statistics(self):
		""" On a single line, print the rates of all major callbacks in this script. """

		# Calculate passed time
		now = self.get_clock().now().nanoseconds/1e9
		passed_time = now - self.timestamp_broadcast_status

		# Calculate rates up to two decimals
		rate_moveref_callback = round(self.tracker_moveref_callback/passed_time,2)

		# Format information to string
		printstring = ras_display_tools.terminal_fleet_module_string(self.get_namespace()[1:], ['moveref_callback',rate_moveref_callback,'hz'])

		# Print
		self.get_logger().info(printstring)

		# Reset trackers
		self.tracker_moveref_callback = 0
		self.timestamp_broadcast_status = now

def main(args=None):
	rclpy.init(args=args)

	node = FormationTrajectoryPlannerNode()

	# Start the nodes processing thread
	rclpy.spin(node)

	# at termination of the code (generally with ctrl-c) Destroy the node explicitly
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()