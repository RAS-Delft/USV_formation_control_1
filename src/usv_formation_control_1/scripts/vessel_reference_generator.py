#!/usr/bin/env python

"""
Generates a vessels reference speed and heading by steering towards the reference state and controlling a fixed distance towards the target. 
This is used in the context of formation control where vessels are metaphorically "dragged along by a virtual rope" attached to the reference.
Control of absolute distance between ship and reference is PID based.

Researchlab Autonomous Shipping Delft - (info: bartboogmans@hotmail.com)

"""

import rospy
import numpy as np
import argparse
import time
import math
from sensor_msgs.msg import NavSatFix
import pyproj
from std_msgs.msg import Float32, Float32MultiArray
import ras_tf_lib.ras_tf_lib1  as rtf
from ras_tf_lib.ras_tf_lib1 import rascolors
from ras_tf_lib.controlUtils import PIDController, generate_rosmsg_PID_status

parser = argparse.ArgumentParser(description='get input of vessel ID')
parser.add_argument('name', type=str, nargs=1,help='Identifier of the vessel')
parser.add_argument('distance', type=float, nargs=1,help='distance between ship and ref')
args, unknown = parser.parse_known_args()


PERIOD_BROADCAST_STATUS = 5.0 # seconds
PERIOD_RECALC_REFERENCES = 0.01 # seconds
PERIOD_BROADCAST_CONTROL_STATUS = 1.0/16.0 # Hz
MIN_PERIOD_CONTROL = 1.0/16.0 # maximum PID update rate is 20Hz
geodesic = pyproj.Geod(ellps='WGS84')

class distancePIDController(PIDController):
	def calc_error(self):
		return  self.state - self.ref

class NomotoFollowingDistanceController():
	def __init__(self,object_id:str,distance_:float):
		self.tstart = time.time()
		self.name = object_id
		self.distance = distance_

		# Look if vessel_id is a field of the rascolors class. Set self.color to the corresponding color or otherwise VESSEL_COLOR_DEFAULT
		self.color = getattr(rascolors, self.name, rascolors.VESSEL_COLOR_DEFAULT)
	
		self.tlast_control = 0
		self.pid_state_updated = 0
		self.timestamp_broadcast_status = time.time()
		self.geopos_ref = []
		self.geopos_state = []
		self.velocityPID = distancePIDController(0.25,0.00,0.0,ref_init=distance_) # ki was 0.018
		self.velocityPID.integral_limits=[-66.0,66.0] # based on benchmark error of 2.0m over a period of 8s
		self.velocityPID.output_limits=[0.25,0.5]

		# Communication
		self.node = rospy.init_node(self.name + '_vel_heading_reference_generator', anonymous=False)
		self.publisher_ref_heading = rospy.Publisher('/'+self.name+'/reference/yaw', Float32, queue_size=1)
		self.publisher_distance = rospy.Publisher('/'+self.name+'/diagnostics/refposdistance', Float32, queue_size=1)
		self.publisher_ref_velocity = rospy.Publisher('/'+self.name+'/reference/velocity', Float32MultiArray, queue_size=1)
		self.reference_subscriber = rospy.Subscriber('/'+self.name+'/reference/geopos',NavSatFix,self.callback_reference)
		self.state_geopos_subscriber = rospy.Subscriber('/'+self.name+'/state/geopos',NavSatFix,self.callback_state_geopos)
		self.pid_status_publisher = rospy.Publisher('/'+self.name+'/diagnostics/distanceController/pid_status', Float32MultiArray, queue_size=1)
		#self.publisher_filtered_distance = rospy.Publisher('/'+self.name+'/diagnostics/filtered_distance', Float32, queue_size=1)
		
		# Diagnostics
		self.tracker_num_ref_callback = 0
		self.tracker_num_state_callback  = 0
		self.tracker_num_control_callback  = 0
		self.tracker_num_mainloop_callback = 0

		self.timestamp_broadcast_control_status=0

				
	def callback_reference(self,msg:NavSatFix):
		self.tracker_num_ref_callback += 1
		self.last_ref_update = time.time()
		self.geopos_ref = [msg.latitude,msg.longitude]
		self.pid_state_updated = 1

	def callback_state_geopos(self,msg:NavSatFix):
		self.tracker_num_state_callback += 1
		self.last_state_geopos_update = time.time()
		self.geopos_state = [msg.latitude,msg.longitude]
		self.pid_state_updated = 1
	
	def run_controls(self):
		"""
		Calculate reference velocity and heading for the ship
		"""
		if self.geopos_ref and self.geopos_state:
			# Calculate azimuth and distance between ship and reference
			fwd_azimuth_deg,back_azimuth_deg,distance = geodesic.inv(self.geopos_state[1], self.geopos_state[0],self.geopos_ref[1],self.geopos_ref[0])

			# Run PID controller to maintain desired distance between ship and reference
			self.velocityPID.setState(distance)
			ref_vel_surge = self.velocityPID.compute()

			# Broadcast messages
			msg = Float32MultiArray()
			msg.data = [ref_vel_surge,0,0]
			self.publisher_ref_velocity.publish(msg)
			
			msg = Float32()
			msg.data = math.radians(fwd_azimuth_deg)
			self.publisher_ref_heading.publish(msg)

			msg = Float32()
			msg.data = distance
			self.publisher_distance.publish(msg)

	def run(self):
		rate = rospy.Rate(1000)  # Hz
		while not rospy.is_shutdown():
			now = time.time()
			self.tracker_num_mainloop_callback += 1

			if now-self.tlast_control>MIN_PERIOD_CONTROL and self.pid_state_updated:
				self.tlast_control = now
				self.tracker_num_control_callback += 1
				self.run_controls()

			if now - self.timestamp_broadcast_status > PERIOD_BROADCAST_STATUS:
				self.timestamp_broadcast_status = now
				
				# Determine system frequencies rounded to two decimals
				freq_ref_callback = round(self.tracker_num_ref_callback/PERIOD_BROADCAST_STATUS,2)
				freq_gepos_state_callback = round(self.tracker_num_state_callback/PERIOD_BROADCAST_STATUS,2)
				freq_control_callback = round(self.tracker_num_control_callback/PERIOD_BROADCAST_STATUS,2)
				freq_mainloop_callback = round(self.tracker_num_mainloop_callback/PERIOD_BROADCAST_STATUS,2)

				# Make strings of numbers without color if above zero and and rascolors.FAIL otherwise
				freq_ref_callback_str = rascolors.OKGREEN + str(freq_ref_callback) + rascolors.NORMAL if freq_ref_callback > 0 else rascolors.FAIL + str(freq_ref_callback) + rascolors.NORMAL
				freq_gepos_state_callback_str = rascolors.OKGREEN + str(freq_gepos_state_callback) + rascolors.NORMAL if freq_gepos_state_callback > 0 else rascolors.FAIL + str(freq_gepos_state_callback) + rascolors.NORMAL
				freq_control_callback_str = rascolors.OKGREEN + str(freq_control_callback) + rascolors.NORMAL if freq_control_callback > 0 else rascolors.FAIL + str(freq_control_callback) + rascolors.NORMAL
				freq_mainloop_callback_str = rascolors.OKGREEN + str(freq_mainloop_callback) + rascolors.NORMAL if freq_mainloop_callback > 0 else rascolors.FAIL + str(freq_mainloop_callback) + rascolors.NORMAL

				# Print system state
				print(' '+self.color + self.name + rascolors.NORMAL +' [Velocity/yaw reference calculator]['+str(round(time.time()-self.tstart,2)) + 's] freq ref callback: ' + freq_ref_callback_str + ' Hz, freq state callback: ' + freq_gepos_state_callback_str + ' Hz, freq control callback: ' + freq_control_callback_str + ' Hz, freq mainloop callback: ' + freq_mainloop_callback_str + ' Hz')

			
				# Reset counters
				self.tracker_num_ref_callback = 0.0
				self.tracker_num_state_callback  = 0.0
				self.tracker_num_control_callback  = 0.0
				self.tracker_num_mainloop_callback = 0.0

			if now - self.timestamp_broadcast_control_status > PERIOD_BROADCAST_CONTROL_STATUS:
				self.timestamp_broadcast_control_status = now

				# Publish PID status
				self.pid_status_publisher.publish(generate_rosmsg_PID_status(self.velocityPID))

			rate.sleep()

if __name__ == '__main__':
	vesselcontroller = NomotoFollowingDistanceController(args.name[0],args.distance[0])
	vesselcontroller.run()