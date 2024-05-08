#!/usr/bin/env python

import rospy
import argparse
import time
import math
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray
import ras_tf_lib.ras_tf_lib1  as rtf
from ras_tf_lib.ras_tf_lib1 import rascolors as rascolors
import ras_tf_lib.controlUtils as rasControlUtils

parser = argparse.ArgumentParser(description='get input of ship ID')
parser.add_argument('vessel_id', type=str, nargs=1,help='Vessel Identifier of the specified ship')
args, unknown = parser.parse_known_args()
PERIOD_BROADCAST_STATUS = 5.0 # seconds
MIN_PERIOD_CONTROL = 1.0/20.0 # seconds

class ControlEffortAllocator():
	""" allocate control efforts to actuators of the vessel"""

	def __init__(self,object_id):
		self.tstart = time.time()
		self.name = object_id

		# Look if vessel_id is a field of the rascolors class. Set self.color to the corresponding color or otherwise VESSEL_COLOR_DEFAULT
		self.color = getattr(rascolors, self.name, rascolors.VESSEL_COLOR_DEFAULT)
	
		self.node = rospy.init_node(self.name + '_heading_controller', anonymous=False)
		self.timestamp_broadcast_status = 0
		
		# Storing the current setpoints of the control efforts
		self.torque = 0
		self.force_surge = 0
		
		# Import the lambda functions that denote the relation between thruster usage and force
		if object_id == "RAS_TN_OR":
			# The orange ship has different thrusers than the other ships, currently. 
			self.forceToThrust = [rasControlUtils.TN_force_to_thrusterusage1("aft_red"),rasControlUtils.TN_force_to_thrusterusage1("aft_red"),rasControlUtils.TN_force_to_thrusterusage1("bowThruster")]
		else:
			self.forceToThrust = [rasControlUtils.TN_force_to_thrusterusage1("aft_black"),rasControlUtils.TN_force_to_thrusterusage1("aft_black"),rasControlUtils.TN_force_to_thrusterusage1("bowThruster")]
			
		# Subscribers and publishers
		self.subscriber_torque = rospy.Subscriber('/'+self.name+'/reference/controlEffort/torqueZ', Float32,self.callback_torque)
		self.subscriber_force_surge = rospy.Subscriber('/'+self.name+'/reference/controlEffort/forceX', Float32,self.callback_force_surge)
		self.publisher_actuation = rospy.Publisher('/'+self.name+'/reference/actuation', Float32MultiArray, queue_size=1)

		# Rate tracking of allocation protocol
		self.tlast_allocate = 0

		# Diagnostics
		self.resetTrackers()

	def resetTrackers(self):
		self.tracker_callback_torque = 0
		self.tracker_callback_force_surge = 0
		self.tracker_callback_run_allocationProtocol = 0
		self.tracker_callback_mainloop = 0
				
	def callback_torque(self,msg):
		self.tracker_callback_torque += 1
		self.torque = msg.data
		self.torque_set = 1
		self.allocate_forces()

	def callback_force_surge(self,msg):
		self.tracker_callback_force_surge += 1
		self.force_surge = msg.data
		self.force_surge_set = 1
		self.allocate_forces()
	
	def allocate_forces(self):
		# run controls if the last control was more than MIN_PERIOD_CONTROL seconds ago
		if time.time()-self.tlast_allocate>MIN_PERIOD_CONTROL:
			self.tracker_callback_run_allocationProtocol += 1
			# The two thrusters are treated as one thruster with two times the force at identical angles.
			# The location of the thruster with respect to the center origin of the vessel (in meters, x direction)
			r_thr = -0.42
			
			# maximum thrust output of each regular aft thruster
			max_thrust = 2.5 # N

			# Calculate the resultant force in x direction
			force_x = self.force_surge

			# Calculate the resultant force in y direction that the thruster needs to give to yield the desired torque
			force_y = self.torque/r_thr
			
			# Calculate the absolute force and angle of the resultant force per thruster
			force_abs = math.sqrt(force_x**2 + force_y**2)/2
			force_angle = math.atan2(force_y,force_x)
			
			# limit force_abs to max_thrust
			if force_abs > max_thrust:
				force_abs = max_thrust  
			
			# force per aft thruster is looked up from previously determined relation between force and thruster usage in rpm
			aft_propeller_vel_portside = self.forceToThrust[0](force_abs)*60
			aft_propeller_vel_starboard = self.forceToThrust[1](force_abs)*60

			# Publish actuation
			msg = Float32MultiArray()
			msg.data = [aft_propeller_vel_portside,aft_propeller_vel_starboard,float('nan'),force_angle,force_angle]
			self.publisher_actuation.publish(msg)
			
			self.tlast_allocate = time.time()

			# Print vessel name, force_abs, force_angle, aft propeller vel_portside.
			#print(f' {self.color}{self.name}{rascolors.NORMAL} force_abs: {force_abs:.2f} N, force_angle: {force_angle:.2f} rad, aft propeller vel_portside: {aft_propeller_vel_portside:.2f} rpm')
	
	def run(self):
		rate = rospy.Rate(1000)  # Hz
		while not rospy.is_shutdown():
			now = time.time()
			self.tracker_callback_mainloop += 1
			if now - self.timestamp_broadcast_status > PERIOD_BROADCAST_STATUS:
				self.timestamp_broadcast_status = now

				# Determine system frequencies rounded to two decimals
				freq_callback_run_allocation = round(self.tracker_callback_run_allocationProtocol / PERIOD_BROADCAST_STATUS, 2)
				freq_callback_mainloop = round(self.tracker_callback_mainloop / PERIOD_BROADCAST_STATUS, 2)
				freq_callback_torque = round(self.tracker_callback_torque / PERIOD_BROADCAST_STATUS, 2)
				freq_callback_force_surge = round(self.tracker_callback_force_surge / PERIOD_BROADCAST_STATUS, 2)
				
				# Make strings of numbers without color if above zero and and rascolors.FAIL otherwise
				freq_main_loop_str = rascolors.OKGREEN +str(freq_callback_mainloop)  + rascolors.NORMAL if freq_callback_mainloop > 0 else rascolors.FAIL + str(freq_callback_mainloop) + rascolors.NORMAL
				freq_callback_run_allocation_str = rascolors.OKGREEN +str(freq_callback_run_allocation)  + rascolors.NORMAL if freq_callback_run_allocation > 0 else rascolors.FAIL + str(freq_callback_run_allocation) + rascolors.NORMAL
				freq_callback_torque_str = rascolors.OKGREEN +str(freq_callback_torque)  + rascolors.NORMAL if freq_callback_torque > 0 else rascolors.FAIL + str(freq_callback_torque) + rascolors.NORMAL
				freq_callback_force_surge_str = rascolors.OKGREEN +str(freq_callback_force_surge)  + rascolors.NORMAL if freq_callback_force_surge > 0 else rascolors.FAIL + str(freq_callback_force_surge) + rascolors.NORMAL

				# Print system state
				print(f' {self.color}{self.name}{rascolors.NORMAL} [Control effort allocator][{round(time.time()-self.tstart,2)}s] freq_callback_torque: {freq_callback_torque_str} Hz, freq_callback_force_surge: {freq_callback_force_surge_str} Hz, freq_callback_run_allocation: {freq_callback_run_allocation_str} Hz, freq_callback_mainloop: {freq_main_loop_str} Hz')

				self.resetTrackers()
			
			rate.sleep()

if __name__ == '__main__':
	controlEffortAllocator = ControlEffortAllocator(args.vessel_id[0])
	controlEffortAllocator.run()