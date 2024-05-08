#!/usr/bin/env python

import rospy
import argparse
import time
import math
from std_msgs.msg import Float32, Float32MultiArray
import ras_tf_lib.ras_tf_lib1  as rtf
from ras_tf_lib.ras_tf_lib1 import rascolors as rascolors

from ras_tf_lib.controlUtils import PIDController, generate_rosmsg_PID_status

parser = argparse.ArgumentParser(description='get input of ship ID')
parser.add_argument('vessel_id', type=str, nargs=1,help='Vessel Identifier of the specified ship')
args, unknown = parser.parse_known_args()
PERIOD_BROADCAST_STATUS = 5.0 # seconds
MIN_PERIOD_CONTROL = 1.0/20.0 # seconds
PERIOD_BROADCAST_CONTROL_STATUS = 1.0/16.0 # Hz

class PIDController_rotationally(PIDController):
	"""
	Subclass of PIDController with variation that calculates the error in a rotational sense. 
	This means that the error is calculated as the shortest angle between the reference and the state.
	"""
	def calc_error(self):
		"""
		Calculate the error in a rotational sense
		Overwrites the (linear) calc_error method of the PIDController class to be rotationally continuous between 0 and 2pi
		"""
		return rtf.signed_shortest_angle_radians(self.ref,self.state)

class HeadingControllerNode():
	def __init__(self,object_id):
		self.tstart = time.time()
		self.name = object_id

		# Look if vessel_id is a field of the rascolors class. Set self.color to the corresponding color or otherwise VESSEL_COLOR_DEFAULT
		self.color = getattr(rascolors, self.name, rascolors.VESSEL_COLOR_DEFAULT)
	
		self.node = rospy.init_node(self.name + '_heading_controller', anonymous=False)
		self.timestamp_broadcast_status = 0
		

		self.pid_state_updated = 0
		
		# create PID controllers
		#self.headingPID = PIDController_rotationally(0.6,0.12,0.0)# OLD, referring to direct azimuth output
		self.headingPID = PIDController_rotationally(0.32,0.0,0.0)# ki = 0.0962
		self.headingPID.output_limits=[-0.63,0.63] # [N*m] NEW, referring to desired torque output
		self.headingPID.integral_limits = [-1.0,1.0]
		
		# State tracking of the formation
		self.subscriber_reference = rospy.Subscriber('/'+self.name+'/reference/yaw', Float32,self.callback_reference)
		self.subscriber_state = rospy.Subscriber('/'+self.name+'/state/yaw',Float32,self.callback_state)
		#self.publisher_actuation = rospy.Publisher('/'+self.name+'/reference/actuation', Float32MultiArray, queue_size=1)
		self.pid_status_publisher = rospy.Publisher('/'+self.name+'/diagnostics/headingController/pid_status', Float32MultiArray, queue_size=1)
		self.publisher_desired_torque = rospy.Publisher('/'+self.name+'/reference/controlEffort/torqueZ', Float32, queue_size=1)
		# Rate tracking of controls
		self.tlast_control = 0

		# Diagnostics
		self.resetTrackers()
		self.timestamp_broadcast_control_status=0

	def resetTrackers(self):
		self.tracker_callback_reference = 0
		self.tracker_callback_state = 0
		self.tracker_num_control_callback = 0
		self.tracker_callback_mainloop = 0
				
	def callback_reference(self,msg):
		self.tracker_callback_reference += 1
		self.headingPID.setRef(msg.data)
		self.pid_state_updated = 1
		self.run_controls()

	def callback_state(self,msg):
		self.tracker_callback_state += 1
		self.headingPID.setState(msg.data)
		self.pid_state_updated = 1
		self.run_controls()
	
	def run_controls(self):
		# Run PID controller
		desired_torque = self.headingPID.compute() 

		# Publish actuation
		msg = Float32(data=desired_torque)
		self.publisher_desired_torque.publish(msg)


	def run(self):
		rate = rospy.Rate(1000)  # Hz
		while not rospy.is_shutdown():
			now = time.time()
			self.tracker_callback_mainloop += 1

			if now-self.tlast_control>MIN_PERIOD_CONTROL and self.pid_state_updated:
				self.tlast_control = now
				self.tracker_num_control_callback += 1
				self.run_controls()

			if now - self.timestamp_broadcast_status > PERIOD_BROADCAST_STATUS:
				self.timestamp_broadcast_status = now

				# Determine system frequencies rounded to two decimals
				freq_callback_reference = round(self.tracker_callback_reference / PERIOD_BROADCAST_STATUS, 2)
				freq_callback_state = round(self.tracker_callback_state / PERIOD_BROADCAST_STATUS, 2)
				freq_callback_run_controls = round(self.tracker_num_control_callback / PERIOD_BROADCAST_STATUS, 2)
				freq_callback_mainloop = round(self.tracker_callback_mainloop / PERIOD_BROADCAST_STATUS, 2)
				
				# Make strings of numbers without color if above zero and and rascolors.FAIL otherwise
				freq_main_loop_str = rascolors.OKGREEN +str(freq_callback_mainloop)  + rascolors.NORMAL if freq_callback_mainloop > 0 else rascolors.FAIL + str(freq_callback_mainloop) + rascolors.NORMAL
				freq_callback_reference_str = rascolors.OKGREEN +str(freq_callback_reference)  + rascolors.NORMAL if freq_callback_reference > 0 else rascolors.FAIL + str(freq_callback_reference) + rascolors.NORMAL
				freq_callback_state_str = rascolors.OKGREEN +str(freq_callback_state)  + rascolors.NORMAL if freq_callback_state > 0 else rascolors.FAIL + str(freq_callback_state) + rascolors.NORMAL
				freq_callback_run_controls_str = rascolors.OKGREEN +str(freq_callback_run_controls)  + rascolors.NORMAL if freq_callback_run_controls > 0 else rascolors.FAIL + str(freq_callback_run_controls) + rascolors.NORMAL


				# Print system state
				print(' '+self.color + self.name + rascolors.NORMAL +' [heading controller]['+str(round(time.time()-self.tstart,2)) + 's] freq_callback_reference: ' + freq_callback_reference_str + 'Hz, freq_callback_state: ' + freq_callback_state_str + 'Hz, freq_callback_run_controls: ' + freq_callback_run_controls_str + 'Hz, freq_callback_mainloop: ' + freq_main_loop_str + 'Hz')


				self.resetTrackers()
			
			if now - self.timestamp_broadcast_control_status > PERIOD_BROADCAST_CONTROL_STATUS:
				self.timestamp_broadcast_control_status = now

				# Publish PID status
				self.pid_status_publisher.publish(generate_rosmsg_PID_status(self.headingPID))
			rate.sleep()

if __name__ == '__main__':
	vesselcontroller = HeadingControllerNode(args.vessel_id[0])
	vesselcontroller.run()