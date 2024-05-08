#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Float32MultiArray
import argparse
import time
import ras_tf_lib.ras_tf_lib1  as rtf
from ras_tf_lib.ras_tf_lib1 import rascolors as rascolors

from ras_tf_lib.controlUtils import PIDController, generate_rosmsg_PID_status
parser = argparse.ArgumentParser(description='get input of ship ID')
parser.add_argument('vessel_id', type=str, nargs=1,help='Vessel Identifier of the specified ship')
args, unknown = parser.parse_known_args()
MIN_PERIOD_CONTROL = 1.0/16.0 # maximum PID update rate is 20Hz
REPORT_PERIOD = 5
PERIOD_BROADCAST_CONTROL_STATUS = 1.0/16.0 # Hz


class SurgeVelController():
	def __init__(self,vessel_id_:str):
		# Old kpid values: Kp_:float=14000.0,Ki_:float=1700.0,Kd_:float=0.0
		self.name = vessel_id_
		
		# Look if vessel_id is a field of the rascolors class. Set self.color to the corresponding color or otherwise VESSEL_COLOR_DEFAULT
		self.color = getattr(rascolors, self.name, rascolors.VESSEL_COLOR_DEFAULT)

		self.PID = PIDController(21.0,0.0,0.0) # kp was 21.0 kd was 2.55
		self.PID.integral_limits=[-8*0.25, 8*0.25] # set limit to a total buildup of 8 seconds with a characteristic error of 0.25m/s/s
		self.PID.output_limits=[0.1,3.0] # Newtons of thrust from both aft thrusters combined

		self.tstart = time.time()
		self.treport = time.time()

		# State if the reference or state of the controller has been updated
		self.pid_state_updated = 0

		# Rate tracking of controls
		self.tlast_control = 0

		# Diagnostics
		self.tracker_num_ref_callback = 0
		self.tracker_num_vel_callback = 0
		self.tracker_num_control_callback = 0
		self.tracker_num_main_loop = 0
		self.timestamp_broadcast_control_status=0

		# ROS components
		self.node = rospy.init_node('surge_controller_node_'+self.name, anonymous=True)
		self.publisher_forceX = rospy.Publisher('/'+self.name+'/reference/controlEffort/forceX', Float32, queue_size=1)
		self.pid_status_publisher = rospy.Publisher('/'+self.name+'/diagnostics/surgeVelocityController/pid_status', Float32MultiArray, queue_size=1)
		self.velocity_subscriber = rospy.Subscriber('/'+self.name+'/state/velocity', Float32MultiArray, self.velocity_callback)
		self.reference_subscriber = rospy.Subscriber('/'+self.name+'/reference/velocity', Float32MultiArray, self.reference_callback)

	def reference_callback(self, msg:Float32MultiArray):
		self.tracker_num_ref_callback += 1
		self.PID.setRef(msg.data[0])
		self.pid_state_updated = 1

	def velocity_callback(self, msg:Float32MultiArray):
		self.tracker_num_vel_callback += 1
		self.PID.setState(msg.data[0])
		self.pid_state_updated = 1
	
	def run_controls(self):
		# Calculate PID output
		out = self.PID.compute()

		# Send message
		msg = Float32(data=out)
		self.publisher_forceX.publish(msg)

	def run(self):
		rate = rospy.Rate(1000)  # Hz
		while not rospy.is_shutdown():
			now = time.time()
			self.tracker_num_main_loop += 1

			if now-self.tlast_control>MIN_PERIOD_CONTROL and self.pid_state_updated:
				self.tlast_control = now
				self.tracker_num_control_callback += 1
				self.run_controls()

			if now - self.timestamp_broadcast_control_status > PERIOD_BROADCAST_CONTROL_STATUS:
				self.timestamp_broadcast_control_status = now

				# Publish PID status
				self.pid_status_publisher.publish(generate_rosmsg_PID_status(self.PID))

			if now-self.treport >= REPORT_PERIOD:
				# Determine system frequencies rounded to two decimals
				freq_control_loop = round(self.tracker_num_control_callback/(now-self.tstart),2)
				freq_vel_callback = round(self.tracker_num_vel_callback/(now-self.tstart),2)
				freq_ref_callback = round(self.tracker_num_ref_callback/(now-self.tstart),2)
				freq_main_loop = round(self.tracker_num_main_loop/(now-self.tstart),2)

				# Make strings of numbers without color if above zero and and rascolors.FAIL otherwise
				freq_main_loop_str = rascolors.OKGREEN +str(freq_main_loop) + rascolors.NORMAL if freq_main_loop>0 else rascolors.FAIL+str(freq_main_loop) + rascolors.NORMAL
				freq_control_loop_str = rascolors.OKGREEN +str(freq_control_loop) + rascolors.NORMAL if freq_control_loop>0 else rascolors.FAIL+str(freq_control_loop) + rascolors.NORMAL
				freq_vel_callback_str = rascolors.OKGREEN +str(freq_vel_callback) + rascolors.NORMAL if freq_vel_callback>0 else rascolors.FAIL+str(freq_vel_callback) + rascolors.NORMAL
				freq_ref_callback_str = rascolors.OKGREEN +str(freq_ref_callback) + rascolors.NORMAL if freq_ref_callback>0 else rascolors.FAIL+str(freq_ref_callback) + rascolors.NORMAL

				# Print system state
				print(' '+self.color + self.name + rascolors.NORMAL +' [surge velocity controller]['+str(round(time.time()-self.tstart,2)) + 's] freq_control_loop: ' + freq_control_loop_str + 'Hz, freq_vel_callback: ' + freq_vel_callback_str + 'Hz, freq_ref_callback: ' + freq_ref_callback_str + 'Hz, freq_main_loop: ' + freq_main_loop_str + 'Hz')

			
				self.treport = now
			rate.sleep()

if __name__ == '__main__':
	pid_controller_node = SurgeVelController(args.vessel_id[0])
	pid_controller_node.run()