#!/usr/bin/env python3

"""
Generates a vessels reference speed and heading by steering towards the reference state and controlling a fixed distance towards the target. 
This is used in the context of formation control where vessels are metaphorically "dragged along by a virtual rope" attached to the reference.
Control of absolute distance between ship and reference is PID based.

Researchlab Autonomous Shipping Delft - (info: bartboogmans@hotmail.com)

"""

import rclpy
import argparse
import time
import math
from sensor_msgs.msg import NavSatFix
import pyproj
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from ras_ros_core_control_modules.tools.control_tools import PIDController, generate_rosmsg_PID_status
import ras_ros_core_control_modules.tools.display_tools as ras_display_tools
import ras_ros_core_control_modules.tools.geometry_tools as ras_geometry_tools

parser = argparse.ArgumentParser(description='From given position and reference position, calculate reference heading and velocity to attempt to maintain a fixed distance from moving reference')
parser.add_argument('name', type=str ,help='Identifier of the vessel')
parser.add_argument('distance', type=float,help='distance between ship and ref')
parser.add_argument("-r") # ROS2 arguments
args, unknown = parser.parse_known_args()

REF_DISTANCE = args.distance
OBJECT_ID = args.name
PERIOD_BROADCAST_STATUS = 5.0 # seconds
PERIOD_RECALC_REFERENCES = 0.05 # seconds
PERIOD_BROADCAST_CONTROL_STATUS = 1.0/16.0 # Hz
MIN_PERIOD_CONTROL = 1.0/16.0 # maximum PID update rate is 20Hz
geodesic = pyproj.Geod(ellps='WGS84')

class distancePIDController(PIDController):
	def calc_error(self):
		return  self.state - self.ref

class NomotoFollowingDistanceControllerNode(Node):
	def __init__(self):	
		super().__init__('formation1_reference_generator')

		self.tlast_control = 0
		self.pid_state_updated = 0
		self.timestamp_broadcast_status = time.time()
		self.geopos_ref = []
		self.geopos_state = []
		self.velocityPID = distancePIDController(0.25,0.00,0.0,ref_init=REF_DISTANCE) # ki was 0.018
		self.velocityPID.integral_limits=[-66.0,66.0] # based on benchmark error of 2.0m over a period of 8s
		self.velocityPID.output_limits=[0.25,0.5]

		# Communication
		## Inputs
		self.reference_geopos_subscriber = self.create_subscription(NavSatFix,'reference/geopos',self.callback_reference,10)
		self.state_geopos_subscriber = self.create_subscription(NavSatFix,'telemetry/gnss/fix',self.callback_state_geopos,10)

		## Outputs
		self.publisher_ref_heading = self.create_publisher(Float32, 'reference/heading', 10)
		self.publisher_ref_velocity = self.create_publisher(Float32MultiArray, 'reference/velocity', 10)
		self.publisher_distance = self.create_publisher(Float32, 'diagnostics/ref_target_distance', 10)

		# Timer for broadcasting output
		self.timer_control = self.create_timer(MIN_PERIOD_CONTROL, self.run_controls)

		# Diagnostics
		self.timer_statistics_last = self.get_clock().now().nanoseconds/1e9
		self.timer_broadcast_status = self.create_timer(PERIOD_BROADCAST_STATUS, self.print_statistics)
		self.tracker_callback_publish_reference_heading = 0
		self.tracker_callback_publish_reference_velocity = 0
		self.tracker_callback_reference_geopos = 0
		self.tracker_callback_state_geopos = 0
		self.tracker_num_control_callback = 0

		self.last_ref_update = None
		self.last_ref_geopos_update = None
		self.geopos_ref = None
		self.geopos_state = None

	def callback_reference(self,msg:NavSatFix):
		self.tracker_callback_reference_geopos += 1
		self.last_ref_geopos_update = time.time()
		self.geopos_ref = [msg.latitude,msg.longitude]

	def callback_state_geopos(self,msg:NavSatFix):
		self.tracker_callback_state_geopos += 1
		self.last_state_geopos_update = time.time()
		self.geopos_state = [msg.latitude,msg.longitude]
	
	def run_controls(self):
		"""
		Calculate reference velocity and heading for the ship
		"""
		if self.geopos_ref and self.geopos_state:
			self.tracker_num_control_callback += 1
			# Calculate azimuth and distance between ship and reference
			fwd_azimuth_deg,back_azimuth_deg,distance = geodesic.inv(self.geopos_state[1], self.geopos_state[0],self.geopos_ref[1],self.geopos_ref[0])

			# Run PID controller to maintain desired distance between ship and reference
			self.velocityPID.setState(distance)
			ref_vel_surge = self.velocityPID.compute()

			# Broadcast messages
			msg = Float32MultiArray()
			msg.data = [ref_vel_surge,0.0,0.0]
			self.publisher_ref_velocity.publish(msg)
			
			msg = Float32()
			msg.data = math.radians(fwd_azimuth_deg)
			self.publisher_ref_heading.publish(msg)

			msg = Float32()
			msg.data = distance
			self.publisher_distance.publish(msg)

	def print_statistics(self):
		""" On a single line, print the rates of all major callbacks in this script. """
		
		# Calculate passed time
		now = self.get_clock().now().nanoseconds/1e9
		passed_time = now - self.timer_statistics_last

		# Calculate rates
		rate_callback_run_controls = self.tracker_num_control_callback / passed_time
		rate_callback_reference_geopos = self.tracker_callback_reference_geopos / passed_time
		rate_callback_state_geopos = self.tracker_callback_state_geopos / passed_time

		# Format string
		printstring = ras_display_tools.terminal_fleet_module_string(self.get_namespace()[1:],['callback_run_controls',rate_callback_run_controls,'Hz'],['callback_reference_geopos',rate_callback_reference_geopos,'Hz'],['callback_state_geopos',rate_callback_state_geopos,'Hz'])

		# Print
		self.get_logger().info(printstring)

		# Reset trackers
		self.tracker_num_control_callback = 0
		self.tracker_callback_reference_geopos = 0
		self.tracker_callback_state_geopos = 0
		self.timer_statistics_last = now

def main(args=None):
	rclpy.init(args=args)

	node = NomotoFollowingDistanceControllerNode()

	# Start the nodes processing thread
	rclpy.spin(node)

	# at termination of the code (generally with ctrl-c) Destroy the node explicitly
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()