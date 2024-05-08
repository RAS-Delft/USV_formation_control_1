#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

def publish_vel_ref():
    # Initialize ROS node
    rospy.init_node('publish_vel_ref', anonymous=True)

    # Create publisher on topic RAS_TN_RE/vel_ref of type Float32MultiArray
    pub = rospy.Publisher('RAS_TN_DB/reference/velocity', Float32MultiArray, queue_size=10)

    # Set publishing rate to 10Hz
    rate = rospy.Rate(10)

    # Define two different velocity reference messages
    vel_ref_1 = Float32MultiArray()
    vel_ref_1.data = [0.4, 0, 0]
    vel_ref_2 = Float32MultiArray()
    vel_ref_2.data = [0.6, 0, 0]
    vel_ref_3 = Float32MultiArray()
    vel_ref_3.data = [0.3, 0, 0]
    vel_ref_4 = Float32MultiArray()
    vel_ref_4.data = [0.6, 0, 0]

    # Alternate between vel_ref_1 and vel_ref_2 every 20 seconds
    t_start = rospy.get_time()
    while not rospy.is_shutdown():
        t_elapsed = rospy.get_time() - t_start
        if t_elapsed < 20:
            vel_ref_msg = vel_ref_1
        elif t_elapsed < 40:
            vel_ref_msg = vel_ref_2
        elif t_elapsed < 60:
             vel_ref_msg = vel_ref_3
        else:
            vel_ref_msg = vel_ref_4
            if t_elapsed >= 80:
                t_start = rospy.get_time()

        # Publish message
        pub.publish(vel_ref_msg)

        # Sleep to maintain 10Hz publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_vel_ref()
    except rospy.ROSInterruptException:
        pass
