#!/usr/bin/env python

import tf
import rospy

def node():
        # Create node
        rospy.init_node('pose_corr_publisher', anonymous=True)
        # Define rate
        rate = rospy.Rate(10) # 10 hz

        # Create transform broadcaster
        br = tf.TransformBroadcaster()

        roll, pitch, yaw = 0, 0, 3.14/2.0

        # While node is active
        while not rospy.is_shutdown():
                br.sendTransform( (0,0,0),  tf.transformations.quaternion_from_euler(roll, pitch, yaw), rospy.Time.now(), 'odom', 'minefield' )
                rate.sleep()

#Main
if __name__ == '__main__':
        node()