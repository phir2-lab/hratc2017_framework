#!/usr/bin/python
# -*- coding:utf8 -*-
import rospy, os, sys, curses, time, cv2, tf
import numpy as np
from numpy import deg2rad
from curses import wrapper
from threading import Thread
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# read/write stuff on screen
std = None  

# Robot data
robotTwist = Twist()
robotOdom = Odometry()
radStep = deg2rad(15)
linStep = 0.1

# Position callback
def receiveOdom(actualOdometry):
    global robotOdom 
    robotOdom = actualOdometry
    showStats()

# Printing stuff on screen
def showStats():
    if std == None:
        return
    std.clear()
    std.addstr(0,0,"Press Esc to Quit...")
    std.addstr(1,0,"Linear:")
    std.addstr(2, 0, "{} \t {} \t {}".format(robotTwist.linear.x,robotTwist.linear.y,robotTwist.linear.z))
    std.addstr(4,0,"Angular:")
    std.addstr(5, 0, "{} \t {} \t {}".format(robotTwist.angular.x,robotTwist.angular.y,robotTwist.angular.z))
    std.addstr(7,0,"Actual Position:")
    std.addstr(8, 0, "{} \t {} \t {}".format(robotOdom.pose.pose.position.x,robotOdom.pose.pose.position.y,robotOdom.pose.pose.position.z))

    std.refresh()

def publishPTU(PTUpanValue, PTUtiltValue, pubPTU):
    msg = JointTrajectory()
    msg.header.stamp = rospy.get_rostime()
    msg.points.append(JointTrajectoryPoint())
    msg.joint_names.append("ptu_d46_pan_joint")
    msg.points[0].positions.append(PTUpanValue)
    msg.points[0].velocities.append(0.8)
    msg.points[0].accelerations.append(0.8)
    msg.joint_names.append("ptu_d46_tilt_joint")
    msg.points[0].positions.append(PTUtiltValue)
    msg.points[0].velocities.append(0.8)
    msg.points[0].accelerations.append(0.8)
    msg.points[0].time_from_start = rospy.Duration.from_sec(0.5)
    pubPTU.publish(msg)

# Basic control
def KeyCheck(stdscr):
    stdscr.keypad(True)
    stdscr.nodelay(True)

    k = None
    global std
    std = stdscr

    #publishing topics
    pubVel   = rospy.Publisher('/sim_p3at/cmd_vel', Twist)

    # Pan & Tilt Unit - both pan and tilt accept values from -0.5 to 0.5 rad
    pubPTU = rospy.Publisher('/ptu_d46_controller/command', JointTrajectory)
    PTUpanValue = 0.0
    PTUtiltValue= 0.0

    # While 'Esc' is not pressed
    while k != chr(27):
        # Check no key
        try:
            k = stdscr.getkey()
        except:
            k = None

        # PTU movement
        if k == "h":
            PTUpanValue+= 0.1
            if PTUpanValue > 0.5:
                PTUpanValue = 0.5
            publishPTU(PTUpanValue, PTUtiltValue, pubPTU) 
        if k == "j":
            PTUpanValue -= 0.1
            if PTUpanValue < -0.5:
                PTUpanValue = -0.5
            publishPTU(PTUpanValue, PTUtiltValue, pubPTU)
        if k == "k":
            PTUtiltValue+= 0.1
            if PTUtiltValue > 0.5:
                PTUtiltValue = 0.5
            publishPTU(PTUpanValue, PTUtiltValue, pubPTU)
        if k == "l":
            PTUtiltValue-= 0.1
            if PTUtiltValue < -0.5:
                PTUtiltValue = -0.5
            publishPTU(PTUpanValue, PTUtiltValue, pubPTU)

        # Robot movement
        if k == " ":
            robotTwist.linear.x  = 0.0           
            robotTwist.angular.z = 0.0
        if k == "KEY_LEFT":
            robotTwist.angular.z += radStep
        if k == "KEY_RIGHT":
            robotTwist.angular.z -= radStep
        if k == "KEY_UP":
            robotTwist.linear.x +=  linStep
        if k == "KEY_DOWN":
            robotTwist.linear.x -= linStep

        robotTwist.angular.z = min(robotTwist.angular.z,deg2rad(90))
        robotTwist.angular.z = max(robotTwist.angular.z,deg2rad(-90))
        robotTwist.linear.x = min(robotTwist.linear.x,1.0)
        robotTwist.linear.x = max(robotTwist.linear.x,-1.0)
        pubVel.publish(robotTwist)

        showStats()
        time.sleep(0.1)

    stdscr.keypad(False)
    rospy.signal_shutdown("Shutdown Competitor")
    
# Initialize curses stuff
def StartControl():
    wrapper(KeyCheck)

# ROSPy stuff
def spin():
    rospy.spin()

if __name__ == '__main__':
    # Initialize client node
    rospy.init_node('client')

    # Subscribing to all these topics to bring the robot or simulation to live data
    rospy.Subscriber("/sim_p3at/odom", Odometry, receiveOdom)

    #Starting curses and ROS
    Thread(target = StartControl).start()
    Thread(target = spin).start()

