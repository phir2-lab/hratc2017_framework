/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, ISR University of Coimbra.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the ISR University of Coimbra nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Goncalo Cabrita on 21/04/2014
 *********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>

#include "../UTMConverter/UTMConverter.h"

ros::Publisher * pub_ptr;

void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    sensor_msgs::NavSatFix fix = *msg;
    
    UTMCoordinates utm;
    UTMConverter::latitudeAndLongitudeToUTMCoordinates(fix, utm);
    
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "gps_antenna";
    
    odom_msg.pose.pose.position.x = utm.easting;
    odom_msg.pose.pose.position.y = utm.northing;
    odom_msg.pose.pose.position.z = msg->altitude;
    odom_msg.pose.pose.orientation.w = 0;
    odom_msg.pose.pose.orientation.x = 1;
    odom_msg.pose.pose.orientation.y = 0;
    odom_msg.pose.pose.orientation.z = 0;
    
    double cov[] = {0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 500, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 99999, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 99999, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 99999};
    for(int i=0 ; i<36 ; i++) odom_msg.pose.covariance[i] = cov[i];
    
    pub_ptr->publish(odom_msg);
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "gps_to_odom");
    
    ROS_INFO("GPS to Odom");
    
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    
    ros::Publisher pub = n.advertise<nav_msgs::Odometry>("gps/odom", 50);
    pub_ptr = &pub;
    
    ros::Subscriber sub = n.subscribe("gps/fix", 50, fixCallback);
    
    ros::spin();
    
    return 0;
}
