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
* Author: Goncalo Cabrita on 21/02/2014
*********************************************************************/

#include <ros/ros.h>
#include <std_msgs/Float64.h>

ros::Publisher * upper_pub_ptr;
ros::Publisher * lower_pub_ptr;
ros::Publisher * mirror_pub_ptr;

void commandCallback(const std_msgs::Float64::ConstPtr& msg)
{
    std_msgs::Float64 command;
    
    command.data = msg->data; 
    upper_pub_ptr->publish(command);
    lower_pub_ptr->publish(command);

    command.data = -1*msg->data;
    mirror_pub_ptr->publish(command);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle n;

    ros::Subscriber command_sub = n.subscribe("/arm/lift_controller/command", 1, commandCallback);
    
    ros::Publisher upper_pub = n.advertise<std_msgs::Float64>("/arm/upper_lift_controller/command", 1);
    ros::Publisher lower_pub = n.advertise<std_msgs::Float64>("/arm/lower_lift_controller/command", 1);
    ros::Publisher mirror_pub = n.advertise<std_msgs::Float64>("/arm/mirror_lift_controller/command", 1);
    
    upper_pub_ptr = &upper_pub;
    lower_pub_ptr = &lower_pub;
    mirror_pub_ptr = &mirror_pub;

    ros::Delay(3.0).sleep();

    std_msgs::Float64 msg;
    msg.data = 0.0;

    upper_pub.publish(msg);
    lower_pub.publish(msg);
    mirror_pub.publish(msg);
    
    ros::spin();

    return 0;
}
