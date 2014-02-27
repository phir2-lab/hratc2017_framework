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
#include <std_msgs/Bool.h>
#include <control_msgs/JointControllerState.h>

ros::Publisher * state_pub_ptr;

ros::Publisher * sweep_pub_ptr;
ros::Publisher * upper_pub_ptr;
ros::Publisher * lower_pub_ptr;
ros::Publisher * mirror_pub_ptr;

double state;
double goal;
bool sweep;

double min;
double max;

void sweepStateCallback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    state = msg->process_value;
}

void stateCallback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    control_msgs::JointControllerState joint_state;

    joint_state.header = msg->header;
    joint_state.set_point = msg->set_point;
    joint_state.process_value = msg->process_value;
    joint_state.process_value_dot = msg->process_value_dot;
    joint_state.error = msg->error;
    joint_state.time_step = msg->time_step;
    joint_state.command = msg->command;
    joint_state.p = msg->p;
    joint_state.i = msg->i;
    joint_state.d = msg->d;
    joint_state.i_clamp = msg->i_clamp;

    state_pub_ptr->publish(joint_state);

    state = msg->process_value;
}

void commandCallback(const std_msgs::Float64::ConstPtr& msg)
{
    std_msgs::Float64 command;
    
    command.data = msg->data; 
    upper_pub_ptr->publish(command);
    lower_pub_ptr->publish(command);

    command.data = -1*msg->data;
    mirror_pub_ptr->publish(command);
}

void sweepCallback(const std_msgs::Bool::ConstPtr& msg)
{
    sweep = msg->data;

    std_msgs::Float64 command;
    command.data = goal = min;
    sweep_pub_ptr->publish(command);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle n;

    double min = -0.8;
    double max = 0.8;

    ros::Subscriber sweep_state_sub = n.subscribe("/arm/sweep_controller/state", 1, sweepStateCallback);

    ros::Subscriber state_sub = n.subscribe("/arm/upper_lift_controller/state", 1, stateCallback);
    ros::Publisher state_pub = n.advertise<control_msgs::JointControllerState>("/arm/lift_controller/state", 1);

    state_pub_ptr = &state_pub;

    ros::Subscriber command_sub = n.subscribe("/arm/lift_controller/command", 1, commandCallback);
    
    ros::Publisher sweep_pub = n.advertise<std_msgs::Float64>("/arm/sweep_controller/command", 1);
    ros::Publisher upper_pub = n.advertise<std_msgs::Float64>("/arm/upper_lift_controller/command", 1);
    ros::Publisher lower_pub = n.advertise<std_msgs::Float64>("/arm/lower_lift_controller/command", 1);
    ros::Publisher mirror_pub = n.advertise<std_msgs::Float64>("/arm/mirror_lift_controller/command", 1);
    
    sweep_pub_ptr = &sweep_pub;
    upper_pub_ptr = &upper_pub;
    lower_pub_ptr = &lower_pub;
    mirror_pub_ptr = &mirror_pub;

    sweep = false;
    ros::Subscriber sweep_sub = n.subscribe("/arm/sweep", 1, sweepCallback);

    ros::Duration(3.0).sleep();

    std_msgs::Float64 msg;
    msg.data = 0.0;

    upper_pub.publish(msg);
    lower_pub.publish(msg);
    mirror_pub.publish(msg);

    ros::Rate r(50.0);
    while(ros::ok())
    {
        //ROS_INFO("state %lf goal %lf", state, goal);
        if(sweep && fabs(state - goal) <= 0.01)
        {
            std_msgs::Float64 command;
            command.data = goal = (goal == min) ? max : min;
            sweep_pub.publish(command);
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
