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

#ifndef TRUE_ROBOT_POSE_H
#define TRUE_ROBOT_POSE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>

#include <gazebo_msgs/ModelStates.h>

using namespace std;

class TrueRobotPose
{
public:
    TrueRobotPose(ros::NodeHandle *nh);

    const geometry_msgs::PoseStamped & getLocalPose();
    const vector<geometry_msgs::PoseStamped> & getWheelsPoses();
    const geometry_msgs::PoseStamped & getLeftCoilPose();
    const geometry_msgs::PoseStamped & getRightCoilPose();

private:
    ros::NodeHandle* n;
    tf::TransformListener* listener;

    bool isSimulation;

    geometry_msgs::PoseStamped globalPose_;
    geometry_msgs::PoseStamped localPose_;
    vector<geometry_msgs::PoseStamped> wheelsPoses;
    geometry_msgs::PoseStamped leftCoilPose_;
    geometry_msgs::PoseStamped rightCoilPose_;

    vector<geometry_msgs::PoseStamped> emptyVector;
    geometry_msgs::PoseStamped emptyPose_;

    ros::Subscriber sub_gazebo_;

    void getGazeboModelStates(const gazebo_msgs::ModelStates::ConstPtr &ms);
    geometry_msgs::Pose gazeboPose_;

    void getSimulatedMinefieldCenter();
    geometry_msgs::Pose minefieldCenter_;
    float minefieldWidth, minefieldHeight;
};

#endif /* TRUE_ROBOT_POSE_H */
