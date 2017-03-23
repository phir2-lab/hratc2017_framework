/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, ISR University of Coimbra.
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
* Updated: Baptiste Gil on 16/04/2015
*
* This software publishes a static transform between the world frame
* and the minefield frame.
*
* The minefield frame is centered on the 4 GPS coordinates that define
* the minefield itself.
*
*   1 ------------------------------------------------------------ 4
*   |                                                              |
*   |                              y                               |
*   |                              ^                               |
*   |                              |                               |
*   |                              o-- > x                         |
*   |                            z                                 |
*   |                                                              |
*   |                                                              |
*   |                                                              |
*   2 ------------------------------------------------------------ 3
*
*********************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include <visualization_msgs/MarkerArray.h>

#include "../UTMConverter/UTMConverter.h"

#include <vector>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "minefield_static_tf_publisher");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    string s, s1, s2, s3;
    stringstream ss;

    vector<tf::Vector3> minefieldCorners;

    // Get minefield corners
    int count=1;
    while(true){
        ss.str("");
        ss << count;
        s = "minefield/corner"+ss.str();

        if(!pn.hasParam(s))
            break;

        // Convert the GPS coordinates into UTM coordinates
        UTMCoordinates utm;
        sensor_msgs::NavSatFix fix;

        s1 = s+"/latitude";
        s2 = s+"/longitude";
        s3 = s+"/altitude";

        if(!pn.hasParam(s1) && !pn.hasParam(s2) && !pn.hasParam(s3))
        {
            ROS_FATAL("Config -- Unable to start without the 4 corners of the minefield!!!");
            ROS_BREAK();
        }
        pn.getParam(s1, fix.latitude);
        pn.getParam(s2, fix.longitude);
        pn.getParam(s3, fix.altitude);

        //UTMConverter::latitudeAndLongitudeToUTMCoordinates(fix, utm);
        minefieldCorners.push_back(tf::Vector3(fix.latitude, fix.longitude, fix.altitude));

    	ROS_INFO("Minefield static tf broadcaster -- Corner%d lat:%lf long:%lf alt:%lf - x:%lf y:%lf z:%lf", count, fix.latitude, fix.longitude, fix.altitude, minefieldCorners.back().x(), minefieldCorners.back().y(), minefieldCorners.back().z());

        count++;
    }


    // Find the center of the minefield, this will be the origin of the minefield frame
    tf::Vector3 center(0,0,0);
    for(int i=0; i<minefieldCorners.size(); i++){
	center += minefieldCorners[i];
    }
    center /= minefieldCorners.size();

    ROS_INFO("Minefield static tf broadcaster -- The center of the minefield is x:%lf y:%lf z:%lf (m)", center.x(), center.y(), center.z());

    double rate;
    pn.param("rate", rate, 20.0);

    std::string parent_frame;
    pn.param<std::string>("parent_frame", parent_frame, "map");
    
    std::string child_frame;
    pn.param<std::string>("child_frame", child_frame, "minefield");

    ROS_INFO("Minefield static tf broadcaster -- Publishing static transform from %s to %s at %lf hz...", parent_frame.c_str(), child_frame.c_str(), rate);

    // Generate the transformation from the world frame to the minefield frame
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(center.x(), center.y(), center.z()) );
    tf::Quaternion q;
//    q.setRPY(0.0, 0.0, atan2((corner4.y()+corner3.y())/2.0 - center.y(), (corner4.x()+corner3.x())/2.0 - center.x()));
    //q.setRPY(0.0, 0.0, atan2((minefieldCorners[2].x()-minefieldCorners[1].x()),(minefieldCorners[2].y()-minefieldCorners[1].y())));
    q.setRPY(0.0, 0.0, 0.0);
    transform.setRotation(q);

    //Publish the corners
    ros::Publisher corner_pub = n.advertise<visualization_msgs::MarkerArray>("corners", 10, true);

    visualization_msgs::MarkerArray marker_corner_array;
    int num_corners=minefieldCorners.size();
    //Resize corners array
    marker_corner_array.markers.resize(num_corners);

    for (int count=0; count<num_corners; count++){

        //Corners defintion
        marker_corner_array.markers.at(count).header.frame_id=parent_frame;
        marker_corner_array.markers.at(count).id = count;
        marker_corner_array.markers.at(count).header.stamp = ros::Time::now();
        marker_corner_array.markers.at(count).ns = "minefield_corners";
        marker_corner_array.markers.at(count).action = visualization_msgs::Marker::ADD;
        marker_corner_array.markers.at(count).pose.orientation.w = 1.0;
        marker_corner_array.markers.at(count).type = visualization_msgs::Marker::CYLINDER;
        // Corners size
        marker_corner_array.markers.at(count).scale.x = 0.1;
        marker_corner_array.markers.at(count).scale.y = 0.1;
        marker_corner_array.markers.at(count).scale.z = 0.1;
        // Corners color
        marker_corner_array.markers.at(count).color.r = 1.0;
        marker_corner_array.markers.at(count).color.g = 1.0;
        marker_corner_array.markers.at(count).color.b = 1.0;
        marker_corner_array.markers.at(count).color.a = 1.0;
	// Corners position
        marker_corner_array.markers.at(count).pose.position.x = minefieldCorners[count].x();
        marker_corner_array.markers.at(count).pose.position.y = minefieldCorners[count].y();
        marker_corner_array.markers.at(count).pose.position.z = center.z();
    }
    
    corner_pub.publish(marker_corner_array);

    // Publish the transformation at the desired rate!
    ros::Rate loop(rate);
    while(ros::ok())
    {
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame, child_frame));
        ros::spinOnce();

        loop.sleep();
    }
    return 0;
}

