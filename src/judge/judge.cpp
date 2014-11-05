#include "judge.h"

#include <iostream>
#include <string>

using namespace std;

Judge::Judge()
{
    n = new ros::NodeHandle("~");

    string filename;
    if(n->getParam("config", filename)==false)
    {
        ROS_ERROR("Failed to get param 'config'");
    }

    robotPose = new RobotPose("/minefield","/robot_pose_ekf/odom");

    config = new Config(filename);

//    // Initialize publishers
//    initializeMarker();
    initializeTrueMinesMarkers();
    initializeRobotPath();

    rate = new ros::Rate(5);
}

void Judge::run()
{
//    ros::Rate r(5); // try to keep the loop in a 10 Hz frequency

    int count = 0;
    while (ros::ok())
    {
        geometry_msgs::PoseStamped p = robotPose->GetLocalPose();
        robotZ = p.pose.position.z;
//        cout << "Pose:" << p.pose.position.x << ' ' << p.pose.position.y << ' ' << tf::getYaw(p.pose.orientation) << endl;

//        updateMarker();
        updateTrueMinesMarkers();
        updateRobotPath(count++);

        ros::spinOnce();
        rate->sleep();
//        r.sleep();
    }
}

void Judge::initializeMarker()
{
    pub_marker = n->advertise<visualization_msgs::Marker>("visualization_marker", 1);
    // Set our initial shape type to be a cube
    shape = visualization_msgs::Marker::CUBE;
}

void Judge::initializeTrueMinesMarkers()
{
    pub_trueMinesMarker = n->advertise<visualization_msgs::MarkerArray>("trueMines_marker", 1);

    trueMinesMarkers.markers.resize(config->numMines);

    cout << trueMinesMarkers.markers.size() << endl;

    for(int i=0; i<trueMinesMarkers.markers.size(); i++){
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        trueMinesMarkers.markers[i].header.frame_id = "minefield";
        trueMinesMarkers.markers[i].header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        trueMinesMarkers.markers[i].ns = "trueMines";
        trueMinesMarkers.markers[i].id = i;

        // Set the marker type.
        trueMinesMarkers.markers[i].type = visualization_msgs::Marker::CYLINDER;

        // Set the marker action.  Options are ADD and DELETE
        trueMinesMarkers.markers[i].action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        trueMinesMarkers.markers[i].pose.position.x = config->minesPositions[i].x;
        trueMinesMarkers.markers[i].pose.position.y = config->minesPositions[i].y;
        trueMinesMarkers.markers[i].pose.position.z = 0;
        trueMinesMarkers.markers[i].pose.orientation.x = 0.0;
        trueMinesMarkers.markers[i].pose.orientation.y = 0.0;
        trueMinesMarkers.markers[i].pose.orientation.z = 0.0;
        trueMinesMarkers.markers[i].pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        trueMinesMarkers.markers[i].scale.x = 0.2;
        trueMinesMarkers.markers[i].scale.y = 0.2;
        trueMinesMarkers.markers[i].scale.z = 0.01;

        // Set the color -- BLUE!
        trueMinesMarkers.markers[i].color.r = 0.0f;
        trueMinesMarkers.markers[i].color.g = 0.4f;
        trueMinesMarkers.markers[i].color.b = 1.0f;
        trueMinesMarkers.markers[i].color.a = 1.0;

        trueMinesMarkers.markers[i].lifetime = ros::Duration();
    }
}

void Judge::initializeRobotPath()
{
    pub_robotPath = n->advertise<visualization_msgs::Marker>("hratc_path",10);
    robotpath.header.frame_id = "minefield";
    robotpath.header.stamp = ros::Time::now();
    robotpath.ns =  "robotPath";
    robotpath.action = visualization_msgs::Marker::ADD;
    robotpath.pose.orientation.w  = 1.0;
    robotpath.type = visualization_msgs::Marker::LINE_STRIP;
    robotpath.scale.x = 0.1;
    robotpath.color.r = 0;
    robotpath.color.g = 0.0;
    robotpath.color.b = 0.5;
    robotpath.color.a = 1.0;
}

void Judge::updateMarker()
{
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/minefield";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Cycle between different shapes
    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
        shape = visualization_msgs::Marker::SPHERE;
        marker.color.a = 0.5;
        break;
    case visualization_msgs::Marker::SPHERE:
        shape = visualization_msgs::Marker::ARROW;
        break;
    case visualization_msgs::Marker::ARROW:
        shape = visualization_msgs::Marker::CYLINDER;
        break;
    case visualization_msgs::Marker::CYLINDER:
        shape = visualization_msgs::Marker::CUBE;
        break;
    }

    ROS_INFO("AAAA");

    // Publish the marker
    pub_marker.publish(marker);
}

void Judge::updateTrueMinesMarkers()
{
    cout << "Publishing true mines" << endl;
    // Publish the marker
    for(int i=0; i<trueMinesMarkers.markers.size(); i++){
        trueMinesMarkers.markers[i].header.stamp = ros::Time::now();
        trueMinesMarkers.markers[i].pose.position.z = robotZ-0.1;
//        cout << trueMinesMarkers.markers[i].ns << ' ' << trueMinesMarkers.markers[i].id << endl;
//        rate->sleep();
    }

    pub_trueMinesMarker.publish(trueMinesMarkers);

}

void Judge::updateRobotPath(int i)
{
    geometry_msgs::Point p = robotPose->GetLocalPose().pose.position;
//    p.x = i/10;
//    cout << p.x << endl;
//    p.y = 0;
//    p.z = 0;
    robotpath.points.push_back(p);
    pub_robotPath.publish(robotpath);
}
