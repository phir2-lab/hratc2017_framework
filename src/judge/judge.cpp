#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "robotPose.h"



ros::NodeHandle* n;
RobotPose* robotPose;

ros::Publisher pub_marker;
uint32_t shape;

ros::Publisher pub_robotPath;
visualization_msgs::Marker robotpath;

void initializeMarker()
{
    pub_marker = n->advertise<visualization_msgs::Marker>("visualization_marker", 1);
    // Set our initial shape type to be a cube
    shape = visualization_msgs::Marker::CUBE;
}

void initializeRobotPath()
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

void updateMarker()
{
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
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

void updateRobotPath(int i)
{
    geometry_msgs::Point p = robotPose->GetLocalPose().pose.position;
//    p.x = i/10;
//    cout << p.x << endl;
//    p.y = 0;
//    p.z = 0;
    robotpath.points.push_back(p);
    pub_robotPath.publish(robotpath);
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "hratc_viewer");
    ROS_INFO("HRATC");

    n = new ros::NodeHandle();

    robotPose = new RobotPose("/minefield","/robot_pose_ekf/odom");
    geometry_msgs::PoseStamped p;

//    // Initialize publishers
//    initializeMarker();
    initializeRobotPath();

//    // Initialize subscribers
////    ros::Subscriber sub_odom = n.subscribe("odom", 50, fixCallback);
////    ros::Subscriber sub_robotPoseEKF = n.subscribe("/robot_pose_ekf/odom", 50, receiveRobotPoseEKF);

    ros::Rate r(10); // try to keep the loop in a 10 Hz frequency

    int count = 0;
    while (ros::ok())
    {
//        updateMarker();
        updateRobotPath(count++);
        p = robotPose->GetGlobalPose();
//        cout << "Pose:" << p.pose.position.x << ' ' << p.pose.position.y << ' ' << tf::getYaw(p.pose.orientation) << endl;

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
