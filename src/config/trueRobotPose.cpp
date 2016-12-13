#include "trueRobotPose.h"

#include <string>
#include <sstream>
#include <cmath>
using namespace std;

TrueRobotPose::TrueRobotPose(ros::NodeHandle *nh):
    n(nh)
{
    // Check if it is simulation, so it must wait the generation of the mines map
    if(n->getParam("isSimulation", isSimulation)==false)
    {
        isSimulation = false;
    }

    listener = new tf::TransformListener;

    // Check if it is simulation, so it must wait the generation of the mines map
    if(isSimulation){
        sub_gazebo_ = n->subscribe("/gazebo/model_states", 100, &TrueRobotPose::getGazeboModelStates, this);

        getSimulatedMinefieldCenter();
    }

    emptyPose_.header.frame_id = "UNDEF";
    emptyPose_.pose.position.x = 0;
    emptyPose_.pose.position.y = 0;
    emptyPose_.pose.position.z = 0;
    quaternionTFToMsg(tf::createQuaternionFromYaw(0.0*M_PI/180.0),
                          emptyPose_.pose.orientation);

    localPose_ = globalPose_ = emptyPose_;
    wheelsPoses.resize(4);
}

void TrueRobotPose::getSimulatedMinefieldCenter()
{
    // Computa o centro do minefield em coordenadas locais (media dos cantos)

    geometry_msgs::Point center;
    center.x = 0.0;
    center.y = 0.0;

    int num_corners;
    if(!n->hasParam("coordinates/num_corners"))
    {
        ROS_FATAL("ERROR -- coordinates/num_corners!!!");
        ROS_BREAK();
    }
    n->getParam("coordinates/num_corners", num_corners);

    string s;
    stringstream ss;
    cout << "num_corners " << num_corners << endl;
    for(int c=0; c<num_corners; c++)
    {
            ss.str("");
            ss << c+1;
            s = "coordinates/corners_positions/corner"+ss.str();

            if(!n->hasParam(s))
                break;
            float x, y;
            n->getParam(s+"/x", x);
            n->getParam(s+"/y", y);
            cout << "p" << c+1 << ": " << x << ' ' << y << endl;

            center.x += x;
            center.y += y;
    }
    minefieldCenter_.position.y = center.y / (float) num_corners;
    minefieldCenter_.position.x = center.x / (float) num_corners;
    quaternionTFToMsg(tf::createQuaternionFromYaw(90.0*M_PI/180.0),
                          minefieldCenter_.orientation);

    minefieldWidth = minefieldHeight = 10.0;

    //-5 4.5
//    cout << "\n\n\n\n" << minefieldCenter_.position.x << ' ' << minefieldCenter_.position.y << "\n\n\n\n" << endl;
}

//gazebo  | real
//0   0   | 5   4.5
//3.5 0   | 5   1
//4.5 -5  | 0   0
//9.2 -5.1| 0  -4.7
//8.9 -10 |-5  -4.4
//4.5 -10 |-5   0

const geometry_msgs::PoseStamped & TrueRobotPose::getLocalPose()
{
    return localPose_;
}

void TrueRobotPose::getGazeboModelStates(const gazebo_msgs::ModelStates::ConstPtr &ms)
{
    int e = 0;
    for(int i = ms->name.size()-1; i>=0; --i){
        string str = ms->name[i];
        if(str.compare("pioneer3at_robot") == 0){
            e = i;
        }
    }

    // during simulation the robot starts in position

    gazeboPose_.position.x = ms->pose[e].position.x;
    gazeboPose_.position.y = ms->pose[e].position.y;
    gazeboPose_.position.z = ms->pose[e].position.z;
    gazeboPose_.orientation.x = ms->pose[e].orientation.x;
    gazeboPose_.orientation.y = ms->pose[e].orientation.y;
    gazeboPose_.orientation.z = ms->pose[e].orientation.z;
    gazeboPose_.orientation.w = ms->pose[e].orientation.w;

    localPose_.header.frame_id = "localPose";
    localPose_.pose = gazeboPose_;
    localPose_.pose.position.x = gazeboPose_.position.y - minefieldCenter_.position.y;
    localPose_.pose.position.y = -gazeboPose_.position.x + minefieldCenter_.position.x;
    double yaw = tf::getYaw(gazeboPose_.orientation) - tf::getYaw(minefieldCenter_.orientation);
    quaternionTFToMsg(tf::createQuaternionFromYaw(yaw),localPose_.pose.orientation);

//    cout << ms->name[e] << " "
//         << gazeboPose_.position.x << " " << gazeboPose_.position.y << endl;
}

const geometry_msgs::PoseStamped & TrueRobotPose::getLeftCoilPose()
{
    tf::StampedTransform coilTransform;

    ros::Time now = ros::Time::now();
    try{
        // faster lookup transform so far
        listener->waitForTransform("/base_link", "/left_coil", now, ros::Duration(2.0));
        listener->lookupTransform("/base_link", "/left_coil", now, coilTransform);
    }
    catch (tf::TransformException &ex) {
//        ROS_ERROR("%s",ex.what());
        // 30 Hz -- maybe too fast
        ros::Duration(0.05).sleep();
        return emptyPose_;
    }

    // Use reference robot pose
    tf::Transform robotTransform;
    robotTransform.setOrigin( tf::Vector3(localPose_.pose.position.x,
                                          localPose_.pose.position.y,
                                          localPose_.pose.position.z) );
    tf::Quaternion q;
    quaternionMsgToTF(localPose_.pose.orientation,q);
    robotTransform.setRotation(q);

    // Compute corrected coil pose
    tf::Transform Result;
    Result.mult(robotTransform, coilTransform);

    leftCoilPose_.header.frame_id = "left_coil";
    leftCoilPose_.header.stamp = ros::Time::now();
    leftCoilPose_.pose.position.x = Result.getOrigin().x();
    leftCoilPose_.pose.position.y = Result.getOrigin().y();
    leftCoilPose_.pose.position.z = Result.getOrigin().z();
    quaternionTFToMsg(Result.getRotation(),leftCoilPose_.pose.orientation);

    return leftCoilPose_;
}

const geometry_msgs::PoseStamped & TrueRobotPose::getRightCoilPose()
{
    tf::StampedTransform coilTransform;

    ros::Time now = ros::Time::now();
    try{
        // faster lookup transform so far
        listener->waitForTransform("/base_link", "/right_coil", now, ros::Duration(2.0));
        listener->lookupTransform("/base_link", "/right_coil", now, coilTransform);
    }
    catch (tf::TransformException &ex) {
//        ROS_ERROR("%s",ex.what());
        // 30 Hz -- maybe too fast
        ros::Duration(0.05).sleep();
        return emptyPose_;
    }

    // Use reference robot pose
    tf::Transform robotTransform;
    robotTransform.setOrigin( tf::Vector3(localPose_.pose.position.x,
                                          localPose_.pose.position.y,
                                          localPose_.pose.position.z) );
    tf::Quaternion q;
    quaternionMsgToTF(localPose_.pose.orientation,q);
    robotTransform.setRotation(q);

    // Compute corrected coil pose
    tf::Transform Result;
    Result.mult(robotTransform, coilTransform);

    rightCoilPose_.header.frame_id = "left_coil";
    rightCoilPose_.header.stamp = ros::Time::now();
    rightCoilPose_.pose.position.x = Result.getOrigin().x();
    rightCoilPose_.pose.position.y = Result.getOrigin().y();
    rightCoilPose_.pose.position.z = Result.getOrigin().z();
    quaternionTFToMsg(Result.getRotation(),rightCoilPose_.pose.orientation);

    return rightCoilPose_;
}

const vector<geometry_msgs::PoseStamped> & TrueRobotPose::getWheelsPoses()
{
    if(localPose_.header.frame_id.compare("UNDEF") == 0)
        return emptyVector;

    // Use reference robot pose
    tf::Transform robotTransform;
    robotTransform.setOrigin( tf::Vector3(localPose_.pose.position.x,
                                          localPose_.pose.position.y,
                                          localPose_.pose.position.z) );
    tf::Quaternion q;
    quaternionMsgToTF(localPose_.pose.orientation,q);
    robotTransform.setRotation(q);

    // Get relative pose of each wheel
    tf::StampedTransform wheelTransform;
    tf::Transform Result;

    //// 0 - BACK LEFT WHEEL
    ros::Time now = ros::Time::now();
    try{
        // faster lookup transform so far
        listener->waitForTransform("/base_link", "/p3at_back_left_wheel", now, ros::Duration(2.0));
        listener->lookupTransform("/base_link", "/p3at_back_left_wheel", now, wheelTransform);
    }
    catch (tf::TransformException &ex) {
//        ROS_ERROR("%s",ex.what());
        // 30 Hz -- maybe too fast
        ros::Duration(0.05).sleep();
        return emptyVector;
    }


    // Compute corrected coil pose
    Result.mult(robotTransform, wheelTransform);

    wheelsPoses[0].header.frame_id = "p3at_back_left_wheel";
    wheelsPoses[0].header.stamp = ros::Time::now();
    wheelsPoses[0].pose.position.x = Result.getOrigin().x();
    wheelsPoses[0].pose.position.y = Result.getOrigin().y();
    wheelsPoses[0].pose.position.z = Result.getOrigin().z();
    quaternionTFToMsg(Result.getRotation(),wheelsPoses[0].pose.orientation);

    //// 1 - BACK RIGHT WHEEL
//    now = ros::Time::now();
    try{
        // faster lookup transform so far
        listener->waitForTransform("/base_link", "/p3at_back_right_wheel", now, ros::Duration(2.0));
        listener->lookupTransform("/base_link", "/p3at_back_right_wheel", now, wheelTransform);
    }
    catch (tf::TransformException &ex) {
//        ROS_ERROR("%s",ex.what());
        // 30 Hz -- maybe too fast
        ros::Duration(0.05).sleep();
        return emptyVector;
    }

    // Compute corrected coil pose
    Result.mult(robotTransform, wheelTransform);

    wheelsPoses[1].header.frame_id = "p3at_back_right_wheel";
    wheelsPoses[1].header.stamp = ros::Time::now();
    wheelsPoses[1].pose.position.x = Result.getOrigin().x();
    wheelsPoses[1].pose.position.y = Result.getOrigin().y();
    wheelsPoses[1].pose.position.z = Result.getOrigin().z();
    quaternionTFToMsg(Result.getRotation(),wheelsPoses[1].pose.orientation);

    //// 2 - FRONT LEFT WHEEL
//    now = ros::Time::now();
    try{
        // faster lookup transform so far
        listener->waitForTransform("/base_link", "/p3at_front_left_wheel", now, ros::Duration(2.0));
        listener->lookupTransform("/base_link", "/p3at_front_left_wheel", now, wheelTransform);
    }
    catch (tf::TransformException &ex) {
//        ROS_ERROR("%s",ex.what());
        // 30 Hz -- maybe too fast
        ros::Duration(0.05).sleep();
        return emptyVector;
    }

    // Compute corrected coil pose
    Result.mult(robotTransform, wheelTransform);

    wheelsPoses[2].header.frame_id = "p3at_front_left_wheel";
    wheelsPoses[2].header.stamp = ros::Time::now();
    wheelsPoses[2].pose.position.x = Result.getOrigin().x();
    wheelsPoses[2].pose.position.y = Result.getOrigin().y();
    wheelsPoses[2].pose.position.z = Result.getOrigin().z();
    quaternionTFToMsg(Result.getRotation(),wheelsPoses[2].pose.orientation);

    //// 3 - FRONT RIGHT WHEEL
//    now = ros::Time::now();
    try{
        // faster lookup transform so far
        listener->waitForTransform("/base_link", "/p3at_front_right_wheel", now, ros::Duration(2.0));
        listener->lookupTransform("/base_link", "/p3at_front_right_wheel", now, wheelTransform);
    }
    catch (tf::TransformException &ex) {
//        ROS_ERROR("%s",ex.what());
        // 30 Hz -- maybe too fast
        ros::Duration(0.05).sleep();
        return emptyVector;
    }

    // Compute corrected coil pose
    Result.mult(robotTransform, wheelTransform);

    wheelsPoses[3].header.frame_id = "p3at_front_right_wheel";
    wheelsPoses[3].header.stamp = ros::Time::now();
    wheelsPoses[3].pose.position.x = Result.getOrigin().x();
    wheelsPoses[3].pose.position.y = Result.getOrigin().y();
    wheelsPoses[3].pose.position.z = Result.getOrigin().z();
    quaternionTFToMsg(Result.getRotation(),wheelsPoses[3].pose.orientation);

    return wheelsPoses;
}
