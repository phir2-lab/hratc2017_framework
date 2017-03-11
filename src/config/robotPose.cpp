#include "robotPose.h"

RobotPose::RobotPose(string targetFrame, string inputTopic)
{
    target_frame_ = targetFrame;
    input_topic_ = inputTopic;

    sub_.subscribe(n_, input_topic_, 10);
    tf_filter_ = new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(sub_, tf_, target_frame_, 10);
    tf_filter_->registerCallback( boost::bind(&RobotPose::robotPoseCallback, this, _1) );

    listener = new tf::TransformListener;

//    // start tf listeners -- one per wheel
//    for(int i=0; i<4;++i){
//        listeners.push_back(new tf::TransformListener);
//    }
    wheelsPoses.resize(4);


    emptyPose_.header.frame_id = "UNDEF";
    emptyPose_.pose.position.x = 0;
    emptyPose_.pose.position.y = 0;
    emptyPose_.pose.position.z = 0;
    emptyPose_.pose.orientation.x = 0;
    emptyPose_.pose.orientation.y = 0;
    emptyPose_.pose.orientation.z = 0;
    emptyPose_.pose.orientation.w = 1;
    localPose_ = globalPose_ = emptyPose_;
}

const geometry_msgs::PoseStamped & RobotPose::getGlobalPose()
{
    return globalPose_;
}

const geometry_msgs::PoseStamped & RobotPose::getLocalPose()
{
    return localPose_;
}

//  Callback to register with tf::MessageFilter to be called when transforms are available
void RobotPose::robotPoseCallback(const boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped>& msg)
{
    globalPose_.header.frame_id = msg->header.frame_id;
//        cout << globalPose_.header.frame_id << ' ' << target_frame_ << endl;
    globalPose_.header.stamp = msg->header.stamp;
    globalPose_.pose.position.x = msg->pose.pose.position.x;
    globalPose_.pose.position.y = msg->pose.pose.position.y;
    globalPose_.pose.position.z = msg->pose.pose.position.z;
    globalPose_.pose.orientation = msg->pose.pose.orientation;

    try
    {
        tf_.transformPose(target_frame_, globalPose_, localPose_);
//            ROS_INFO("Robot is at x:%lf y:%lf yaw:%lf", pose_in.pose.position.x, pose_in.pose.position.y, tf::getYaw(pose_in.pose.orientation));
//            ROS_INFO("Robot is at x:%lf y:%lf yaw:%lf", pose_out.pose.position.x, pose_out.pose.position.y, tf::getYaw(pose_out.pose.orientation));
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("Failure %s\n", ex.what());
    }
}

const vector<geometry_msgs::PoseStamped> & RobotPose::getWheelsPoses()
{
    tf::StampedTransform transform;

    try{
        // faster lookup transform so far
        listener->lookupTransform("/minefield", "/p3at_back_left_wheel", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
//        ROS_ERROR("%s",ex.what());
        // 30 Hz -- maybe too fast
        ros::Duration(0.05).sleep();
        return emptyVector;
    }

    wheelsPoses[0].header.frame_id = "p3at_back_left_wheel";
    wheelsPoses[0].header.stamp = ros::Time::now();
    wheelsPoses[0].pose.position.x = transform.getOrigin().x();
    wheelsPoses[0].pose.position.y = transform.getOrigin().y();
    wheelsPoses[0].pose.position.z = transform.getOrigin().z();
    quaternionTFToMsg(transform.getRotation(),wheelsPoses[0].pose.orientation);

    try{
        // faster lookup transform so far
        listener->lookupTransform("/minefield", "/p3at_back_right_wheel", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
//        ROS_ERROR("%s",ex.what());
        // 30 Hz -- maybe too fast
        ros::Duration(0.05).sleep();
        return emptyVector;
    }

    wheelsPoses[1].header.frame_id = "p3at_back_right_wheel";
    wheelsPoses[1].header.stamp = ros::Time::now();
    wheelsPoses[1].pose.position.x = transform.getOrigin().x();
    wheelsPoses[1].pose.position.y = transform.getOrigin().y();
    wheelsPoses[1].pose.position.z = transform.getOrigin().z();
    quaternionTFToMsg(transform.getRotation(),wheelsPoses[1].pose.orientation);

    try{
        // faster lookup transform so far
        listener->lookupTransform("/minefield", "/p3at_front_left_wheel", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
//        ROS_ERROR("%s",ex.what());
        // 30 Hz -- maybe too fast
        ros::Duration(0.05).sleep();
        return emptyVector;
    }

    wheelsPoses[2].header.frame_id = "p3at_front_left_wheel";
    wheelsPoses[2].header.stamp = ros::Time::now();
    wheelsPoses[2].pose.position.x = transform.getOrigin().x();
    wheelsPoses[2].pose.position.y = transform.getOrigin().y();
    wheelsPoses[2].pose.position.z = transform.getOrigin().z();
    quaternionTFToMsg(transform.getRotation(),wheelsPoses[2].pose.orientation);

    try{
        // faster lookup transform so far
        listener->lookupTransform("/minefield", "/p3at_front_right_wheel", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
//        ROS_ERROR("%s",ex.what());
        // 30 Hz -- maybe too fast
        ros::Duration(0.05).sleep();
        return emptyVector;
    }

    wheelsPoses[3].header.frame_id = "p3at_front_right_wheel";
    wheelsPoses[3].header.stamp = ros::Time::now();
    wheelsPoses[3].pose.position.x = transform.getOrigin().x();
    wheelsPoses[3].pose.position.y = transform.getOrigin().y();
    wheelsPoses[3].pose.position.z = transform.getOrigin().z();
    quaternionTFToMsg(transform.getRotation(),wheelsPoses[3].pose.orientation);


    return wheelsPoses;
}

const geometry_msgs::PoseStamped & RobotPose::getLeftCoilPose()
{
    tf::StampedTransform transform;

    try{
        // faster lookup transform so far
        listener->lookupTransform("/minefield", "/left_coil", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
//        ROS_ERROR("%s",ex.what());
        // 30 Hz -- maybe too fast
        ros::Duration(0.05).sleep();
        return emptyPose_;
    }

    leftCoilPose_.header.frame_id = "left_coil";
    leftCoilPose_.header.stamp = ros::Time::now();
    leftCoilPose_.pose.position.x = transform.getOrigin().x();
    leftCoilPose_.pose.position.y = transform.getOrigin().y();
    leftCoilPose_.pose.position.z = transform.getOrigin().z();
    quaternionTFToMsg(transform.getRotation(),leftCoilPose_.pose.orientation);

    return leftCoilPose_;
}

const geometry_msgs::PoseStamped & RobotPose::getRightCoilPose()
{
    tf::StampedTransform transform;

    try{
        // faster lookup transform so far
        listener->lookupTransform("/minefield", "/right_coil", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
//        ROS_ERROR("%s",ex.what());
        // 30 Hz -- maybe too fast
        ros::Duration(0.05).sleep();
        return emptyPose_;
    }

    rightCoilPose_.header.frame_id = "right_coil";
    rightCoilPose_.header.stamp = ros::Time::now();
    rightCoilPose_.pose.position.x = transform.getOrigin().x();
    rightCoilPose_.pose.position.y = transform.getOrigin().y();
    rightCoilPose_.pose.position.z = transform.getOrigin().z();
    quaternionTFToMsg(transform.getRotation(),rightCoilPose_.pose.orientation);

    return rightCoilPose_;
}

