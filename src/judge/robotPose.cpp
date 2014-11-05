#include "robotPose.h"

RobotPose::RobotPose(string targetFrame, string inputTopic)
{
    target_frame_ = targetFrame;
    input_topic_ = inputTopic;

    sub_.subscribe(n_, input_topic_, 10);
    tf_filter_ = new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(sub_, tf_, target_frame_, 10);
    tf_filter_->registerCallback( boost::bind(&RobotPose::msgCallback, this, _1) );
}

const geometry_msgs::PoseStamped & RobotPose::GetGlobalPose()
{
    return globalPose_;
}

const geometry_msgs::PoseStamped & RobotPose::GetLocalPose()
{
    return localPose_;
}

//  Callback to register with tf::MessageFilter to be called when transforms are available
void RobotPose::msgCallback(const boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped>& msg)
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
