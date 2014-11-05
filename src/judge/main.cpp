#include <ros/ros.h>
#include "judge.h"

#include <string>

int main(int argc,char **argv)
{
    ros::init(argc, argv, "hratc_viewer");
    ROS_INFO("HRATC");

    Judge dredd;

    dredd.run();

    return 0;
}
