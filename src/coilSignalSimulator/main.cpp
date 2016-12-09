#include <ros/ros.h>
#include "CoilSignalSimulator.h"

#include <string>

int main(int argc,char **argv)
{
    ros::init(argc, argv, "coilSignalSimulator");
    ROS_INFO("coilSignalSimulator");

    CoilSignalSimulator css;

    css.run();

    return 0;
}
