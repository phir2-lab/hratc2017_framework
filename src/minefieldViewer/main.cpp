#include <ros/ros.h>
#include "minefieldviewer.h"
#include <iostream>
using std::cout;
using std::endl;
using std::cerr;

#define WIDTH  1000
#define HEIGHT 1000
#define RESOLUTION 0.05
#define DETECTION_RADIUS 0.15 //m


int main( int argc, char** argv )
{
    
    // initialize ROS
    ros::init(argc, argv, "mineFieldViewer");

    // initialize mineField
    minefieldViewer m(RESOLUTION, WIDTH, HEIGHT, DETECTION_RADIUS);
    m.run();
}
