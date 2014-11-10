#ifndef MINEFIELDVIEWER_H
#define MINEFIELDVIEWER_H
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include "../config/config.h"

#include <string>
using std::string;

#include <vector>
using std::vector;

#define WIDTH  1000
#define HEIGHT 1000
#define RESOLUTION 0.05 // 20 cells per meter
#define DETECTION_RADIUS 0.15 //m

// Minefield Viewer class
class minefieldViewer
{
public:
    minefieldViewer();
    void run();
private:
    // Node handler pointer
    ros::NodeHandle* mapNodeHandler;

    // the coverage map
    nav_msgs::OccupancyGrid grid;

    // coils and coverage prereqs
    bool getCoilTransform(int i);
    void fillGrid();
    tf::StampedTransform transform;
    vector< tf::TransformListener* > listeners;
    
    // radius in cells
    float cellRadius;

    // Config class
    Config* config;

    // coverage rate
    float coverage;
  
};

#endif // MINEFIELDVIEWER_H
