#include "minefieldviewer.h"
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
using std::cout;
using std::endl;
using std::cerr;


minefieldViewer::minefieldViewer() :
    mapNodeHandler(new ros::NodeHandle("~")),
    grid(),
    transform(),
    listeners()
{
    // loading config file at $(hratc201X_framework)/src/config/config.ini)
    string filename;
    if(mapNodeHandler->getParam("config", filename)==false)
    {
        ROS_ERROR("Failed to get param 'config'");
    }
    config = new Config(filename);

    // Loading grid with config file data 
    grid.info.resolution = config->resolution;  // float32
    grid.info.width      = config->width;       // uint32
    grid.info.height     = config->height;      // uint32
    // 0-100 -> selecting gray (50)
    grid.data.resize(grid.info.width*grid.info.height, 50);
    // setting origin 
    grid.info.origin.position.x = -config->width/2.0*grid.info.resolution;    // uint32
    grid.info.origin.position.y = -config->height/2.0*grid.info.resolution;   // uint32
    // detection radius in cells
    cellRadius = config->detectionMinDist/grid.info.resolution;

    // start tf listeners -- one per coil
    for(int i=0; i<3;++i)
        listeners.push_back(new tf::TransformListener);
}

void minefieldViewer::run()
{
    // creating the node handler and publisher for the map
    ros::Rate r(30);
    ros::Publisher map_pub = mapNodeHandler->advertise<nav_msgs::OccupancyGrid>("occupancyGrid", 1);
    ros::Publisher cover_pub = mapNodeHandler->advertise<std_msgs::Float32>("coverageRate", 1);

    // ROS loop
    while (ros::ok())
    {
        for(int i=0; i<3;++i)
        {
            // getting left, middle and right coils transforms
            if ( getCoilTransform(i) )
            {
                fillGrid();
            }
        }

        // Setting frame id and timestamp for the occupancy grid
        grid.header.frame_id = "/minefield";
        grid.header.stamp = ros::Time::now();
        grid.info.origin.position.z = transform.getOrigin().z()-0.30;                // uint32 -- set by hand

        // Publish the view of the coverage area
        map_pub.publish(grid);
        //publish the rate of coverage area
        std_msgs::Float32 rate;
        rate.data = coverage/float(grid.info.width*grid.info.height);
        cover_pub.publish( rate );

        r.sleep();
    }
}

bool minefieldViewer::getCoilTransform(int i)
{
    bool no_error = true;
    string coils[]={"/left_coil", "middle_coil", "/right_coil"};
    try{
        // faster lookup transform so far
        listeners[i]->lookupTransform("/minefield", coils[i],
        ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        bool no_error = false;
        // 30 Hz -- maybe too fast
        ros::Duration(0.05).sleep();
    }

    return no_error;
}

void minefieldViewer::fillGrid()
{ // fill detection area
    

    // get origin in cells
    double w = round(transform.getOrigin().x()/grid.info.resolution + grid.info.width/ 2.0);
    double h = round(transform.getOrigin().y()/grid.info.resolution + grid.info.height/2.0);

    for(int x=-cellRadius; x<+cellRadius; ++x)
    {
        for(int y=-cellRadius; y<+cellRadius; ++y)
        {
            if(w>=0 && w<grid.info.width && h>=0 && h<grid.info.height)
            {
                //cout << "x " << x << " y " << y << endl;
                // setting color to scanned (white)
                if( sqrt(x*x+y*y) <= cellRadius )
                {
                    // count new cell
                    if(grid.data[(h+x)*grid.info.width + w+y]!=0)
                        coverage+=1;
                    // mark cell as covered
                    grid.data[(h+x)*grid.info.width + w+y] =0;
                }            
            }
        }
    }
}
