#include "minefieldviewer.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
using std::cout;
using std::endl;
using std::cerr;


minefieldViewer::minefieldViewer(float resolution, int w, int h, float r) :
    grid(),
    transform(),
    listeners()


{
    // start tf listeners -- one per coil
    for(int i=0; i<3;++i)
        listeners.push_back(new tf::TransformListener);

    // 0-100 -> selecting gray (50)
    grid.data.resize(w*h, 50);
    grid.info.resolution = resolution;  // float32
    grid.info.width      = w;           // uint32
    grid.info.height     = h;           // uint32

    // setting information
    grid.info.origin.position.x = -w/2.0*resolution;    // uint32
    grid.info.origin.position.y = -h/2.0*resolution;    // uint32
    

    // radius in cells
    cellRadius = r/resolution;
}

void minefieldViewer::run()
{
    // creating the node handler and publisher for the map
    ros::NodeHandle mapNodeHandler;
    ros::Rate r(30);
    ros::Publisher map_pub = mapNodeHandler.advertise<nav_msgs::OccupancyGrid>("occupancyGrid", 1);

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
        cout << "Z origin " << transform.getOrigin().z() << endl;
        grid.info.origin.position.z = transform.getOrigin().z()-0.30;                // uint32 -- set by hand

        // Publish the marker
        map_pub.publish(grid);

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
                    grid.data[(h+x)*grid.info.width + w+y] =0;
            }


        }
    }
}
