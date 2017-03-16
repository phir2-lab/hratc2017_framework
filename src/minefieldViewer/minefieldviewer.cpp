#include "minefieldviewer.h"
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <sstream>
#include <queue>

using namespace std;

minefieldViewer::minefieldViewer() :
    mapNodeHandler(new ros::NodeHandle("~")),
    grid(),
    transform(),
    listeners()
{
    ros::Subscriber sub_configDone = mapNodeHandler->subscribe("/configDone", 100, &minefieldViewer::checkStart, this);
    ros::Rate rate(20);

    // Check if it is simulation, so it must wait the generation of the mines map
    bool isSimulation;
    if(mapNodeHandler->getParam("isSimulation", isSimulation)==false)
    {
        canStart = true;
    }else{
        if(isSimulation)
            canStart = false;
        else
            canStart = true;
    }

    cout << "Waiting to start!" << endl;
    while (canStart == false)
    {
        ros::spinOnce();
        rate.sleep();
    }
    cout << "Done" << endl;

//    // loading config file at $(hratc201X_framework)/src/config/config.ini)
//    string filename;
//    if(mapNodeHandler->getParam("config", filename)==false)
//    {
//        ROS_ERROR("Failed to get param 'config'");
//    }
//    config = new Config(filename);
    config = new Config(mapNodeHandler);
    
    trueRobotPose = new RobotPose("/minefield","/robot_pose_ekf/odom");
    //trueRobotPose = new TrueRobotPose(mapNodeHandler);

    // Loading grid with config file data 
    grid.info.resolution = config->resolution;  // float32
    grid.info.width      = config->numCellsInX;       // uint32
    grid.info.height     = config->numCellsInY;      // uint32
    // 0-100 -> selecting gray (50)
    grid.data.resize(grid.info.width*grid.info.height, 50);
    // setting origin 
//    grid.info.origin.position.x = -config->numCellsInX/2.0*grid.info.resolution;    // uint32
//    grid.info.origin.position.y = -config->numCellsInY/2.0*grid.info.resolution;   // uint32
    grid.info.origin.position.x = config->lowerBound.x();    // uint32
    grid.info.origin.position.y = config->lowerBound.y();    // uint32

    initializeGrid();

    // detection radius in cells
    cellRadius = config->detectionMinDist/grid.info.resolution;

    // start tf listeners -- one per coil
    for(int i=0; i<3;++i)
        listeners.push_back(new tf::TransformListener);

    ROS_INFO("Minefieldviewer -- Done");
}

void minefieldViewer::initializeGrid()
{
    int h = grid.info.height;
    int w = grid.info.width;
    int count=0;

    // Rasterize boundaries of the minefield -- DDA algorithm
    for(int c=0; c<config->minefieldCorners.size(); ++c){
        int begin = c;
        int end = (c+1)%config->minefieldCorners.size();

        int x0 = round((config->minefieldCorners[begin].x() - grid.info.origin.position.x)/grid.info.resolution);
        int y0 = round((config->minefieldCorners[begin].y() - grid.info.origin.position.y)/grid.info.resolution);
        int x1 = round((config->minefieldCorners[end].x()   - grid.info.origin.position.x)/grid.info.resolution);
        int y1 = round((config->minefieldCorners[end].y()   - grid.info.origin.position.y)/grid.info.resolution);

        double difX = x1-x0;
        double difY = y1-y0;
        double dist = max(fabs(difX),fabs(difY));

        double deltaX = difX/dist;
        double deltaY = difY/dist;

        double i=x0;
        double j=y0;
        for(int k=0;k<=(int)dist;k++){
            i+=deltaX;
            j+=deltaY;

            if(grid.data[(int)i+(int)j*w] != 100)
                count++;

            grid.data[(int)i+(int)j*w] = 100;
        }
    }

    // Mark area outside minefield -- floodfill
    queue<pair<int,int> > cellsQueue;
    cellsQueue.push(pair<int,int>(0,0));
    grid.data[0+0*w] = 100;

    while(!cellsQueue.empty()){
        int i=cellsQueue.front().first;
        int j=cellsQueue.front().second;
        cellsQueue.pop();

        count++;

        if(i>0){
            if(grid.data[(i-1)+j*w] != 100){
                grid.data[(i-1)+j*w] = 100;
                cellsQueue.push(pair<int,int>(i-1,j));
            }
        }
        if(i<w-1){
            if(grid.data[(i+1)+j*w] != 100){
                grid.data[(i+1)+j*w] = 100;
                cellsQueue.push(pair<int,int>(i+1,j));
            }
        }
        if(j>0){
            if(grid.data[i+(j-1)*w] != 100){
                grid.data[i+(j-1)*w] = 100;
                cellsQueue.push(pair<int,int>(i,j-1));
            }
        }
        if(j<h-1){
            if(grid.data[i+(j+1)*w] != 100){
                grid.data[i+(j+1)*w] = 100;
                cellsQueue.push(pair<int,int>(i,j+1));
            }
        }

    }

    totalValidCells = h*w - count;
    cout << "h:" << h << " w:" << w << " count:" << count << "total: " << totalValidCells << endl;
}

void minefieldViewer::checkStart(const std_msgs::Bool::ConstPtr &flag)
{
    if(flag->data == true)
        canStart = true;
}

void minefieldViewer::run()
{
    // creating the coverage rate and the map
    ros::Rate r(30);
    ros::Publisher map_pub = mapNodeHandler->advertise<nav_msgs::OccupancyGrid>("occupancyGrid", 1);
    ros::Publisher cover_pub = mapNodeHandler->advertise<std_msgs::Float32>("coverageRate", 1);

    int count = 0;

    // ROS loop
    while (ros::ok())
    {
        leftCoilPose = trueRobotPose->getLeftCoilPose();
        if(leftCoilPose.header.frame_id.compare("UNDEF") != 0){
            count++;
            if(count > 10)
                fillGrid(leftCoilPose);
        }

        rightCoilPose = trueRobotPose->getRightCoilPose();
        if(rightCoilPose.header.frame_id.compare("UNDEF") != 0){
            count++;
            if(count > 10)
                fillGrid(rightCoilPose);
        }


//        // find transformations for each coil
//        for(int i=0; i<3;++i)
//        {
//            // getting left, middle and right coils transforms
//            if ( getCoilTransform(i) )
//            {
//                // mark grid cells as visited
//                fillGrid();
//            }
//        }
        // Setting frame id and timestamp for the occupancy grid
        grid.header.frame_id = "/minefield";
        grid.header.stamp = ros::Time::now();
        //plain grid height is set to the last coil z position
//        grid.info.origin.position.z = 0;
//        grid.info.origin.position.z = transform.getOrigin().z()-0.30;
        grid.info.origin.position.z = leftCoilPose.pose.position.z-0.30;

        // Publish the view of the coverage area
        map_pub.publish(grid);



        //publish the rate of coverage area
        std_msgs::Float32 rate;
//        rate.data = coverage/float(grid.info.width*grid.info.height);
        rate.data = coverage/totalValidCells;
        cover_pub.publish( rate );

        ros::spinOnce();
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
//        ROS_ERROR("%s",ex.what());
        bool no_error = false;
        // 30 Hz -- maybe too fast
        ros::Duration(0.05).sleep();
    }

    return no_error;
}

void minefieldViewer::fillGrid(geometry_msgs::PoseStamped& coilPose)
{ // fill detection area
    

    // get origin in cells
//    double w = round(transform.getOrigin().x()/grid.info.resolution + grid.info.width/ 2.0);
//    double h = round(transform.getOrigin().y()/grid.info.resolution + grid.info.height/2.0);
//    double w = round((transform.getOrigin().x()-grid.info.origin.position.x)/grid.info.resolution);
//    double h = round((transform.getOrigin().y()-grid.info.origin.position.y)/grid.info.resolution);
    double w = round((coilPose.pose.position.x - grid.info.origin.position.x)/grid.info.resolution);
    double h = round((coilPose.pose.position.y - grid.info.origin.position.y)/grid.info.resolution);

    for(int x=-cellRadius; x<+cellRadius; ++x)
    {
        for(int y=-cellRadius; y<+cellRadius; ++y)
        {
            if(
                ( w+x>=0) && ( w+x<grid.info.width) &&
                ( h+y>=0) && ( h+y<grid.info.height)
              )
            {
                //cout << "x " << x << " y " << y << endl;
                // setting color to scanned (white)
                if( sqrt(x*x+y*y) <= cellRadius )
                {
                    if(grid.data[(h+y)*grid.info.width + w+x]==100)
                        continue;

                    // count new cell
                    if(grid.data[(h+y)*grid.info.width + w+x]!=0)
                        coverage+=1;
                    // mark cell as covered
                    grid.data[(h+y)*grid.info.width + w+x] =0;
                }            
            }
        }
    }
}

void minefieldViewer::fillGrid()
{ // fill detection area


    // get origin in cells
//    double w = round(transform.getOrigin().x()/grid.info.resolution + grid.info.width/ 2.0);
//    double h = round(transform.getOrigin().y()/grid.info.resolution + grid.info.height/2.0);
    double w = round((transform.getOrigin().x()-grid.info.origin.position.x)/grid.info.resolution);
    double h = round((transform.getOrigin().y()-grid.info.origin.position.y)/grid.info.resolution);


    for(int x=-cellRadius; x<+cellRadius; ++x)
    {
        for(int y=-cellRadius; y<+cellRadius; ++y)
        {
            if(
                ( w+x>=0) && ( w+x<grid.info.width) &&
                ( h+y>=0) && ( h+y<grid.info.height)
              )
            {
                //cout << "x " << x << " y " << y << endl;
                // setting color to scanned (white)
                if( sqrt(x*x+y*y) <= cellRadius )
                {
                    if(grid.data[(h+y)*grid.info.width + w+x]==100)
                        continue;

                    // count new cell
                    if(grid.data[(h+y)*grid.info.width + w+x]!=0)
                        coverage+=1;
                    // mark cell as covered
                    grid.data[(h+y)*grid.info.width + w+x] =0;
                }
            }
        }
    }
}
