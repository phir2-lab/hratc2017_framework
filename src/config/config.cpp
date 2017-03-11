#include "config.h"

#include <sstream>
#include <angles/angles.h>
#include <visualization_msgs/Marker.h>
#include "../UTMConverter/UTMConverter.h"

using namespace std;

Config::Config(ros::NodeHandle *nh) : n(nh)
{
    canStart=false;
    //sub_corners = n->subscribe("/corners", 100, &Config::readMinefieldCornersFromTopic, this);

    tf::Vector3 pos = getMinefieldOrigin();
    ROS_INFO("Config -- Minefield Center x:%lf y:%lf z:%lf", pos.x(), pos.y(), pos.z());

    ros::Rate* rate = new ros::Rate(20);

    readMinefieldCorners();

//    while(!canStart){
//        ros::spinOnce();
//        rate->sleep();
//    }
    bool gpsCoordinate;
    n->getParam("judge/GPS_coordinate", gpsCoordinate);
    if(!gpsCoordinate){
	    readMinesPositions_Cartesian();
    }else{
	    readMinesPositions_GPS();
    }     

    readJudgeInformation();
}

void Config::readMinefieldCornersFromTopic(const visualization_msgs::MarkerArray::ConstPtr & corners)
{
    if(canStart)
        return;

//    cout << corners->markers.size()<< endl;

    for(int i=0; i<corners->markers.size(); i++){
//        visualization_msgs::Marker& m = corners->markers[i];
        minefieldCorners.push_back(tf::Vector3(corners->markers[i].pose.position.x, corners->markers[i].pose.position.y, corners->markers[i].pose.position.z));
    }

    // Convert minefield corners in relation to tf/minefield frame
    for(int i=0; i<minefieldCorners.size(); i++){
        geometry_msgs::PointStamped pointIn, pointOut;
        pointIn.header.frame_id = "/map";
        pointIn.point.x = minefieldCorners[i].x();
        pointIn.point.y = minefieldCorners[i].y();
        pointIn.point.z = minefieldCorners[i].z();

        try
        {
            listener.transformPoint("/minefield", pointIn, pointOut);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("Failure %s\n", ex.what());
        }

        minefieldCorners[i][0] = pointOut.point.x;
        minefieldCorners[i][1] = pointOut.point.y;
        minefieldCorners[i][2] = pointOut.point.z;
    }

    ROS_INFO("Config -- Minefield Corners in /minefield frame");
    for(int i=0; i<minefieldCorners.size(); i++)
        ROS_INFO("Config -- Corner%d x:%lf y:%lf z:%lf", i+1, minefieldCorners[i].x(), minefieldCorners[i].y(), minefieldCorners[i].z());

    // Find boundaries
    lowerBound=upperBound=minefieldCorners[0];
    for(int i=1; i<minefieldCorners.size(); ++i){
        lowerBound[0] = min(minefieldCorners[i].x(),lowerBound.x());
        lowerBound[1] = min(minefieldCorners[i].y(),lowerBound.y());
        lowerBound[2] = min(minefieldCorners[i].z(),lowerBound.z());
        upperBound[0] = max(minefieldCorners[i].x(),upperBound.x());
        upperBound[1] = max(minefieldCorners[i].y(),upperBound.y());
        upperBound[2] = max(minefieldCorners[i].z(),upperBound.z());
    }
    lowerBound[0] -= 0.2;
    lowerBound[1] -= 0.2;
    upperBound[0] += 0.2;
    upperBound[1] += 0.2;

    width = upperBound.x()-lowerBound.x();
    height = upperBound.y()-lowerBound.y();
    ROS_INFO("Config -- Minefield xi:%lf xf:%lf yi:%lf yf:%lf = w:%lf h:%lf",
             lowerBound.x(), upperBound.x(), lowerBound.y(), upperBound.y(), width, height);

    canStart = true;
}

tf::Vector3 Config::getMinefieldOrigin()
{
    tf::StampedTransform transform;
    tf::Vector3 origin;

    bool success;
    while(!success){
        try{
            success=true;
            // faster lookup transform so far
            listener.lookupTransform("/map", "/minefield", ros::Time(0), transform);

        }
        catch (tf::TransformException &ex) {
            success=false;
            ros::Duration(0.05).sleep();
        }
    }

    origin[0] = transform.getOrigin().x();
    origin[1] = transform.getOrigin().y();
    origin[2] = transform.getOrigin().z();
    return origin;
}

void Config::readMinefieldCorners()
{
    string s, s1, s2, s3;
    stringstream ss;

    // Get minefield corners
    int count=1;
    while(true){
        ss.str("");
        ss << count;
        s = "minefield/corner"+ss.str();

        if(!n->hasParam(s))
            break;

        // Convert the GPS coordinates into UTM coordinates
        UTMCoordinates utm;
        sensor_msgs::NavSatFix fix;

        s1 = s+"/latitude";
        s2 = s+"/longitude";
        s3 = s+"/altitude";

        if(!n->hasParam(s1) && !n->hasParam(s2) && !n->hasParam(s3))
        {
            ROS_FATAL("Config -- Unable to start without the 4 corners of the minefield!!!");
            ROS_BREAK();
        }
        n->getParam(s1, fix.latitude);
        n->getParam(s2, fix.longitude);
        n->getParam(s3, fix.altitude);

        //UTMConverter::latitudeAndLongitudeToUTMCoordinates(fix, utm);
        minefieldCorners.push_back(tf::Vector3(fix.latitude, fix.longitude, fix.altitude));

        count++;
    }

//    ROS_INFO("Config -- Minefield Corners in /map frame");
//    for(int i=0; i<minefieldCorners.size(); i++)
//        ROS_INFO("Config -- Corner%d x:%lf y:%lf z:%lf", i+1, minefieldCorners[i].x(), minefieldCorners[i].y(), minefieldCorners[i].z());

    // Convert minefield corners in relation to tf/minefield frame
    for(int i=0; i<minefieldCorners.size(); i++){
        geometry_msgs::PointStamped pointIn, pointOut;
        pointIn.header.frame_id = "/map";
        pointIn.point.x = minefieldCorners[i].x();
        pointIn.point.y = minefieldCorners[i].y();
        pointIn.point.z = minefieldCorners[i].z();

        try
        {
            listener.transformPoint("/minefield", pointIn, pointOut);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("Failure %s\n", ex.what());
        }

        minefieldCorners[i][0] = pointOut.point.x;
        minefieldCorners[i][1] = pointOut.point.y;
        minefieldCorners[i][2] = pointOut.point.z;
    }

    ROS_INFO("Config -- Minefield Corners in /minefield frame");
    for(int i=0; i<minefieldCorners.size(); i++)
        ROS_INFO("Config -- Corner%d x:%lf y:%lf z:%lf", i+1, minefieldCorners[i].x(), minefieldCorners[i].y(), minefieldCorners[i].z());

    // Find boundaries
    lowerBound=upperBound=minefieldCorners[0];
    for(int i=1; i<minefieldCorners.size(); ++i){
        lowerBound[0] = min(minefieldCorners[i].x(),lowerBound.x());
        lowerBound[1] = min(minefieldCorners[i].y(),lowerBound.y());
        lowerBound[2] = min(minefieldCorners[i].z(),lowerBound.z());
        upperBound[0] = max(minefieldCorners[i].x(),upperBound.x());
        upperBound[1] = max(minefieldCorners[i].y(),upperBound.y());
        upperBound[2] = max(minefieldCorners[i].z(),upperBound.z());
    }
    lowerBound[0] -= 0.2;
    lowerBound[1] -= 0.2;
    upperBound[0] += 0.2;
    upperBound[1] += 0.2;

    width = upperBound.x()-lowerBound.x();
    height = upperBound.y()-lowerBound.y();
    ROS_INFO("Config -- Minefield xi:%lf xf:%lf yi:%lf yf:%lf = w:%lf h:%lf",
             lowerBound.x(), upperBound.x(), lowerBound.y(), upperBound.y(), width, height);
}

void Config::readMinesPositions_Cartesian()
{
    if(!n->hasParam("judge/num_mines") && !n->hasParam("judge/mines_positions"))
    {
        ROS_FATAL("Config -- Unable to start without the positions of the mines!!!");
        ROS_BREAK();
    }

    // Read mines information
    n->getParam("judge/num_mines", numMines);

    if(randomMines == false){
        string s;
        stringstream ss;
        double x, y;

        // Read mines positions
        for(int i=1; i<=numMines; i++){
            ss.str("");
            ss << i;
            s="judge/mines_positions/mine"+ss.str()+"/x";
            n->getParam(s.c_str(), x);
            s="judge/mines_positions/mine"+ss.str()+"/y";
            n->getParam(s.c_str(), y);
            minesPositions.push_back(Position2D(x,y));

        }

//        ROS_INFO("Config -- Mines positions in /minefield frame");
//        for(int i=0; i<minesPositions.size(); i++)
//            ROS_INFO("Config -- Mine%d x:%lf y:%lf", i+1, minesPositions[i].x, minesPositions[i].y);
    }
}

void Config::readMinesPositions_GPS()
{

    if(!n->hasParam("judge/num_mines") && !n->hasParam("judge/mines_positions"))
    {
        ROS_FATAL("Config -- Unable to start without the positions of the mines!!!");
        ROS_BREAK();
    }

    // Read mines information
    n->getParam("judge/num_mines", numMines);

    if(randomMines == false){
        string s;
        stringstream ss;
        double z;

        // Read mines positions
        for(int i=1; i<=numMines; i++){
	    UTMCoordinates utm;
            sensor_msgs::NavSatFix fix;
	    z = 0;
            ss.str("");
            ss << i;

            s="judge/mines_positions/mine"+ss.str()+"/x";
            n->getParam(s.c_str(), fix.longitude);
            s="judge/mines_positions/mine"+ss.str()+"/y";
            n->getParam(s.c_str(), fix.latitude);
	    fix.altitude = z;
	    UTMConverter::latitudeAndLongitudeToUTMCoordinates(fix, utm);
	    minesPositions.push_back(Position2D(utm.easting, utm.northing));
        }

		    // Convert minefield corners in relation to tf/minefield frame
    for(int i=0; i<minesPositions.size(); i++){
        geometry_msgs::PointStamped pointIn, pointOut;
        pointIn.header.frame_id = "/map";
        pointIn.point.x = minesPositions[i].x;
        pointIn.point.y = minesPositions[i].y;

        try
        {
            listener.transformPoint("/minefield", pointIn, pointOut);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("Failure %s\n", ex.what());
        }

        minesPositions[i].x = pointOut.point.x;
        minesPositions[i].y = pointOut.point.y;
    }

//        ROS_INFO("Config -- Mines positions in /minefield frame");
//        for(int i=0; i<minesPositions.size(); i++)
//            ROS_INFO("Config -- Mine%d x:%lf y:%lf", i+1, minesPositions[i].x, minesPositions[i].y);
    }
}


void Config::readJudgeInformation()
{
    string s;

    // Read judge information - use default if not available
    s = "judge/GPS_coordinate";
    n->param<bool>(s, GPS_coordinate, false);
    s = "judge/map_resolution";
    n->param<double>(s, resolution, 0.05);
    s = "judge/random_mines";
    n->param<bool>(s, randomMines, false);
    s = "judge/detection_min_dist";
    n->param<double>(s, detectionMinDist, 0.5);
    s = "judge/explosion_max_dist";
    n->param<double>(s, explosionMaxDist, 0.3);

    numCellsInX = width/resolution;
    numCellsInY = height/resolution;
}
