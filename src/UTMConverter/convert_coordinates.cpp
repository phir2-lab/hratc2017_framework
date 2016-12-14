#include "UTMConverter.h"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <fstream>

using namespace std;

float shiftX, shiftY;
UTMCoordinates utm;
float altitude;

UTMCoordinates localToUTM(geometry_msgs::Point p)
{
    UTMCoordinates u;
    u.easting = p.x + shiftX;
    u.northing = p.y + shiftY;
    u.hemisphere = utm.hemisphere;
    u.grid_zone = utm.grid_zone;
    return u;
}

void printToFile(fstream& file, string str, string space, sensor_msgs::NavSatFix gps)
{
    file << space << "   " << str << ": " << endl;
    file << space << "      latitude: " << gps.latitude << endl;
    file << space << "      longitude: " << gps.longitude << endl;
    file << space << "      altitude: " << altitude << endl << endl;
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "convert_coordinates");
    ROS_INFO("convert_coordinates");

    ros::NodeHandle* n;
    n = new ros::NodeHandle("~");
    ros::Rate* rate;
    rate = new ros::Rate(20);

    cout << std::setprecision(20);

    fstream output;
    output.open("/home/phi/GPScoordinates.yaml",std::fstream::out);
    if(!output.is_open()){
        cout << "CAGOU" << endl;
        return 0;
    }
    output << "GPScoordinates:" << endl;
    output << std::setprecision(20);


    // Computa o centro do minefield em UTM

    string s, s1, s2, s3;
    stringstream ss;

    sensor_msgs::NavSatFix gps_origin;
    s = "coordinates/GPS_origin";
    s1 = s+"/latitude";
    s2 = s+"/longitude";
    s3 = s+"/altitude";

    if(!n->hasParam(s1) && !n->hasParam(s2) && !n->hasParam(s3))
    {
        ROS_FATAL("ERROR -- coordinates/GPS_origin!!!");
        ROS_BREAK();
    }
    n->getParam(s1, gps_origin.latitude);
    n->getParam(s2, gps_origin.longitude);
    n->getParam(s3, gps_origin.altitude);
    altitude = gps_origin.altitude;
    cout << "Minefield center " << gps_origin.latitude << ' ' << gps_origin.longitude << endl;


    UTMConverter::latitudeAndLongitudeToUTMCoordinates(gps_origin, utm);
    cout << "Minefield center " << utm.easting << ' ' << utm.northing << endl;

    // Computa o centro do minefield em coordenadas locais (media dos cantos)

    geometry_msgs::Point center;
    center.x = 0.0;
    center.y = 0.0;

    int num_corners;
    if(!n->hasParam("coordinates/num_corners"))
    {
        ROS_FATAL("ERROR -- coordinates/num_corners!!!");
        ROS_BREAK();
    }
    n->getParam("coordinates/num_corners", num_corners);

    cout << "num_corners " << num_corners << endl;
    for(int c=0; c<num_corners; c++)
    {
            ss.str("");
            ss << c+1;
            s = "coordinates/corners_positions/corner"+ss.str();

            if(!n->hasParam(s))
                break;
            float x, y;
            n->getParam(s+"/x", x);
            n->getParam(s+"/y", y);
            cout << "p" << c+1 << ": " << x << ' ' << y << endl;

            center.x += x;
            center.y += y;
    }
    center.x /= (float) num_corners;
    center.y /= (float) num_corners;
    cout << "Minefield center (local): " << center.x << ' ' << center.y << endl;

    // Computa transformação entre coordenada local e global do centro do minefield
    shiftX = utm.easting - center.x;
    shiftY = utm.northing - center.y;

    // Transforma todos os pontos pra coordenada global (GPS)

    // minefield center
    UTMCoordinates utmcenter = localToUTM(center);
    cout << "Minefield center " << utmcenter.easting << ' ' << utmcenter.northing << endl;
    sensor_msgs::NavSatFix gpsCenter;
    UTMConverter::UTMCoordinatesToLatitudeAndLongitude(utmcenter,gpsCenter);
    cout << "Minefield center " << gpsCenter.latitude << ' ' << gpsCenter.longitude << endl;

    // robot
    geometry_msgs::Point robotLocal;
    n->getParam("coordinates/robot_origin/x", robotLocal.x);
    n->getParam("coordinates/robot_origin/y", robotLocal.y);
    UTMCoordinates utmRobot = localToUTM(robotLocal);
    cout << "robot utm " << utmRobot.easting << ' ' << utmRobot.northing << endl;
    sensor_msgs::NavSatFix gpsRobot;
    UTMConverter::UTMCoordinatesToLatitudeAndLongitude(utmRobot,gpsRobot);
    cout << "robot gps " << gpsRobot.latitude << ' ' << gpsRobot.longitude << endl;
    printToFile(output,"robot","",gpsRobot);

    // corners
    output << "   corners:" << endl;
    for(int c=0; c<num_corners; c++)
    {
            ss.str("");
            ss << c+1;
            s = "coordinates/corners_positions/corner"+ss.str();

            if(!n->hasParam(s))
                break;
            geometry_msgs::Point p;
            n->getParam(s+"/x", p.x);
            n->getParam(s+"/y", p.y);

            UTMCoordinates utmCorner = localToUTM(p);
            sensor_msgs::NavSatFix gpsCorner;
            UTMConverter::UTMCoordinatesToLatitudeAndLongitude(utmCorner,gpsCorner);
            printToFile(output,"corner"+ss.str(),"    ",gpsCorner);
    }

    // obstacles
    int num_obstacles;
    if(!n->hasParam("coordinates/num_obstacles"))
    {
        ROS_FATAL("ERROR -- coordinates/num_obstacles!!!");
        ROS_BREAK();
    }
    n->getParam("coordinates/num_obstacles", num_obstacles);

    output << "   obstacles:" << endl;
    for(int c=0; c<num_obstacles; c++)
    {
            ss.str("");
            ss << c+1;
            s = "coordinates/obstacles_positions/obstacle"+ss.str();

            if(!n->hasParam(s))
                break;
            geometry_msgs::Point p;
            n->getParam(s+"/x", p.x);
            n->getParam(s+"/y", p.y);

            UTMCoordinates utmObstacle = localToUTM(p);
            sensor_msgs::NavSatFix gpsObstacle;
            UTMConverter::UTMCoordinatesToLatitudeAndLongitude(utmObstacle,gpsObstacle);
            printToFile(output,"obstacle"+ss.str(),"    ",gpsObstacle);
    }

    output.close();

    return 0;
}
