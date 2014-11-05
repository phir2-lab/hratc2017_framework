#ifndef JUDGE_H
#define JUDGE_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

using namespace std;

#include "robotPose.h"
#include "config.h"

class Judge
{
    public:
        Judge();

        void run();

    private:
        ros::NodeHandle* n;
        ros::Rate* rate;
        RobotPose* robotPose;
        Config* config;

        ros::Publisher pub_marker;
        uint32_t shape;

        float robotZ;

        ros::Publisher pub_trueMinesMarker;
        vector<visualization_msgs::Marker> trueMinesMarker;
        visualization_msgs::MarkerArray trueMinesMarkers;

        ros::Publisher pub_robotPath;
        visualization_msgs::Marker robotpath;

        void initializeMarker();
        void initializeTrueMinesMarkers();
        void initializeTrueMinesMarkers2();
        void initializeRobotPath();
        void updateMarker();
        void updateTrueMinesMarkers();
        void updateTrueMinesMarkers2();
        void updateRobotPath(int i);

};

#endif /* JUDGE_H */
