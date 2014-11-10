#ifndef JUDGE_H
#define JUDGE_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

using namespace std;

#include "robotPose.h"
#include "config.h"

enum mineType { PROPERLY_DETECTED, WRONGLY_DETECTED, KNOWN_EXPLODED, UNKNOWN_EXPLODED};

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
        float robotZ;

        ros::Subscriber sub_setMine;
        ros::Subscriber sub_occupancyGrid;

        ros::Publisher pub_trueMinesMarker;
        ros::Publisher pub_properlyDetectedMinesMarker;
        ros::Publisher pub_wronglyDetectedMinesMarker;
        ros::Publisher pub_knownExplodedMinesMarker;
        ros::Publisher pub_unknownExplodedMinesMarker;
        visualization_msgs::MarkerArray trueMines;
        visualization_msgs::MarkerArray properlyDetectedMines;
        visualization_msgs::MarkerArray wronglyDetectedMines;
        visualization_msgs::MarkerArray knownExplodedMines;
        visualization_msgs::MarkerArray unknownExplodedMines;
        vector<bool> detected;
        vector<bool> exploded;
        vector<bool> unresolved;

        ros::Publisher pub_robotPath;
        visualization_msgs::Marker robotpath;

        void initializeMinesMarkers();
        void initializeRobotPath();

        void checkMineDetection(const geometry_msgs::PoseStamped::ConstPtr &guess);
        void checkMineExplosion();
        void addMineMarker(mineType mtype, Position2D pos);

        void updateMinesMarkers();
        void updateRobotPath();
};

#endif /* JUDGE_H */
