#ifndef JUDGE_H
#define JUDGE_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>

using namespace std;

#include "robotPose.h"
#include "../config/config.h"

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
        vector<int> unresolved;

        ros::Publisher pub_robotPath;
        visualization_msgs::Marker robotpath;

        ros::Publisher pub_textProperlyDetectedMines;
        ros::Publisher pub_textWronglyDetectedMines;
        ros::Publisher pub_textKnownExplodedMines;
        ros::Publisher pub_textUnknownExplodedMines;
        visualization_msgs::Marker textProperlyDetectedMines;
        visualization_msgs::Marker textWronglyDetectedMines;
        visualization_msgs::Marker textKnownExplodedMines;
        visualization_msgs::Marker textUnknownExplodedMines;


        void initializeMinesMarkers();
        void initializeScoreboard();
        void initializeRobotPath();

        void checkMineDetection(const geometry_msgs::PoseStamped::ConstPtr &guess);
        void checkUnresolvedMines(const nav_msgs::OccupancyGrid::ConstPtr &grid);
        void checkMineExplosion();
        void addMineMarker(mineType mtype, Position2D pos);

        void updateMinesMarkers();
        void updateScoreboard();
        void updateRobotPath();
};

#endif /* JUDGE_H */
