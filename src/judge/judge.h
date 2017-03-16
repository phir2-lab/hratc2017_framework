#ifndef JUDGE_H
#define JUDGE_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <fstream>

using namespace std;

#include "../config/config.h"
#include "../config/robotPose.h"
#include "../config/trueRobotPose.h"

enum mineType { PROPERLY_DETECTED, WRONGLY_DETECTED, KNOWN_EXPLODED, UNKNOWN_EXPLODED, VISITED_UNDETECTED, NOTVISITED_UNDETECTED};

class Judge
{
    public:
        Judge();

        void run();

    private:
        ros::NodeHandle* n;
        ros::Rate* rate;
        
        //TESTING
        RobotPose* trueRobotPose;
        //TrueRobotPose* trueRobotPose;

        bool canStart;
        Config* config;
        float robotZ;

        fstream logFile;

        ros::Subscriber sub_configDone;
        ros::Subscriber sub_setMine;
        ros::Subscriber sub_occupancyGrid;
        ros::Subscriber sub_coveredArea;

        float coverageRate;

        ros::Publisher pub_trueMinesMarker;
        ros::Publisher pub_properlyDetectedMinesMarker;
        ros::Publisher pub_wronglyDetectedMinesMarker;
        ros::Publisher pub_knownExplodedMinesMarker;
        ros::Publisher pub_unknownExplodedMinesMarker;
        ros::Publisher pub_visitedUndetectedMinesMarker;
        ros::Publisher pub_notVisitedUndetectedMinesMarker;
        visualization_msgs::MarkerArray trueMines;
        visualization_msgs::MarkerArray properlyDetectedMines;
        visualization_msgs::MarkerArray wronglyDetectedMines;
        visualization_msgs::MarkerArray knownExplodedMines;
        visualization_msgs::MarkerArray unknownExplodedMines;
        visualization_msgs::MarkerArray visitedUndetectedMines;
        visualization_msgs::MarkerArray notVisitedUndetectedMines;
        vector<bool> detected;
        vector<bool> exploded;
        vector<int> unresolved;

        ros::Publisher pub_robotPath;
        visualization_msgs::Marker robotpath;

        ros::Publisher pub_trueRobotMarker;
        visualization_msgs::Marker trueRobotMarker;


        ros::Publisher pub_textElapsedTime;
        ros::Publisher pub_textProperlyDetectedMines;
        ros::Publisher pub_textWronglyDetectedMines;
        ros::Publisher pub_textKnownExplodedMines;
        ros::Publisher pub_textUnknownExplodedMines;
        ros::Publisher pub_textVisitedUndetectedMines;
        ros::Publisher pub_textNotVisitedUndetectedMines;
        ros::Publisher pub_textCoverage;
        visualization_msgs::Marker textCoverage;
        visualization_msgs::Marker textElapsedTime;
        visualization_msgs::Marker textProperlyDetectedMines;
        visualization_msgs::Marker textWronglyDetectedMines;
        visualization_msgs::Marker textKnownExplodedMines;
        visualization_msgs::Marker textUnknownExplodedMines;
        visualization_msgs::Marker textVisitedUndetectedMines;
        visualization_msgs::Marker textNotVisitedUndetectedMines;

        ros::WallTime start, last, current;

        void initializeLogFile();
        void initializeMinesMarkers();
        void initializeScoreboard();
        void initializeRobotPath();
        void initializeTrueRobotMarker();

        void checkStart(const std_msgs::Bool::ConstPtr &flag);

        void checkMineDetection(const geometry_msgs::PoseStamped::ConstPtr &guess);
        void checkUnresolvedMines(const nav_msgs::OccupancyGrid::ConstPtr &grid);
        void checkMineExplosion();
        void addMineMarker(mineType mtype, Position2D pos);

        void getCoverageRate(const std_msgs::Float32::ConstPtr &rate);

        void updateMinesMarkers();
        void updateScoreboard();
        void updateRobotPath();
        void updateTrueRobotMarker();

        void saveLog();
};

#endif /* JUDGE_H */
