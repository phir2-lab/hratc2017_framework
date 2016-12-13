#ifndef COILSIGNALSIMULATOR_H
#define COILSIGNALSIMULATOR_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <metal_detector_msgs/Coil.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <vector>


using namespace std;

#include "../config/config.h"
#include "../config/robotPose.h"
#include "../config/trueRobotPose.h"


struct metalObject
{
    float maxValue;
    float stdDev;
    float eta, var;
    float x, y;
};

class CoilSignalSimulator
{
    public:
        CoilSignalSimulator();

        void run();

    private:
        void initMetalObjects();
        void computeDistanceToMines();
        float Gaussian(float sqrdist, const metalObject& m);
        float GaussianMine(float sqrdist, const metalObject& m);
        void publishSimulatedCoilValues();

        ros::NodeHandle* n;
        ros::Rate* rate;
        tf::TransformListener* listener;
        Config* config;
        RobotPose* robotPose;
        TrueRobotPose* trueRobotPose;
        unsigned int countMsgs;

        geometry_msgs::PoseStamped leftCoilPose;
        geometry_msgs::PoseStamped rightCoilPose;
        float leftValue;
        float rightValue;

        std_msgs::Bool canStart;
        ros::Publisher pub_startEveryone;
        ros::Publisher pub_coils;

        ros::WallTime start, last, current;

        // simulation settings
        float maxValueAtMine;
        float stdDevMine;
        float noise;
        float minValue;

        metalObject mine;
        vector<metalObject> otherObjects;

};

#endif /* COILSIGNALSIMULATOR_H */
