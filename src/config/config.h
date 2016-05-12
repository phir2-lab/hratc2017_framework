#ifndef CONFIG_H
#define CONFIG_H

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <iostream>

using namespace std;

class Position2D
{
    public:
        Position2D(double a, double b)
        {
            x=a; y=b;
        }

        friend ostream& operator<<(ostream& os, const Position2D& p)
        {
            os << "(" << p.x << ',' << p.y << ")";
            return os;
        }

        double x, y;

};


class Config
{
    public:
        Config(ros::NodeHandle *nh);

        bool hasEnded();

        // MAP INFO
        vector<tf::Vector3> minefieldCorners;
        tf::Vector3 lowerBound;
        tf::Vector3 upperBound;


        double width;
        double height;
        double resolution;
        int numCellsInX;
        int numCellsInY;

        // MINES INFO
        int numMines;
        bool randomMines;
	bool GPS_coordinate;
        double detectionMinDist;
        double explosionMaxDist;
        vector<Position2D> minesPositions;

    private:
        ros::NodeHandle* n;
        tf::TransformListener listener;

        tf::Vector3 getMinefieldOrigin();
        void readMinefieldCorners();
        void readMinefieldCornersFromTopic(const visualization_msgs::MarkerArray::ConstPtr & corners);
        void readJudgeInformation();
        void readMinesPositions_Cartesian();
	void readMinesPositions_GPS();

        bool canStart;
        ros::Subscriber sub_corners;

};

#endif /* CONFIG_H */
