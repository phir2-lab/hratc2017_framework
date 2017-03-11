#include "CoilSignalSimulator.h"

#include <iostream>
#include <cmath>
#include <stdlib.h>

using namespace std;

CoilSignalSimulator::CoilSignalSimulator()
{

    n = new ros::NodeHandle("~");

    rate = new ros::Rate(20);

    config = new Config(n);

    robotPose = new RobotPose("/minefield","/robot_pose_ekf/odom");
    trueRobotPose = new TrueRobotPose(n);

    canStart.data = true;
    pub_startEveryone = n->advertise<std_msgs::Bool>("/configDone", 1);
    pub_coils = n->advertise<metal_detector_msgs::Coil>("/coils", 1);

    countMsgs = 0;

    start = ros::WallTime::now();
    last = start;

    // simulation settings
    noise = 0.05;
    minValue = 0.3;

    initMetalObjects();
}

double random_number(double low, double high)
{
    return low + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(high-low)));
}

void CoilSignalSimulator::initMetalObjects()
{
    mine.maxValue = 0.8;
    mine.stdDev = 0.3; //10cm
    mine.eta = mine.maxValue * 1.0/(mine.stdDev * sqrt(2.0*M_PI));
    mine.var = mine.stdDev*mine.stdDev;
//    cout << "\n\n\n\n\nMINES: " << mine.eta << ' ' << mine.var << "\n\n\n\n" << endl;

    metalObject m;
    m.maxValue = 0.3;
    m.stdDev = 1; //1m
    m.x = -2.3;
    m.y = 0.2;
    m.eta = m.maxValue * 1.0/(m.stdDev * sqrt(2.0*M_PI));
    m.var = m.stdDev*m.stdDev;
    otherObjects.push_back(m);

    m.maxValue = 0.1;
    m.stdDev = 3; //3m
    m.x = -0.9;
    m.y = 0.2;
    m.eta = m.maxValue * 1.0/(m.stdDev * sqrt(2.0*M_PI));
    m.var = m.stdDev*m.stdDev;
    otherObjects.push_back(m);

    m.maxValue = 0.4;
    m.stdDev = 0.3; //30cm
    m.x = 3.4;
    m.y = -4.2;
    m.eta = m.maxValue * 1.0/(m.stdDev * sqrt(2.0*M_PI));
    m.var = m.stdDev*m.stdDev;
    otherObjects.push_back(m);

    m.maxValue = 0.3;
    m.stdDev = 0.8; // 80cm
    m.x = -1.3;
    m.y = -2.1;
    m.eta = m.maxValue * 1.0/(m.stdDev * sqrt(2.0*M_PI));
    m.var = m.stdDev*m.stdDev;
    otherObjects.push_back(m);

    m.maxValue = 0.2;
    m.stdDev = 2; //2m
    m.x = 1.9;
    m.y = 3.2;
    m.eta = m.maxValue * 1.0/(m.stdDev * sqrt(2.0*M_PI));
    m.var = m.stdDev*m.stdDev;
    otherObjects.push_back(m);
}

float CoilSignalSimulator::GaussianMine(float sqrdist, const metalObject& m)
{
    if(sqrdist > 0.09) // 30cm
        return 0.0;
    else
        return m.eta * exp(-0.5*sqrdist/m.var);
}

float CoilSignalSimulator::Gaussian(float sqrdist, const metalObject& m)
{
    if(sqrdist > 20.0*(m.var))
        return 0.0;
    else
        return m.eta * exp(-0.5*sqrdist/m.var);
}

void CoilSignalSimulator::computeDistanceToMines()
{
    // Left coil
    cout << "Left ";
    leftValue = minValue + random_number(-noise,noise);
    for(int m=0; m < config->numMines; m++){
        float sqr_dist = pow(config->minesPositions[m].x - leftCoilPose.pose.position.x, 2.0) +
                          pow(config->minesPositions[m].y - leftCoilPose.pose.position.y, 2.0);
        float val = Gaussian(sqr_dist,mine);
        cout << val << "(" << sqr_dist << ") ";
        leftValue += val;
    }
    cout << endl << flush;
    for(int m=0; m < otherObjects.size(); m++){
        float sqr_dist = pow(otherObjects[m].x - leftCoilPose.pose.position.x, 2.0) +
                          pow(otherObjects[m].y - leftCoilPose.pose.position.y, 2.0);
        float val = Gaussian(sqr_dist,otherObjects[m]);
        leftValue += val;
    }

    if(leftValue < 0.0)
        leftValue = 0.0;
    else if(leftValue > 1.0)
        leftValue = 1.0;

    // Right coil
    rightValue = minValue + random_number(-noise,noise);
    for(int m=0; m < config->numMines; m++){
        float sqr_dist = pow(config->minesPositions[m].x - rightCoilPose.pose.position.x, 2.0) +
                          pow(config->minesPositions[m].y - rightCoilPose.pose.position.y, 2.0);
        float val = Gaussian(sqr_dist,mine);
        rightValue += val;
    }
    for(int m=0; m < otherObjects.size(); m++){
        float sqr_dist = pow(otherObjects[m].x - rightCoilPose.pose.position.x, 2.0) +
                          pow(otherObjects[m].y - rightCoilPose.pose.position.y, 2.0);
        float val = Gaussian(sqr_dist,otherObjects[m]);
        rightValue += val;
    }

    if(rightValue < 0.0)
        rightValue = 0.0;
    else if(rightValue > 1.0)
        rightValue = 1.0;
}

void CoilSignalSimulator::publishSimulatedCoilValues()
{
    metal_detector_msgs::Coil coilMsg;
    coilMsg.header.seq = countMsgs++;
    coilMsg.header.frame_id = "coils";
    coilMsg.header.stamp = ros::Time::now();
    coilMsg.left_coil = leftValue;
    coilMsg.right_coil = rightValue;

    pub_coils.publish(coilMsg);
}

void CoilSignalSimulator::run()
{
    while (ros::ok())
    {
        pub_startEveryone.publish(canStart);

        // get coils poses
//        leftCoilPose = robotPose->getLeftCoilPose();
//        rightCoilPose = robotPose->getRightCoilPose();
        leftCoilPose = trueRobotPose->getLeftCoilPose();
        rightCoilPose = trueRobotPose->getRightCoilPose();

        string str = leftCoilPose.header.frame_id;
        if(str.compare("UNDEF") == 0){
            cout << "UNDEF coils" << endl;
        }else{

//            cout << "Left Position: " << leftCoilPose.pose.position.x << ' ' << leftCoilPose.pose.position.y << endl;
//            cout << "Right Position: " << rightCoilPose.pose.position.x << ' ' << rightCoilPose.pose.position.y << endl;

            computeDistanceToMines();

//            cout << "Left Value: " << leftValue << endl;
//            cout << "Right Value: " << rightValue << endl;

            publishSimulatedCoilValues();
        }

        ros::spinOnce();
        rate->sleep();
    }
}

