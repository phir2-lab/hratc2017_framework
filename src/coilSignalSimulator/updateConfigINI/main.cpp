#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <string>
using namespace std;

#include "../../config/config.h"


int main( int argc, char** argv )
{
    
    // initialize ROS
    ros::init(argc, argv, "update_configINI");

    // Node handler pointer
    ros::NodeHandle* mapNodeHandler;
    mapNodeHandler = new ros::NodeHandle("~");
    ros::Rate loop_rate(10);

    // Config class
    Config* config;
    config = new Config(mapNodeHandler);
    cout << config->width << ' ' << config->height << endl;

    // Update config.ini
    string path = ros::package::getPath("hratc2017_framework");
    path += "/src/config/config.ini";
    cout<<"PATH: "<<path<<endl;
    FILE *arqIn;
    arqIn = fopen(path.c_str(), "w");
    fprintf(arqIn,"[MapDimensions]\n");
    fprintf(arqIn,"width = %f\n",config->width);
    fprintf(arqIn,"height = %f\n",config->height);
    fprintf(arqIn,"resolution = 0.05\n\n");
    fprintf(arqIn,"[Mines]\n");
    fprintf(arqIn,"nummines = %d\n",config->numMines);
    fprintf(arqIn,"randommines = false\n");
    fprintf(arqIn,"minespositions = ");
    for(int i = 0; i < config->numMines; i++){
       fprintf(arqIn,"%.1f,%.1f",config->minesPositions[i].x,config->minesPositions[i].y);
       if(i<config->numMines - 1) fprintf(arqIn,"|");
    }
    fprintf(arqIn,"\n");
    fprintf(arqIn,"detectionmindist = 0.5\n");
    fprintf(arqIn,"explosionmaxdist = 0.3\n");
    fclose(arqIn);


    ros::Publisher releaseAll = mapNodeHandler->advertise<std_msgs::Bool>("/configFirst", 1);
    std_msgs::Bool msg;
    msg.data = true;
    int i=0;
    while(i<10){
	    releaseAll.publish(msg);
	    ros::spinOnce();
	    loop_rate.sleep();
	    i++;
	}
}
