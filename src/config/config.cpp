#include "config.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <errno.h>
#include <string.h>
#include <sstream>
#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;

Config::Config(string name)
{
    // Get full path of the config file
//    string filename = ros::package::getPath("hratc2014_framework");
//    filename += "/src/config/" + name;
    cout << "Config: " << name << endl;

    // Open .ini file
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(name.c_str(), pt);

    // Read map dimensions
    width = pt.get<float>("MapDimensions.width");
    height = pt.get<float>("MapDimensions.height");
    resolution = pt.get<float>("MapDimensions.resolution");


    // Read mines information
    numMines = pt.get<int>("Mines.nummines");
    randomMines = pt.get<bool>("Mines.randommines");
    detectionMinDist = pt.get<float>("Mines.detectionmindist");
    explosionMaxDist = pt.get<float>("Mines.explosionmaxdist");

    if(randomMines == false){
        // Read mines positions
        string input = pt.get<string>("Mines.minespositions");
        string delimiters("|,:");
        vector<string> parts;
        boost::split(parts, input, boost::is_any_of(delimiters));
        for(int i=0; i<parts.size(); i+=2)
            minesPositions.push_back(Position2D(atof(parts[i].c_str()),atof(parts[i+1].c_str())));

        cout << "Mines Positions ";
        for(int i=0; i<minesPositions.size(); i++){
            cout << minesPositions[i] << ' ';
        }
        cout << endl;
    }

}

