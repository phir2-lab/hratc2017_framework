/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Goncalo Cabrita on 21/04/2014
*
* This software publishes a static transform between the world frame
* and the minefield frame.
*
* The minefield frame is centered on the 4 GPS coordinates that define
* the minefield itself.
*
*   1 ------------------------------------------------------------ 4
*   |                                                              |
*   |                              y                               |
*   |                              ^                               |
*   |                              |                               |
*   |                              o-- > x                         |
*   |                            z                                 |
*   |                                                              |
*   |                                                              |
*   |                                                              |
*   2 ------------------------------------------------------------ 3
*
*********************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>

#include <math.h>

#define WSG84_A 6378137.0
#define E2 0.0066943799831668
#define Ee2  0.0067394967352076

// GPStoUTM() by Jorge Fraga
void GPStoUTM(const double lat, const double longi, double &northing, double &easting, int &n_zona, char &letra)
{


    if     ((84 >= lat) && (lat >= 72))  letra = 'X';
    else if ((72 > lat) && (lat >= 64))  letra = 'W';
    else if ((64 > lat) && (lat >= 56))  letra = 'V';
    else if ((56 > lat) && (lat >= 48))  letra = 'U';
    else if ((48 > lat) && (lat >= 40))  letra = 'T';
    else if ((40 > lat) && (lat >= 32))  letra = 'S';
    else if ((32 > lat) && (lat >= 24))  letra = 'R';
    else if ((24 > lat) && (lat >= 16))  letra = 'Q';
    else if ((16 > lat) && (lat >= 8))   letra = 'P';
    else if (( 8 > lat) && (lat >= 0))   letra = 'N';
    else if (( 0 > lat) && (lat >= -8))  letra = 'M';
    else if ((-8 > lat) && (lat >= -16)) letra = 'L';
    else if((-16 > lat) && (lat >= -24)) letra = 'K';
    else if((-24 > lat) && (lat >= -32)) letra = 'J';
    else if((-32 > lat) && (lat >= -40)) letra = 'H';
    else if((-40 > lat) && (lat >= -48)) letra = 'G';
    else if((-48 > lat) && (lat >= -56)) letra = 'F';
    else if((-56 > lat) && (lat >= -64)) letra = 'E';
    else if((-64 > lat) && (lat >= -72)) letra = 'D';
    else if((-72 > lat) && (lat >= -80)) letra = 'C';
    else letra = '-';

    n_zona = int((longi + 180)/6) + 1;

    //caso especial na noruega
    if( lat >= 56.0 && lat < 64.0 && longi >= 3.0 && longi < 12.0 )
        n_zona = 32;

        // caso especial de Svalbard
    if( lat >= 72.0 && lat < 84.0 )
    {
      if(      longi >= 0.0  && longi <  9.0 ) n_zona = 31;
      else if( longi >= 9.0  && longi < 21.0 ) n_zona = 33;
      else if( longi >= 21.0 && longi < 33.0 ) n_zona = 35;
      else if( longi >= 33.0 && longi < 42.0 ) n_zona = 37;
    }

    double lat_rad = lat * M_PI / 180.0;
    double longi_rad = longi * M_PI / 180;

    double origem_longi = (n_zona - 1)*6 - 180 + 3;
    double origem_longi_rad = origem_longi * M_PI / 180.0;

    double a = WSG84_A;
    double e_sq = E2;
    double e_sq2 = Ee2; // e'²
    double N = a / sqrt ( 1 - e_sq * (sin(lat_rad)*sin(lat_rad)));//N = a/sqrt(1-e_sq*(sin(lat_rad)^2));
    double T = tan(lat_rad)*tan(lat_rad); //T = tan(lat_rad)^2;
    double C = e_sq2*cos(lat_rad)*cos(lat_rad);
    double A = cos(lat_rad)*(longi_rad - origem_longi_rad);

    double M = a*((1-e_sq/4 - 3*e_sq*e_sq/64 - 5*e_sq*e_sq*e_sq)*lat_rad -
            (3*e_sq/8 + 3*e_sq*e_sq/32 + 45*e_sq*e_sq*e_sq/1024)*sin(2*lat_rad) +
            (15*e_sq*e_sq/256 + 45*e_sq*e_sq*e_sq/1024) *sin(4*lat_rad) +
            (35*e_sq*e_sq*e_sq/3072)*sin(6*lat_rad));

        double k0 = 0.9996;

        easting = k0*N*(A + (1-T+C)*A*A*A/6 + (5-18*T+T*T+72*C - 58*e_sq2)*A*A*A*A*A/120) + 500000;

        northing = k0*(M+ N*tan(lat_rad)*(A*A/2 + (5 - T + 9*C + 4*C*C)*A*A*A*A/24 +
    (61-58*T+T*T + 600*C - 330*e_sq2)*A*A*A*A*A*A/720));

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "minefield_static_tf_publisher");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    double coordinates[12];

    // Read the 4 corners of the minefield from the parameter server...
    if(!pn.getParam("minefield/corner1/lat", coordinates[0]) && !pn.getParam("minefield/corner1/long", coordinates[1]) && !pn.getParam("minefield/corner1/alt", coordinates[2]) &&
       !pn.getParam("minefield/corner2/lat", coordinates[3]) && !pn.getParam("minefield/corner2/long", coordinates[4]) && !pn.getParam("minefield/corner2/alt", coordinates[5]) &&
       !pn.getParam("minefield/corner3/lat", coordinates[6]) && !pn.getParam("minefield/corner3/long", coordinates[7]) && !pn.getParam("minefield/corner3/alt", coordinates[8]) &&
       !pn.getParam("minefield/corner4/lat", coordinates[9]) && !pn.getParam("minefield/corner4/long", coordinates[10]) && !pn.getParam("minefield/corner4/alt", coordinates[11]))
    {
        ROS_FATAL("Minefield static tf broadcaster -- Unable to start without the 4 corners of the minefield!!!");
        ROS_BREAK();
    }

    // Convert the GPS coordinates into UTM coordinates
    double easting, northing;
    char zone_letter;
    int zone_number;

    GPStoUTM(angles::from_degrees(coordinates[0]), angles::from_degrees(coordinates[1]), northing, easting, zone_number, zone_letter);
    tf::Vector3 corner1 = tf::Vector3(easting, northing, coordinates[2]);

    GPStoUTM(angles::from_degrees(coordinates[3]), angles::from_degrees(coordinates[4]), northing, easting, zone_number, zone_letter);
    tf::Vector3  corner2 = tf::Vector3(easting, northing, coordinates[5]);

    GPStoUTM(angles::from_degrees(coordinates[6]), angles::from_degrees(coordinates[7]), northing, easting, zone_number, zone_letter);
    tf::Vector3 corner3 = tf::Vector3(easting, northing, coordinates[8]);

    GPStoUTM(angles::from_degrees(coordinates[9]), angles::from_degrees(coordinates[10]), northing, easting, zone_number, zone_letter);
    tf::Vector3 corner4 = tf::Vector3(easting, northing, coordinates[11]);

    // Find the center of the minefield, this will be the origin of the minefield frame
    tf::Vector3 center = tf::Vector3((corner1.x() + corner2.x() + corner3.x() + corner4.x()) / 4.0, (corner1.y() + corner2.y() + corner3.y() + corner4.y()) / 4.0, (corner1.z() + corner2.z() + corner3.z() + corner4.z()) / 4.0);

    ROS_INFO("Minefield static tf broadcaster -- The center of the minefield is x:%lf y:%lf z:%lf (m)", center.x(), center.y(), center.z());

    double w =  ( sqrt(pow(corner4.x()-corner1.x(), 2) + pow(corner4.y()-corner1.y(), 2) + pow(corner4.z()-corner1.z(), 2)) * sqrt(pow(corner3.x()-corner2.x(), 2) + pow(corner3.y()-corner2.y(), 2) + pow(corner3.z()-corner2.z(), 2)) ) / 2.0;
    double h =  ( sqrt(pow(corner2.x()-corner1.x(), 2) + pow(corner2.y()-corner1.y(), 2) + pow(corner2.z()-corner1.z(), 2)) * sqrt(pow(corner4.x()-corner3.x(), 2) + pow(corner4.y()-corner3.y(), 2) + pow(corner4.z()-corner3.z(), 2)) ) / 2.0;

    ROS_INFO("Minefield static tf broadcaster -- The minefield is %lfx%lf (m)", w, h);

    double rate;
    pn.param("rate", rate, 20.0);

    std::string parent_frame;
    pn.param<std::string>("parent_frame", parent_frame, "world");

    std::string child_frame;
    pn.param<std::string>("child_frame", child_frame, "minefield");

    ROS_INFO("Minefield static tf broadcaster -- Publishing static transform from %s to %s at %lf hz...", parent_frame.c_str(), child_frame.c_str(), rate);

    // Generate the transformation from the world frame to the minefield frame
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(center.x(), center.y(), center.z()) );
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, atan2((corner4.y()+corner3.y())/2.0 - (corner2.y()+corner1.y())/2.0, (corner4.x()+corner3.x())/2.0 - (corner2.x()+corner1.x())/2.0));
    transform.setRotation(q);

    // Publish the transformation at the desired rate!
    ros::Rate loop(rate);
    while(ros::ok())
    {
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame, child_frame));
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
