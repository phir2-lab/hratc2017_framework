#include "judge.h"

#include <iostream>
#include <string>

using namespace std;

Judge::Judge()
{
    n = new ros::NodeHandle("~");

    string filename;
    if(n->getParam("config", filename)==false)
    {
        ROS_ERROR("Failed to get param 'config'");
    }
    config = new Config(filename);

    robotPose = new RobotPose("/minefield","/robot_pose_ekf/odom");

    sub_setMine = n->subscribe("/HRATC_FW/set_mine", 100, &Judge::checkMineDetection, this);
    sub_occupancyGrid = n->subscribe("/mineFieldViewer/occupancyGrid", 100, &Judge::checkUnresolvedMines, this);

    initializeMinesMarkers();
    initializeRobotPath();

    rate = new ros::Rate(20);
}

void Judge::run()
{
    while (ros::ok())
    {
        geometry_msgs::PoseStamped p = robotPose->getLocalPose();
        robotZ = p.pose.position.z;
//        cout << "Pose:" << p.pose.position.x << ' ' << p.pose.position.y << ' ' << tf::getYaw(p.pose.orientation) << endl;

        updateMinesMarkers();
        updateRobotPath();
        checkMineExplosion();

        ros::spinOnce();
        rate->sleep();
    }
}

void Judge::initializeMinesMarkers()
{
    // Initialize mines publishers
    pub_trueMinesMarker = n->advertise<visualization_msgs::MarkerArray>("trueMines_marker", 1);
    pub_properlyDetectedMinesMarker = n->advertise<visualization_msgs::MarkerArray>("properlyDetectedMines_marker", 1);
    pub_wronglyDetectedMinesMarker = n->advertise<visualization_msgs::MarkerArray>("wronglyDetectedMines_marker", 1);
    pub_knownExplodedMinesMarker = n->advertise<visualization_msgs::MarkerArray>("knownExplodedMines_marker", 1);
    pub_unknownExplodedMinesMarker = n->advertise<visualization_msgs::MarkerArray>("unknownExplodedMines_marker", 1);

    // Initialize true mines markers
    trueMines.markers.resize(config->numMines);
    cout << trueMines.markers.size() << endl;

    detected.resize(config->numMines, false);
    exploded.resize(config->numMines, false);

    for(int i=0; i<trueMines.markers.size(); i++){

        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        trueMines.markers[i].header.frame_id = "minefield";
        trueMines.markers[i].header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        trueMines.markers[i].ns = "trueMines";
        trueMines.markers[i].id = i;

        // Set the marker type.
        trueMines.markers[i].type = visualization_msgs::Marker::CYLINDER;

        // Set the marker action.  Options are ADD and DELETE
        trueMines.markers[i].action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        trueMines.markers[i].pose.position.x = config->minesPositions[i].x;
        trueMines.markers[i].pose.position.y = config->minesPositions[i].y;
        trueMines.markers[i].pose.position.z = 0;
        trueMines.markers[i].pose.orientation.x = 0.0;
        trueMines.markers[i].pose.orientation.y = 0.0;
        trueMines.markers[i].pose.orientation.z = 0.0;
        trueMines.markers[i].pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        trueMines.markers[i].scale.x = 0.2;
        trueMines.markers[i].scale.y = 0.2;
        trueMines.markers[i].scale.z = 0.01;

        // Set the color -- BLUE!
        trueMines.markers[i].color.r = 0.0f;
        trueMines.markers[i].color.g = 0.4f;
        trueMines.markers[i].color.b = 1.0f;
        trueMines.markers[i].color.a = 1.0;

        trueMines.markers[i].lifetime = ros::Duration();
    }
}

void Judge::addMineMarker(mineType mtype, Position2D pos)
{
    visualization_msgs::MarkerArray *array;

    visualization_msgs::Marker mine;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    mine.header.frame_id = "minefield";
    mine.header.stamp = ros::Time::now();

    // Set the marker type.
    mine.type = visualization_msgs::Marker::CYLINDER;

    // Set the marker action.  Options are ADD and DELETE
    mine.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    mine.pose.position.x = pos.x;
    mine.pose.position.y = pos.y;
    mine.pose.position.z = 0;
    mine.pose.orientation.x = 0.0;
    mine.pose.orientation.y = 0.0;
    mine.pose.orientation.z = 0.0;
    mine.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    mine.scale.x = 0.2;
    mine.scale.y = 0.2;
    mine.scale.z = 0.01;

    mine.lifetime = ros::Duration();

    switch(mtype){
        case PROPERLY_DETECTED:
            array = &properlyDetectedMines;
            // Set the namespace.
            mine.ns = "PDMines";
            // Set the color -- GREEN!
            mine.color.r = 0.0f;
            mine.color.g = 0.7f;
            mine.color.b = 0.0f;
            mine.color.a = 1.0;
            break;
        case WRONGLY_DETECTED:
            array = &wronglyDetectedMines;
            // Set the namespace.
            mine.ns = "WDMines";
            // Set the color -- MAGENTA!
            mine.color.r = 0.701f;
            mine.color.g = 0.0f;
            mine.color.b = 0.909f;
            mine.color.a = 1.0;
            break;
        case KNOWN_EXPLODED:
            array = &knownExplodedMines;
            // Set the namespace.
            mine.ns = "KEMines";
            // Set the color -- RED!
            mine.color.r = 1.0f;
            mine.color.g = 0.0f;
            mine.color.b = 0.0f;
            mine.color.a = 1.0;
            break;
        case UNKNOWN_EXPLODED:
            array = &unknownExplodedMines;
            // Set the namespace.
            mine.ns = "UEMines";
            // Set the color -- ORANGE!
            mine.color.r = 1.0f;
            mine.color.g = 0.3f;
            mine.color.b = 0.0f;
            mine.color.a = 1.0;
    }

    // Set the id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    mine.id = array->markers.size();

    array->markers.push_back(mine);
}

void Judge::updateMinesMarkers()
{
//    cout << "Publishing mines markers" << endl;

    for(int i=0; i<trueMines.markers.size(); i++){
        trueMines.markers[i].header.stamp = ros::Time::now();
        trueMines.markers[i].pose.position.z = robotZ-0.2;
    }

    for(int i=0; i<properlyDetectedMines.markers.size(); i++){
        properlyDetectedMines.markers[i].header.stamp = ros::Time::now();
        properlyDetectedMines.markers[i].pose.position.z = robotZ-0.1;
    }

    for(int i=0; i<wronglyDetectedMines.markers.size(); i++){
        wronglyDetectedMines.markers[i].header.stamp = ros::Time::now();
        wronglyDetectedMines.markers[i].pose.position.z = robotZ-0.1;
    }

    for(int i=0; i<knownExplodedMines.markers.size(); i++){
        knownExplodedMines.markers[i].header.stamp = ros::Time::now();
        knownExplodedMines.markers[i].pose.position.z = robotZ-0.1;
    }

    for(int i=0; i<unknownExplodedMines.markers.size(); i++){
        unknownExplodedMines.markers[i].header.stamp = ros::Time::now();
        unknownExplodedMines.markers[i].pose.position.z = robotZ-0.1;
    }

    pub_trueMinesMarker.publish(trueMines);
    pub_properlyDetectedMinesMarker.publish(properlyDetectedMines);
    pub_wronglyDetectedMinesMarker.publish(wronglyDetectedMines);
    pub_knownExplodedMinesMarker.publish(knownExplodedMines);
    pub_unknownExplodedMinesMarker.publish(unknownExplodedMines);
}

void Judge::checkMineDetection(const geometry_msgs::PoseStamped::ConstPtr & guess)
{
    cout << "Received mine guess" << endl;
    float px = guess->pose.position.x;
    float py = guess->pose.position.y;
    cout << px << ' ' << py << endl;

    // Find distances to all mines
    vector<float> dists(trueMines.markers.size());
    bool missed = true;

    for(int i=0; i<trueMines.markers.size(); i++){
        dists[i] = sqrt(pow(trueMines.markers[i].pose.position.x-px,2.0)+
                        pow(trueMines.markers[i].pose.position.y-py,2.0));

        if(dists[i] <= config->detectionMinDist){
            missed = false;
            if(detected[i] == false){
                detected[i] = true;
                addMineMarker(PROPERLY_DETECTED,Position2D(trueMines.markers[i].pose.position.x,trueMines.markers[i].pose.position.y));
            }
        }
    }

    if(missed)
        addMineMarker(WRONGLY_DETECTED,Position2D(px,py));

}

void Judge::checkMineExplosion()
{
    vector<geometry_msgs::PoseStamped> wheelPoses;

    wheelPoses = robotPose->getWheelsPoses();
//    cout << "wheelPoses size:" << wheelPoses.size() << endl;

    // For each wheel
    for(int i=0; i<wheelPoses.size(); i++){
        float px = wheelPoses[i].pose.position.x;
        float py = wheelPoses[i].pose.position.y;
//        cout << px << ' ' << py << endl;

        // Find distances to all mines
        vector<float> dists(trueMines.markers.size());

        for(int i=0; i<trueMines.markers.size(); i++){
            dists[i] = sqrt(pow(trueMines.markers[i].pose.position.x-px,2.0)+
                            pow(trueMines.markers[i].pose.position.y-py,2.0));

            if(dists[i] <= config->explosionMaxDist){
                if(exploded[i] == false){
                    exploded[i] = true;
                    unresolved.push_back(i);
                }
            }
        }
    }
}

void Judge::checkUnresolvedMines(const nav_msgs::OccupancyGrid::ConstPtr & grid)
{
    if(!unresolved.empty()){
        int w = grid->info.width;
        int h = grid->info.height;
        cout << w << ' ' << h << endl;
        for(int i=0; i<unresolved.size();i++){
            // check if region containing mine was visited
            cout << grid->info.origin.position.x << ' ' << grid->info.origin.position.y << endl;

            int x = (-grid->info.origin.position.x + trueMines.markers[unresolved[i]].pose.position.x)/grid->info.resolution;
            int y = (-grid->info.origin.position.y + trueMines.markers[unresolved[i]].pose.position.y)/grid->info.resolution;
            int occ = grid->data[x + y*w];

            cout << "x:" << x << " y:" << y << " occ:" << occ << endl;

            if(occ==0){ // visited
                cout << "Exploded Known Mine!!!!" << endl;
                addMineMarker(KNOWN_EXPLODED,Position2D(trueMines.markers[unresolved[i]].pose.position.x,trueMines.markers[unresolved[i]].pose.position.y));
            }else{
                cout << "Exploded Unknown Mine!!!!" << endl;
                addMineMarker(UNKNOWN_EXPLODED,Position2D(trueMines.markers[unresolved[i]].pose.position.x,trueMines.markers[unresolved[i]].pose.position.y));
            }
        }

        unresolved.clear();
    }
}


void Judge::initializeRobotPath()
{
    pub_robotPath = n->advertise<visualization_msgs::Marker>("hratc_path",10);
    robotpath.header.frame_id = "minefield";
    robotpath.header.stamp = ros::Time::now();
    robotpath.ns =  "robotPath";
    robotpath.action = visualization_msgs::Marker::ADD;
    robotpath.pose.orientation.w  = 1.0;
    robotpath.type = visualization_msgs::Marker::LINE_STRIP;
    robotpath.scale.x = 0.1;
    robotpath.color.r = 0;
    robotpath.color.g = 0.0;
    robotpath.color.b = 0.5;
    robotpath.color.a = 1.0;
}

void Judge::updateRobotPath()
{
    geometry_msgs::Point p = robotPose->getLocalPose().pose.position;
    robotpath.points.push_back(p);
    pub_robotPath.publish(robotpath);
}
