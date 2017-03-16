#include "judge.h"

#include <iostream>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <pwd.h>
#include <unistd.h>

using namespace std;

Judge::Judge()
{

    n = new ros::NodeHandle("~");
    canStart = false;

    rate = new ros::Rate(20);

    sub_configDone = n->subscribe("/configDone", 100, &Judge::checkStart, this);

    // Check if it is simulation, so it must wait the generation of the mines map
    bool isSimulation;
    if(n->getParam("isSimulation", isSimulation)==false)
    {
        canStart = true;
    }else{
        if(isSimulation)
            canStart = false;
        else
            canStart = true;
    }

    cout << "Waiting to start!" << endl;
    while (canStart == false)
    {
        ros::spinOnce();
        rate->sleep();
    }
    cout << "Done" << endl; cout.flush();

//    string filename;
//    if(n->getParam("config", filename)==false)
//    {
//        ROS_ERROR("Failed to get param 'config'");
//    }
//    config = new Config(filename);
    config = new Config(n);

    trueRobotPose = new RobotPose("/minefield","/robot_pose_ekf/odom");
    //trueRobotPose = new TrueRobotPose(n);

    sub_setMine = n->subscribe("/HRATC_FW/set_mine", 100, &Judge::checkMineDetection, this);
    sub_occupancyGrid = n->subscribe("/mineFieldViewer/occupancyGrid", 100, &Judge::checkUnresolvedMines, this);
    sub_coveredArea = n->subscribe("/mineFieldViewer/coverageRate", 100, &Judge::getCoverageRate, this);

    coverageRate = 0.0;

    initializeLogFile();
    initializeMinesMarkers();
    initializeScoreboard();
    initializeRobotPath();
    initializeTrueRobotMarker();

    start = ros::WallTime::now();
    last = start;
}

void Judge::checkStart(const std_msgs::Bool::ConstPtr &flag)
{
    if(flag->data == true)
        canStart = true;
}

void Judge::run()
{
    while (ros::ok())
    {
        geometry_msgs::PoseStamped p = trueRobotPose->getLocalPose();
        robotZ = p.pose.position.z;
//        cout << "Pose A:" << p.pose.position.x << ' ' << p.pose.position.y << ' ' << tf::getYaw(p.pose.orientation) << endl;

//        geometry_msgs::PoseStamped tp = trueRobotPose->getLocalPose();
//        cout << "Pose B:" << tp.pose.position.x << ' ' << tp.pose.position.y << ' ' << tf::getYaw(tp.pose.orientation) << endl;

        updateMinesMarkers();
        updateScoreboard();
        updateRobotPath();
        updateTrueRobotMarker();
        checkMineExplosion();

        current = ros::WallTime::now();
        if(current-last > ros::WallDuration(5.0)){
            saveLog();
//            last = current;
            last = last+ros::WallDuration(5.0);
        }

        ros::spinOnce();
        rate->sleep();
    }
}

void Judge::initializeLogFile()
{
    const char *homedir;
    if ((homedir = getenv("HOME")) == NULL)
        homedir = getpwuid(getuid())->pw_dir;

    string path = string(homedir) + string("/HRATC");

    struct stat st = {0};
    if (stat(path.c_str(), &st) == -1)
        mkdir(path.c_str(), 0700);

    path = path+string("/log.txt");

    logFile.open(path.c_str(), fstream::out);
    if (!logFile.is_open())
        std::cout << "Error opening file";
}

void Judge::initializeMinesMarkers()
{
    // Initialize mines publishers
    pub_trueMinesMarker = n->advertise<visualization_msgs::MarkerArray>("trueMines_marker", 1);
    pub_properlyDetectedMinesMarker = n->advertise<visualization_msgs::MarkerArray>("properlyDetectedMines_marker", 1);
    pub_wronglyDetectedMinesMarker = n->advertise<visualization_msgs::MarkerArray>("wronglyDetectedMines_marker", 1);
    pub_knownExplodedMinesMarker = n->advertise<visualization_msgs::MarkerArray>("knownExplodedMines_marker", 1);
    pub_unknownExplodedMinesMarker = n->advertise<visualization_msgs::MarkerArray>("unknownExplodedMines_marker", 1);
    pub_visitedUndetectedMinesMarker = n->advertise<visualization_msgs::MarkerArray>("visitedUndetectedMines_marker", 1);
    pub_notVisitedUndetectedMinesMarker = n->advertise<visualization_msgs::MarkerArray>("notVisitedUndetectedMines_marker", 1);

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
            break;
        case VISITED_UNDETECTED:
            array = &visitedUndetectedMines;
            // Set the namespace.
            mine.ns = "VUMines";
            // Set the color -- YELLOW!
            mine.color.r = 1.0f;
            mine.color.g = 1.0f;
            mine.color.b = 0.0f;
            mine.color.a = 1.0;
            break;
        case NOTVISITED_UNDETECTED:
            array = &notVisitedUndetectedMines;
            // Set the namespace.
            mine.ns = "NUMines";
            // Set the color -- DARK YELLOW!
            mine.color.r = 0.4f;
            mine.color.g = 0.4f;
            mine.color.b = 0.0f;
            mine.color.a = 1.0;
            break;
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
        trueMines.markers[i].pose.position.z = robotZ-0.4;
    }

    for(int i=0; i<properlyDetectedMines.markers.size(); i++){
        properlyDetectedMines.markers[i].header.stamp = ros::Time::now();
        properlyDetectedMines.markers[i].pose.position.z = robotZ-0.2;
    }

    for(int i=0; i<wronglyDetectedMines.markers.size(); i++){
        wronglyDetectedMines.markers[i].header.stamp = ros::Time::now();
        wronglyDetectedMines.markers[i].pose.position.z = robotZ-0.2;
    }

    for(int i=0; i<knownExplodedMines.markers.size(); i++){
        knownExplodedMines.markers[i].header.stamp = ros::Time::now();
        knownExplodedMines.markers[i].pose.position.z = robotZ-0.1;
    }

    for(int i=0; i<unknownExplodedMines.markers.size(); i++){
        unknownExplodedMines.markers[i].header.stamp = ros::Time::now();
        unknownExplodedMines.markers[i].pose.position.z = robotZ-0.1;
    }

    for(int i=0; i<visitedUndetectedMines.markers.size(); i++){
        visitedUndetectedMines.markers[i].header.stamp = ros::Time::now();
        visitedUndetectedMines.markers[i].pose.position.z = robotZ-0.3;
    }

    for(int i=0; i<notVisitedUndetectedMines.markers.size(); i++){
        notVisitedUndetectedMines.markers[i].header.stamp = ros::Time::now();
        notVisitedUndetectedMines.markers[i].pose.position.z = robotZ-0.35;
    }

    pub_trueMinesMarker.publish(trueMines);
    pub_properlyDetectedMinesMarker.publish(properlyDetectedMines);
    pub_wronglyDetectedMinesMarker.publish(wronglyDetectedMines);
    pub_knownExplodedMinesMarker.publish(knownExplodedMines);
    pub_unknownExplodedMinesMarker.publish(unknownExplodedMines);
    pub_visitedUndetectedMinesMarker.publish(visitedUndetectedMines);
    pub_notVisitedUndetectedMinesMarker.publish(notVisitedUndetectedMines);
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
            if(detected[i] == false && exploded[i] == false){
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

//    wheelPoses = robotPose->getWheelsPoses();
    wheelPoses = trueRobotPose->getWheelsPoses();

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
//            cout << grid->info.origin.position.x << ' ' << grid->info.origin.position.y << endl;

//            int x = (-grid->info.origin.position.x + trueMines.markers[unresolved[i]].pose.position.x)/grid->info.resolution;
//            int y = (-grid->info.origin.position.y + trueMines.markers[unresolved[i]].pose.position.y)/grid->info.resolution;
//            int occ = grid->data[x + y*w];

//            cout << "x:" << x << " y:" << y << " occ:" << occ << endl;
            if(detected[unresolved[i]]){ // detected
//            if(occ==0){ // visited
                cout << "Exploded Known Mine!!!!" << endl;
                addMineMarker(KNOWN_EXPLODED,Position2D(trueMines.markers[unresolved[i]].pose.position.x,trueMines.markers[unresolved[i]].pose.position.y));
            }else{
                cout << "Exploded Unknown Mine!!!!" << endl;
                addMineMarker(UNKNOWN_EXPLODED,Position2D(trueMines.markers[unresolved[i]].pose.position.x,trueMines.markers[unresolved[i]].pose.position.y));
            }
        }

        unresolved.clear();
    }

    int w = grid->info.width;
    int h = grid->info.height;

    // check undetected
    visitedUndetectedMines.markers.clear();
    notVisitedUndetectedMines.markers.clear();

    for(int i=0; i<detected.size();i++){
        if(detected[i]==false){
            int x = (-grid->info.origin.position.x + trueMines.markers[i].pose.position.x)/grid->info.resolution;
            int y = (-grid->info.origin.position.y + trueMines.markers[i].pose.position.y)/grid->info.resolution;
            int occ = grid->data[x + y*w];

            if(occ==0){ // visited
                addMineMarker(VISITED_UNDETECTED,Position2D(trueMines.markers[i].pose.position.x,trueMines.markers[i].pose.position.y));
            }else{
                addMineMarker(NOTVISITED_UNDETECTED,Position2D(trueMines.markers[i].pose.position.x,trueMines.markers[i].pose.position.y));
            }
        }
    }
}

void Judge::getCoverageRate(const std_msgs::Float32::ConstPtr &rate)
{
    coverageRate = rate->data;
}

void Judge::initializeScoreboard()
{
    // Initialize scoreboard publishers
    pub_textProperlyDetectedMines = n->advertise<visualization_msgs::Marker>("scoreboard_properlyDetectedMines", 1);
    pub_textWronglyDetectedMines = n->advertise<visualization_msgs::Marker>("scoreboard_wronglyDetectedMines", 1);
    pub_textKnownExplodedMines = n->advertise<visualization_msgs::Marker>("scoreboard_knownExplodedMines", 1);
    pub_textUnknownExplodedMines = n->advertise<visualization_msgs::Marker>("scoreboard_unknownExplodedMines", 1);
    pub_textVisitedUndetectedMines = n->advertise<visualization_msgs::Marker>("scoreboard_visitedUndetectedMines", 1);
    pub_textNotVisitedUndetectedMines = n->advertise<visualization_msgs::Marker>("scoreboard_notVisitedUndetectedMines", 1);
    pub_textElapsedTime = n->advertise<visualization_msgs::Marker>("scoreboard_elapsedTime", 1);
    pub_textCoverage = n->advertise<visualization_msgs::Marker>("scoreboard_coverage", 1);

    // Initialize default text marker
    visualization_msgs::Marker tempMarker;
    tempMarker.header.frame_id = "minefield";
    tempMarker.header.stamp = ros::Time::now();
    tempMarker.id = 0;
    tempMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    tempMarker.action = visualization_msgs::Marker::ADD;
//    tempMarker.pose.position.x = 0.0;
//    tempMarker.pose.position.y = config->height/2.0;
    tempMarker.pose.position.x = (config->lowerBound.x()+config->upperBound.x())/2.0;
    tempMarker.pose.position.y = config->upperBound.y();
    tempMarker.pose.position.z = 0.0;
    tempMarker.pose.orientation.x = 0.0;
    tempMarker.pose.orientation.y = 0.0;
    tempMarker.pose.orientation.z = 0.0;
    tempMarker.pose.orientation.w = 1.0;
    tempMarker.color.a = 1.0;
    tempMarker.scale.x = 1.0;
    tempMarker.scale.y = 1.0;
    tempMarker.scale.z = 0.6;
    tempMarker.lifetime = ros::Duration();

    textElapsedTime = tempMarker;
    textProperlyDetectedMines = tempMarker;
    textWronglyDetectedMines = tempMarker;
    textKnownExplodedMines = tempMarker;
    textUnknownExplodedMines = tempMarker;
    textVisitedUndetectedMines = tempMarker;
    textNotVisitedUndetectedMines = tempMarker;
    textCoverage = tempMarker;

    // Initialize elapsed mine texts -- GREEN!
    textElapsedTime.ns = "scoreboard_ElapsedTime";
    textElapsedTime.color.r = 0.0f;
    textElapsedTime.color.g = 0.0f;
    textElapsedTime.color.b = 0.0f;
//    textElapsedTime.pose.position.x -= 3.0;
    textElapsedTime.pose.position.y = config->lowerBound.y() - 1.2;

    // Initialize coverage texts -- GREEN!
    textCoverage.color.r = 0.0f;
    textCoverage.color.g = 0.0f;
    textCoverage.color.b = 0.0f;
//    textCoverage.pose.position.x -= 3.0;
    textCoverage.pose.position.y = config->lowerBound.y() - 2.0;

    // PD --- WD
    // VU --- NU
    // KE --- UE

    // Initialize properly detected mines texts -- GREEN!
    textProperlyDetectedMines.ns = "scoreboard_PDMines";
    textProperlyDetectedMines.color.r = 0.0f;
    textProperlyDetectedMines.color.g = 0.7f;
    textProperlyDetectedMines.color.b = 0.0f;
    textProperlyDetectedMines.pose.position.x -= 3.0;
    textProperlyDetectedMines.pose.position.y += 1.8;

    // Initialize wrongly detected mines texts -- MAGENTA!
    textWronglyDetectedMines.ns = "scoreboard_WDMines";
    textWronglyDetectedMines.color.r = 0.701f;
    textWronglyDetectedMines.color.g = 0.0f;
    textWronglyDetectedMines.color.b = 0.909f;
    textWronglyDetectedMines.pose.position.x += 3.0;
    textWronglyDetectedMines.pose.position.y += 1.8;

    // Initialize visited undetected mines texts -- YELLOW!
    textVisitedUndetectedMines.ns = "scoreboard_VUMines";
    textVisitedUndetectedMines.color.r = 1.0f;
    textVisitedUndetectedMines.color.g = 1.0f;
    textVisitedUndetectedMines.color.b = 0.0f;
    textVisitedUndetectedMines.pose.position.x -= 3.0;
    textVisitedUndetectedMines.pose.position.y += 1.2;

    // Initialize not visited undetected mines texts -- DARK YELLOW!
    textNotVisitedUndetectedMines.ns = "scoreboard_NUMines";
    textNotVisitedUndetectedMines.color.r = 0.6f;
    textNotVisitedUndetectedMines.color.g = 0.6f;
    textNotVisitedUndetectedMines.color.b = 0.0f;
    textNotVisitedUndetectedMines.pose.position.x += 3.0;
    textNotVisitedUndetectedMines.pose.position.y += 1.2;

    // Initialize known exploded mines texts -- RED!
    textKnownExplodedMines.ns = "scoreboard_KEMines";
    textKnownExplodedMines.color.r = 1.0f;
    textKnownExplodedMines.color.g = 0.0f;
    textKnownExplodedMines.color.b = 0.0f;
    textKnownExplodedMines.pose.position.x -= 3.0;
    textKnownExplodedMines.pose.position.y += 0.6;

    // Initialize unknown exploded mines texts -- ORANGE!
    textUnknownExplodedMines.ns = "scoreboard_UEMines";
    textUnknownExplodedMines.color.r = 1.0f;
    textUnknownExplodedMines.color.g = 0.3f;
    textUnknownExplodedMines.color.b = 0.0f;
    textUnknownExplodedMines.pose.position.x += 3.0;
    textUnknownExplodedMines.pose.position.y += 0.6;
}

void Judge::updateScoreboard()
{
    std::stringstream ss;

    textElapsedTime.header.stamp = ros::Time::now();
    int elapsed = (current-start).toSec();
    int secs = elapsed%60;
    int mins = elapsed/60;
    ss << "Time: " << setfill('0') << setw(2) << mins << ':' << setfill('0') << setw(2) << secs;
    textElapsedTime.text=ss.str();
    ss.str("");

    textCoverage.header.stamp = ros::Time::now();
    ss << "Coverage: " << setfill('0') << setw(3) << coverageRate*100.0 << "%";
    textCoverage.text=ss.str();
    pub_textCoverage.publish(textCoverage);
    ss.str("");

    textProperlyDetectedMines.header.stamp = ros::Time::now();
    ss << "Properly Detected: " << properlyDetectedMines.markers.size();
    textProperlyDetectedMines.text=ss.str();
    ss.str("");

    textWronglyDetectedMines.header.stamp = ros::Time::now();
    ss << "Wrongly Detected: " << wronglyDetectedMines.markers.size();
    textWronglyDetectedMines.text=ss.str();
    ss.str("");

    textKnownExplodedMines.header.stamp = ros::Time::now();
    ss << "Known Exploded: " << knownExplodedMines.markers.size();
    textKnownExplodedMines.text=ss.str();
    ss.str("");

    textUnknownExplodedMines.header.stamp = ros::Time::now();
    ss << "Unknown Exploded: " << unknownExplodedMines.markers.size();
    textUnknownExplodedMines.text=ss.str();
    ss.str("");

    textVisitedUndetectedMines.header.stamp = ros::Time::now();
    ss << "Visited Undetected: " << visitedUndetectedMines.markers.size();
    textVisitedUndetectedMines.text=ss.str();
    ss.str("");

    textNotVisitedUndetectedMines.header.stamp = ros::Time::now();
    ss << "Not Visited Undetected: " << notVisitedUndetectedMines.markers.size();
    textNotVisitedUndetectedMines.text=ss.str();
    ss.str("");

    pub_textElapsedTime.publish(textElapsedTime);
    pub_textCoverage.publish(textCoverage);
    pub_textProperlyDetectedMines.publish(textProperlyDetectedMines);
    pub_textWronglyDetectedMines.publish(textWronglyDetectedMines);
    pub_textKnownExplodedMines.publish(textKnownExplodedMines);
    pub_textUnknownExplodedMines.publish(textUnknownExplodedMines);
    pub_textVisitedUndetectedMines.publish(textVisitedUndetectedMines);
    pub_textNotVisitedUndetectedMines.publish(textNotVisitedUndetectedMines);
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

void Judge::initializeTrueRobotMarker()
{
    pub_trueRobotMarker = n->advertise<visualization_msgs::Marker>("true_robot_marker",10);
    trueRobotMarker.header.frame_id = "/minefield";
    trueRobotMarker.header.stamp = ros::Time::now();
    trueRobotMarker.ns =  "true_robot_marker";
    trueRobotMarker.action = visualization_msgs::Marker::ADD;
    trueRobotMarker.pose.orientation.w  = 1.0;
    trueRobotMarker.type = visualization_msgs::Marker::ARROW;
    trueRobotMarker.scale.x = 0.6;
    trueRobotMarker.scale.y = 0.2;
    trueRobotMarker.scale.z = 0.2;
    trueRobotMarker.color.r = 0.0;
    trueRobotMarker.color.g = 0.6;
    trueRobotMarker.color.b = 0.0;
    trueRobotMarker.color.a = 1.0;
}

void Judge::updateRobotPath()
{
//    geometry_msgs::Point p = robotPose->getLocalPose().pose.position;
    geometry_msgs::Point p = trueRobotPose->getLocalPose().pose.position;
    robotpath.points.push_back(p);
    pub_robotPath.publish(robotpath);
}

void Judge::updateTrueRobotMarker()
{
    geometry_msgs::Pose prevPose = trueRobotMarker.pose;
    geometry_msgs::Pose nextPose = trueRobotPose->getLocalPose().pose;
    trueRobotMarker.pose.position.x = (prevPose.position.x + nextPose.position.x)/2.0;
    trueRobotMarker.pose.position.y = (prevPose.position.y + nextPose.position.y)/2.0;
    trueRobotMarker.pose.position.z = (prevPose.position.z + nextPose.position.z)/2.0;
    trueRobotMarker.pose.position.z += 1.0;
    trueRobotMarker.pose.orientation = nextPose.orientation;
    pub_trueRobotMarker.publish(trueRobotMarker);
}

void Judge::saveLog()
{
    int elapsed = (current-start).toSec();
    int secs = elapsed%60;
    int mins = elapsed/60;
//    cout << "Time: " << setfill('0') << setw(2) << mins << ':' << setfill('0') << setw(2) << secs << endl;

    logFile << "Time: " << setfill('0') << setw(2) << mins << ':' << setfill('0') << setw(2) << secs << endl;
    logFile << "properlyDetectedMines: " << properlyDetectedMines.markers.size() << endl;
    logFile << "wronglyDetectedMines: " << wronglyDetectedMines.markers.size() << endl;
    logFile << "knownExplodedMines: " << knownExplodedMines.markers.size() << endl;
    logFile << "unknownExplodedMines: " << unknownExplodedMines.markers.size() << endl;
    logFile << "visitedUndetectedMines: " << visitedUndetectedMines.markers.size() << endl;
    logFile << "notVisitedUndetectedMines: " << notVisitedUndetectedMines.markers.size() << endl;
    logFile << "coverageRate: " << coverageRate << endl;
    logFile << endl;
}
