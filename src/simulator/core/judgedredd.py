# -*- coding:utf8 -*-
from PyQt4 import QtGui, QtCore
from numpy import *
import time, rospy, tf, numpy as np, random
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist
from metal_detector_msgs.msg._Coil import Coil
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep


def distance(pt1,pt2):
    return sqrt(pow(pt2[0]-pt1[0],2) + pow(pt2[1]-pt1[1],2))


class JudgeDredd(QtCore.QThread):
    path = []
    lastCoilPose = None
    listener = None
    mines = []
    minesDetected = {}
    minesWrong = []

    isSimulation = True
    receivedRobotPos = QtCore.pyqtSignal(list)
    receivedMinePos = QtCore.pyqtSignal(dict)
    receivedMineWrongPos = QtCore.pyqtSignal(list)
    receivedMineExplodedPos = QtCore.pyqtSignal(list)
    emitCoilSignal = QtCore.pyqtSignal(list)
    emitMap = QtCore.pyqtSignal(list)


    def __init__(self, parent, config):
        QtCore.QThread.__init__(self)
        self.config = config
        self.parent = parent
        self.updateConfig(config)
        self.pubMineDetection_left = self.pubMineDetection_middle = self.pubMineDetection_right = self.pubPose = None


    def __del__(self):
        self.stop()


    def stop(self):
        rospy.signal_shutdown("Shutdown HRATC Framework")
        self.wait()

    
    def updateConfig(self,config):
        self.cellXSize = config.cellWidth
        self.cellYSize = config.cellHeight
        self.width = config.mapWidth
        self.height = config.mapHeight

        self.mines = config.mines

        self.minDistDetection = config.minDistDetection
        self.maxDistExplosion = config.maxDistExplosion

        self.mineMap, self.zeroChannel = config.mineMap, config.zeroChannel

        self.resetScore()


    def resetScore(self):
        self.path = []
        self.lastCoilPose = None
        self.minesWrong = []
        self.minesDetected = {}
        self.minesExploded = []
        mWidth, mHeight = self.width/self.cellXSize,self.height/self.cellYSize
        self.map = np.ones((mWidth, mHeight))*220

        self.emitMap.emit([self.mineMap, self.map,[]])
        self.receivedMineWrongPos.emit(self.minesWrong)
        self.receivedMinePos.emit(self.minesDetected)
        self.receivedMineExplodedPos.emit(self.minesExploded)
        self.receivedRobotPos.emit(self.path)


    def receiveSimulationPose(self, data):
        idx = -1

        for n in data.name:
            if n.find("mobile") != -1:
                idx = data.name.index(n)

        if idx != -1:
            q = [data.pose[idx].orientation.x, data.pose[idx].orientation.y, data.pose[idx].orientation.z, data.pose[idx].orientation.w]
            roll, pitch, yaw =  euler_from_quaternion(q)

            pose = [data.pose[idx].position.x,data.pose[idx].position.y, yaw]
            self.updateRobotPose(pose)


    def updateRobotPose(self,newPose):
        robotX, robotY, yaw = newPose
        if self.listener == None:
            return

        # Lookup for the coils using tf
        try:
            (trans,rot) = self.listener.lookupTransform('base_footprint', 'left_coil', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        leftCoilX = trans[0]*cos(yaw) - trans[1]*sin(yaw)
        leftCoilY = trans[0]*sin(yaw) + trans[1]*cos(yaw)

        try:
            (trans,rot) = self.listener.lookupTransform('base_footprint', 'middle_coil', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          return
        middleCoilX = trans[0]*cos(yaw) - trans[1]*sin(yaw)
        middleCoilY = trans[0]*sin(yaw) + trans[1]*cos(yaw)

        try:
            (trans,rot) = self.listener.lookupTransform('base_footprint', 'right_coil', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        rightCoilX = trans[0]*cos(yaw) - trans[1]*sin(yaw)
        rightCoilY = trans[0]*sin(yaw) + trans[1]*cos(yaw)

        actualCoilPose = [middleCoilX, middleCoilY]
        x,y = robotX,robotY
        lastPose = None
        if len(self.path)>0:
            lastPose = self.path[-1]
        radRange = deg2rad(10)

        if self.lastCoilPose == None or distance(self.lastCoilPose,actualCoilPose) > self.cellXSize*2 or (lastPose != None and (distance(lastPose,newPose) > 0.1 or not -radRange < abs(yaw-lastPose[2]) < radRange)):
            self.lastCoilPose = actualCoilPose
            y, x = ( int((self.height/2 - y)/self.cellYSize), int((self.width/2 + x)/self.cellXSize) )

            coils = [[x+middleCoilX/self.cellXSize,y-middleCoilY/self.cellXSize]]
            coils.append([x+leftCoilX/self.cellXSize,y-leftCoilY/self.cellXSize])
            coils.append([x+rightCoilX/self.cellXSize,y-rightCoilY/self.cellXSize])

            coilsPose = [[leftCoilX,leftCoilY],[middleCoilX,middleCoilY],[rightCoilX,rightCoilY]]
            sensorArea = round(.18/self.cellXSize)
            radius = sensorArea #meters
            y1,x1 = np.ogrid[-radius:radius, -radius: radius]
            mask = x1**2+y1**2 <= radius**2

            topics = [self.pubMineDetection_left, self.pubMineDetection_middle, self.pubMineDetection_right]
            co = 0

            signals = []
            for x,y in coils:
                if x+radius <= self.map.shape[1] and x-radius >=0 and y+radius <= self.map.shape[0] and y-radius >= 0:
                    self.map[y-radius:y+radius,x-radius:x+radius][mask] = 183

                coil = Coil()
                for ch in range(3):
                    coil.channel.append(self.mineMap[3*co+ch,y,x] + random.random()*100)
                    coil.zero.append(self.zeroChannel[3*co+ch])

                if topics[co] != None:
                    topics[co].publish(coil)
                signals.append(coil)
                co += 1

            self.emitCoilSignal.emit(signals)

            self.emitMap.emit([self.mineMap,self.map,coilsPose])

        if lastPose == None or distance(lastPose,newPose) > 0.1 or not -radRange < abs(yaw-lastPose[2]) < radRange:
            self.path.append(newPose)
            self.receivedRobotPos.emit(self.path)
            if self.pubPose != None:
                pose = Pose()
                pose.position.x, pose.position.y = robotX, robotY
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion_from_euler(0.,0.,yaw)
                self.pubPose.publish(pose)

            wheels = [[robotX+.272*cos(yaw)+.2725*sin(yaw), robotY-.272*sin(yaw)+.2725*cos(yaw)],
                        [robotX-.272*cos(yaw)+.2725*sin(yaw), robotY+.272*sin(yaw)+.2725*cos(yaw)],
                        [robotX-.272*cos(yaw)-.2725*sin(yaw), robotY+.272*sin(yaw)-.2725*cos(yaw)],
                        [robotX+.272*cos(yaw)-.2725*sin(yaw), robotY-.272*sin(yaw)-.2725*cos(yaw)]]
            for mine in self.mines:
                for wheel in wheels:
                    if distance(wheel,mine) < self.maxDistExplosion:
                        if not mine in self.minesExploded:
                            self.minesExploded.append(mine)
                            self.receivedMineExplodedPos.emit(self.minesExploded)


    def receiveMinePosition(self,data):

            q = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
            roll, pitch, yaw =  euler_from_quaternion(q)
            mine = [data.position.x, data.position.y]
            m = tuple(min(self.mines,key=lambda m: distance(m,mine)))

            if distance(m,mine) < self.minDistDetection:
                if (self.minesDetected.has_key(m)):
                    if distance(m, self.minesDetected[m]) > distance(m, mine):
                        mine, self.minesDetected[m] = self.minesDetected[m], mine
                        self.receivedMinePos.emit(self.minesDetected)

                    self.minesWrong.append(mine)
                    self.receivedMineWrongPos.emit(self.minesWrong)
                else:
                    self.minesDetected[m] = mine
                    self.receivedMinePos.emit(self.minesDetected)
            else:
                self.minesWrong.append(mine)
                self.receivedMineWrongPos.emit(self.minesWrong)


    def run(self):

        if (self.isSimulation):
            rospy.Subscriber("/gazebo/model_states", ModelStates, self.receiveSimulationPose)
        else:
            pass #Ver como pegar a posição real pelo sistema de landmarks

        rospy.Subscriber("/HRATC_FW/set_mine", Pose, self.receiveMinePosition)
    	self.pubPose = rospy.Publisher('/HRATC_FW/pose', Pose)
        self.pubMineDetection_left = rospy.Publisher('/coil_l', Coil)
        self.pubMineDetection_middle = rospy.Publisher('/coil_m', Coil)
        self.pubMineDetection_right = rospy.Publisher('/coil_r', Coil)
        
        # Added a tf listener to check the position of the coils
        self.listener = tf.TransformListener()

        rospy.spin()

