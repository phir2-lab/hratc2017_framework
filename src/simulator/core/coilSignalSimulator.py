from ConfigParser import ConfigParser
from minemapgenerator import MineMapGenerator, GenerateUsingRealDataset
from numpy import *
import os, random, tf, rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.msg import tfMessage
from metal_detector_msgs.msg._Coil import Coil
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
import rospkg

defaultpath = rospkg.RosPack().get_path("hratc2014_framework") + "/src/config/"
print defaultpath

class CoilSimulator(object):

    def __init__(self):
        self.load()
        self.listener = tf.TransformListener()
        self.pubMineDetection = rospy.Publisher("/coils", Coil)
        #rospy.Subscriber("/robot_pose_ekf/odom", PoseWithCovarianceStamped, self.receiveOdomEKF)


    def load(self,path=defaultpath+"config.ini"):
        configFile = ConfigParser()
        configFile.read(path)
        self.mapWidth = configFile.getfloat("MapDimensions","width")
        self.mapHeight = configFile.getfloat("MapDimensions","height")
        self.resolution = configFile.getfloat("MapDimensions","resolution")

        self.numMines = configFile.getint("Mines","numMines")
        self.randomMines = configFile.getboolean("Mines","RandomMines")

        self.minDistDetection = configFile.getfloat("Mines","DetectionMinDist")
        self.maxDistExplosion = configFile.getfloat("Mines","ExplosionMaxDist")

        self.numCellsX  = self.mapWidth/self.resolution
        self.numCellsY  = self.mapHeight/self.resolution
        self.numCells   = self.numCellsX * self.numCellsY

        self.minesFixedPos = configFile.get("Mines","MinesPositions")
        self.generateMines()


    def generateMines(self):
#        if self.randomMines:
#            self.mines = []
#            for i in range(self.numMines):
#                x = y = 0
#                while(sqrt(pow(x,2) + pow(y,2)) < 2):
#                    x, y = random.randrange(self.mapWidth)  - self.mapWidth/2., random.randrange(self.mapHeight) - self.mapHeight/2.
#                self.mines.append([x, y])
#        else:
        minesPos = self.minesFixedPos
        if minesPos !=  "":
            self.mines = [[float(v) for v in r.split(",")] for r in minesPos.split("|")]
        else:
            self.mines = []

        mWidth, mHeight = self.mapWidth/self.resolution,self.mapHeight/self.resolution
        mines = [ [mWidth/2. + m[0]/self.resolution, mHeight/2. - m[1]/self.resolution] for m in self.mines]

        metals1 = []
        for i in arange(0.25*self.numMines):
            metals1.append([ random.randrange(self.mapWidth)  - self.mapWidth/2., random.randrange(self.mapHeight) - self.mapHeight/2.])

        metals2 = []
        for i in arange(0.25*self.numMines):
            metals2.append([
                                random.randrange(self.mapWidth)  - self.mapWidth/2.,
                                random.randrange(self.mapHeight) - self.mapHeight/2.])

        metals1 = [ [mWidth/2. + m[0]/self.resolution, mHeight/2. - m[1]/self.resolution] for m in metals1]
        metals2 = [ [mWidth/2. + m[0]/self.resolution, mHeight/2. - m[1]/self.resolution] for m in metals2]

        self.mineMap, self.zeroChannel = GenerateUsingRealDataset(mines,metals1,metals2,mWidth,mHeight,True)

    def updateCoils(self,newPose):
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

            coils = [[x+leftCoilX/self.cellXSize,y-leftCoilY/self.cellXSize]]
            coils.append([x+middleCoilX/self.cellXSize,y-middleCoilY/self.cellXSize])
            coils.append([x+rightCoilX/self.cellXSize,y-rightCoilY/self.cellXSize])

            co = 0
            for x,y in coils:
                coil = Coil()
                coil.header.frame_id = "{}_coil".format(["left","middle","right"][co])
                if x <= self.mineMap.shape[2] and x >=0 and y <= self.mineMap.shape[1] and y >= 0:
                    for ch in range(3):
                        coil.channel.append(self.mineMap[3*co+ch,y,x] + random.random()*100)
                        coil.zero.append(self.zeroChannel[3*co+ch])

                if (self.isSimulation):
                    self.pubMineDetection.publish(coil)
                co += 1

    def receiveOdomEKF(self, odor):
        xm ,ym, (pitchm, rollm, yawm) = 549848.663622 , 4448426.10338, euler_from_quaternion([0.,0.,0.999998918808,0.00147050426938])

        q = [odor.pose.pose.orientation.x, odor.pose.pose.orientation.y, odor.pose.pose.orientation.z, odor.pose.pose.orientation.w]
        roll, pitch, yaw =  euler_from_quaternion(q)

        x, y = odor.pose.pose.position.x, odor.pose.pose.position.y
        x1 = (x-xm)*cos(yawm) + (y-ym)*sin(yawm)
        y1 = -(x-xm)*sin(yawm) + (y-ym)*cos(yawm)
        self.pose = [x1, y1, yaw-yawm]
        self.updateCoils(self.pose)



    def pubCoilsonMinefield(self):
        
        if self.listener == None:
            return

        # Lookup for the coils using tf
        try:
            (trans,rot) = self.listener.lookupTransform('minefield', 'left_coil', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        leftCoilX = trans[0]/self.resolution + self.numCellsX/2.0
        leftCoilY = -trans[1]/self.resolution + self.numCellsY/2.0

        try:
            (trans,rot) = self.listener.lookupTransform('minefield', 'middle_coil', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          return
        middleCoilX = trans[0]/self.resolution + self.numCellsX/2.0
        middleCoilY = -trans[1]/self.resolution + self.numCellsY/2.0

        try:
            (trans,rot) = self.listener.lookupTransform('minefield', 'right_coil', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        rightCoilX = trans[0]/self.resolution + self.numCellsX/2.0
        rightCoilY = -trans[1]/self.resolution + self.numCellsY/2.0

        coils = [[leftCoilX,leftCoilY]]
        coils.append([middleCoilX, middleCoilY])
        coils.append([rightCoilX, rightCoilY])

        co = 0
        for x,y in coils:
            coil = Coil()
            coil.header.frame_id = "{}_coil".format(["left","middle","right"][co])
            if x <= self.mineMap.shape[2] and x >=0 and y <= self.mineMap.shape[1] and y >= 0:
                for ch in range(3):
                    coil.channel.append(self.mineMap[3*co+ch,y,x] + random.random()*100)
                    coil.zero.append(self.zeroChannel[3*co+ch])

                self.pubMineDetection.publish(coil)
            co += 1

if __name__=='__main__':
        
    rospy.init_node('coilSimulator')
    coilSimulator = CoilSimulator()
        
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        coilSimulator.pubCoilsonMinefield()
        r.sleep()



