# -*- coding:utf8 -*-
import os, numpy as np
from time import *
from OpenGL.GL import *
from OpenGL.GLU import *
from PyQt4.QtOpenGL import *
from PyQt4 import QtGui, QtCore
from ConfigParser import ConfigParser
from ui.mainwindow import Ui_MainWindow
from numpy import deg2rad, rad2deg, arange
from newcompetitor import NewCompetitorDialog
from credits import Credits
from math import ceil, floor, atan2, degrees, radians, sqrt, cos, sin, pi

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

defaultpath = os.path.dirname(os.path.realpath(__file__))


def GLCircle(center, raio=0.15, angleStep = .5):
    x, y = center
    x, y = float(x), float(y)
    glTranslatef(x, y, 0.)
    glBegin(GL_POLYGON)
    for angle in arange(0,2*pi,deg2rad(angleStep)):
        x, y = raio*cos(angle), raio*sin(angle)
        glVertex3f(x, y, 0.)
    glEnd()
    x, y = center
    x, y = float(x), float(y)
    glTranslatef(-x, -y, 0.)


class GLWidget(QGLWidget):


    def __init__(self, parent = None):
        self.parent = parent
        super(GLWidget, self).__init__(QGLFormat(QGL.SampleBuffers), parent)

        self.center = [0,0]
        self.width, self.height = 0., 0.

        self.mousePos = None
        self.setMouseTracking(True)
        self.left, self.right, self.bottom, self.top = 0, self.width, self.height, 0


    def paintGL(self):

        self.viewUpdate()

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glPointSize(5)
        glLineWidth(1)
        glClear(GL_COLOR_BUFFER_BIT)
        self.parent.window().draw()


    def resizeGL(self, w, h):
        self.width, self.height = w, h
        self.updateGL()


    def initializeGL(self):
        glClearColor(0.86, 0.86, 0.86, 1.0)
        glClear(GL_COLOR_BUFFER_BIT)


    def paintEvent(self,event):
        pass


    def viewUpdate(self):
        w, h = self.width, self.height

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        tam = 12.
        tamHalf = tam/2.
        if w < h:
            aspect = h/float(w)
            self.left, self.right, self.bottom, self.top = -tamHalf, tamHalf, -tamHalf*aspect, tamHalf*aspect
        else:
            aspect = w/float(h)
            self.left, self.right, self.bottom, self.top = -tamHalf*aspect, tamHalf*aspect, -tamHalf, tamHalf
        glOrtho(self.left, self.right, self.bottom, self.top, -50.0, 50.0)
        glViewport(0, 0, w, h)
        gluLookAt(self.center[0], self.center[1], 0, self.center[0], self.center[1], -1, 0, 1, 0)


    def mouseMoveEvent(self,event):
        super(GLWidget,self).mouseMoveEvent(event)
        widthScenario, heightScenario = (self.right - self.left), (self.top - self.bottom)
        self.mousePos = (event.x()*widthScenario/self.width - widthScenario/2., 
                            event.y()*heightScenario/self.height - heightScenario/2.)



class MineWindow(QtGui.QMainWindow, Ui_MainWindow):

    config = None
    path = [[0., 0., 0.]]
    coilsPose = [[-.18,1],[0,1],[.18,1]]
    mines = []
    minesWrong = []
    minesDetected = {}
    minesExploded = []
    coils = None
    competitorName = None
    Map = None

    openConfig = QtCore.pyqtSignal()
    resetScore = QtCore.pyqtSignal()


    def __init__(self,parent,config):
        QtGui.QMainWindow.__init__(self)

        self.setupUi(self)

        #Adds GLWidget
        self.glwidget = GLWidget(self.mapwidget)

        self.updateConfig(config)

        self.connect( self.actionConfiguration, QtCore.SIGNAL("triggered()"), self.openConfig)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(1000)
        self.connect( self.timer, QtCore.SIGNAL("timeout()"), self.updateTimeLabel)

        self.gltimer = QtCore.QTimer(self)
        self.gltimer.setInterval(33)
        self.connect( self.gltimer, QtCore.SIGNAL("timeout()"), self.glwidget.updateGL)

        self.connect( self.startStopPB, QtCore.SIGNAL("clicked()"), self.startCompetitorDialog)
        self.connect( self.actionCoveredArea, QtCore.SIGNAL("toggled(bool)"),   self.uncheckCoilsSignal)

        self.connect( self.actionCoilsSignal, QtCore.SIGNAL("toggled(bool)"),   self.uncheckChannels)
        self.connect( self.actionChannel_1, QtCore.SIGNAL("toggled(bool)"),   self.uncheckChannels)
        self.connect( self.actionChannel_2, QtCore.SIGNAL("toggled(bool)"),   self.uncheckChannels)
        self.connect( self.actionChannel_3, QtCore.SIGNAL("toggled(bool)"),   self.uncheckChannels)

        self.connect( self.actionCoilsSignal, QtCore.SIGNAL("toggled(bool)"),   self.uncheckCoils)
        self.connect( self.actionCoil_1, QtCore.SIGNAL("toggled(bool)"),   self.uncheckCoils)
        self.connect( self.actionCoil_2, QtCore.SIGNAL("toggled(bool)"),   self.uncheckCoils)
        self.connect( self.actionCoil_3, QtCore.SIGNAL("toggled(bool)"),   self.uncheckCoils)

        self.newCompetitorDialog = NewCompetitorDialog(self)
        self.connect(self.newCompetitorDialog, QtCore.SIGNAL("newCompetitor(QString)"), self.startCompetitor)
    
        self.updateStartStop("Start")
        self.gltimer.start()

        self.connect(self.actionCredit, QtCore.SIGNAL("triggered()"), self.showCredits)


    def __del__(self):
        self.stop()


    def stop(self):
        self.gltimer.stop()


    def showCredits(self):
        credits = Credits(self)
        credits.show()


    def uncheckCoilsSignal(self,check):
        if not check:
            self.actionCoilsSignal.setChecked(False)


    def uncheckChannels(self,check):
        sender = self.sender()
        if check or sender==self.actionCoilsSignal:
            if self.actionChannel_1 != sender:
                self.actionChannel_1.setChecked(False)
            if self.actionChannel_2 != sender:
                self.actionChannel_2.setChecked(False)
            if self.actionChannel_3 != sender:
                self.actionChannel_3.setChecked(False)
        if self.actionCoilsSignal.isChecked():
            if not self.actionChannel_1.isChecked() and not self.actionChannel_2.isChecked() and not self.actionChannel_3.isChecked():
                    self.actionChannel_1.setChecked(True)


    def uncheckCoils(self,check):
        sender = self.sender()
        if check or sender==self.actionCoilsSignal:
            if self.actionCoil_1 != sender:
                self.actionCoil_1.setChecked(False)
            if self.actionCoil_2 != sender:
                self.actionCoil_2.setChecked(False)
            if self.actionCoil_3 != sender:
                self.actionCoil_3.setChecked(False)
        if self.actionCoilsSignal.isChecked():
            if not self.actionCoil_1.isChecked() and not self.actionCoil_2.isChecked() and not self.actionCoil_3.isChecked():
                    self.actionCoil_2.setChecked(True)


    def mouseMoveEvent(self,event):
        super(MineWindow,self).mouseMoveEvent(event)
        self.updateStatus()


    def updateStartStop(self,status):
        if status == "Start":
            icon = QtGui.QIcon()
            icon.addPixmap(QtGui.QPixmap(_fromUtf8(defaultpath+"/ui/start.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.startStopPB.setIcon(icon)
        elif status == "Stop":
            icon = QtGui.QIcon()
            icon.addPixmap(QtGui.QPixmap(_fromUtf8(defaultpath+"/ui/stop.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.startStopPB.setIcon(icon)

        self.startStopPB.setText(status)


    def resizeEvent(self,event):
        super(MineWindow,self).resizeEvent(event)
        self.glwidget.resize(self.mapwidget.width(),self.mapwidget.height())


    def startCompetitorDialog(self):
        pbText = str(self.startStopPB.text())

        if pbText == "Start":
            self.newCompetitorDialog.show()
        if pbText == "Stop":
            self.timer.stop()
            self.updateStartStop("Start")

            log = ConfigParser()
            log.add_section("Scoreboard")
            log.set("Scoreboard","MinesDetected",self.mineDetectedLB.text())
            log.set("Scoreboard","WrongMinesDetected",self.wrongMinesDetectedLB.text())
            log.set("Scoreboard","ExplodedMines",self.explodedMinesLB.text())
            log.set("Scoreboard","Time",self.timeLB.text())

            path = os.path.expanduser("~/HRATC 2014 Simulator/")
            if not os.path.exists(path):
                os.mkdir(path)

            path += "logs/"
            if not os.path.exists(path):
                os.mkdir(path)

            cfile = open(path+"/log_{}.log".format(self.competitorName),"w")  
            log.write(cfile)
            cfile.close()

            self.competitorName = None


    def updateScoreboard(self):
        self.mineDetectedLB.setText(str(len(self.minesDetected)))
        self.wrongMinesDetectedLB.setText(str(len(self.minesWrong)))
        self.explodedMinesLB.setText(str(len(self.minesExploded)))


    def startCompetitor(self, name):
        self.competitorName = name

        self.updateStartStop("Stop")
        self.resetScore.emit()
        self.coils = None
        self.start = time()

        self.timer.start()


    def updateConfig(self,config):
        self.cellXSize = config.cellWidth
        self.cellYSize = config.cellHeight
        self.width = config.mapWidth
        self.height = config.mapHeight

        self.mines = config.mines

        self.path = [[0., 0., 0.]]
        self.minesWrong = []
        self.minesDetected = {}
        self.minesExploded = []
        self.coils = None


    def keyPressEvent(self, event):
        super(MineWindow,self).keyPressEvent(event)
        
        key, text = event.key(), str(event.text()).lower()

        if key == QtCore.Qt.Key_Up:
            self.glwidget.center[1]+=3.
        if key == QtCore.Qt.Key_Down:
            self.glwidget.center[1]-=3.
        if key == QtCore.Qt.Key_Right:
            self.glwidget.center[0]+=3.
        if key == QtCore.Qt.Key_Left:
            self.glwidget.center[0]-=3.


    def updateRobotPos(self,path):
        self.path = path


    def addMinePos(self,minesDetected):
        self.minesDetected = minesDetected
        self.updateScoreboard()


    def addMineWrongPos(self,minesWrong):
        self.minesWrong = minesWrong
        self.updateScoreboard()


    def addMineExplodedPos(self,minesExploded):
        self.minesExploded = minesExploded
        self.updateScoreboard()


    def addCoils(self,coil):
        index = 0
        if self.actionCoil_2.isChecked():
            index = 1
        if self.actionCoil_3.isChecked():
            index = 2
        self.coils = coil[index]
        self.updateStatus()


    def updateStatus(self):
        msg = ""
        if self.glwidget.mousePos != None:
            x, y = self.glwidget.mousePos
            msg = "x: {}\ty: {}\t".format(x,y)
        if self.coils != None and self.coils != []:
            c1, c2, c3 = self.coils.channel
            z1, z2, z3 = self.coils.zero
            msg = "{0}Channels: [{1:.2f}, {2:.2f}, {3:.2f}]\tZeros: [{4:.2f}, {5:.2f}, {6:.2f}]".format(msg, c1, c2, c3, z1, z2, z3)
        self.statusbar.showMessage(msg)


    def updateMap(self,Map):
        mineMap, coveredArea, coilsPose = Map
        if coilsPose != []:
            self.coilsPose = coilsPose

        if self.actionCoilsSignal.isChecked():
            index = 0
            if self.actionChannel_1.isChecked():
                index = 0
            if self.actionChannel_2.isChecked():
                index = 1
            if self.actionChannel_3.isChecked():
                index = 2
            if self.actionCoil_2.isChecked():
                index += 3
            if self.actionCoil_3.isChecked():
                index += 6
            textureMap = mineMap[index,:,:]
            textureMap -= textureMap.min()
            textureMap /= textureMap.max()
            textureMap *= 255
            textureMap = textureMap.astype(np.uint8)
            textureMap[0,0] = 0
            textureMap[coveredArea != 183] = 220
            self.Map = textureMap
        else:
            self.Map = coveredArea

        if self.Map.dtype != np.uint8:
            self.Map[self.Map > 255] = 255
            self.Map[self.Map < 0  ] = 0
            self.Map = self.Map.astype(np.uint8)


    def updateTimeLabel(self):
        diff = time() - self.start
        self.timeLB.setText("{0:02}:{1:02}".format(int(diff/60.),int(diff - 60*int(diff/60.))))


    def draw(self):
        if self.competitorName != None:
            if self.actionBoundaries.isChecked():
                self.drawBoundaries()
            if self.actionCoveredArea.isChecked():
                self.drawCoveredArea()
            if self.actionRobotPath.isChecked():
                self.drawPath()
            if self.actionTrueMines.isChecked():
                self.drawMines()
            if self.actionMinesDetected.isChecked():
                self.drawMinesDetected()
            if self.actionWrongMinesDetected.isChecked():
                self.drawMinesWrong()
            if self.actionExplodedMines.isChecked():
                self.drawMinesExploded()
            if self.actionRobot.isChecked():
                self.drawRobot()


    def drawBoundaries(self):


        glColor3f(0.,0.,0.)
        glLineWidth(2.)

        w = self.width/2.
        h = self.height/2.

        glBegin(GL_LINE_STRIP)
        glVertex3f(-w,h,0.)
        glVertex3f(w,h,0.)
        glVertex3f(w,-h,0.)
        glVertex3f(-w,-h,0.)
        glVertex3f(-w,h,0.)
        glEnd()

        glLineWidth(1.)


    def drawCoveredArea(self):
        if self.Map != None:

            glEnable(GL_TEXTURE_2D)

            mapTexture = self.glwidget.bindTexture(
                            QtGui.QImage(
                                self.Map.data,
                                self.Map.shape[1],
                                self.Map.shape[0],
                                QtGui.QImage.Format_Indexed8))

            glColor3f(1.,1.,1.)
            glBindTexture(GL_TEXTURE_2D,mapTexture)
            glBegin(GL_QUADS)

            w = self.width/2.
            h = self.height/2.
            glTexCoord2d(0,1)
            glVertex3f(-w,h,0.)

            glTexCoord2d(1,1)
            glVertex3f(w,h,0.)

            glTexCoord2d(1,0)
            glVertex3f(w,-h,0.)

            glTexCoord2d(0,0)
            glVertex3f(-w,-h,0.)

            glEnd()

            glDisable(GL_TEXTURE_2D)
            self.glwidget.deleteTexture(mapTexture)


    def drawPath(self):
        if self.path != [] and len(self.path) > 1:
            glColor3f(0., 0., 0.)
            glBegin(GL_LINE_STRIP)
            for pt in self.path:
                glVertex3f(pt[0], pt[1], 0.)
            glEnd()


    def drawMines(self):
        if self.mines != []:
            glColor3f(0., 0., 1.)
            for mine in self.mines:
                GLCircle(mine)


    def drawMinesDetected(self):
        if self.minesDetected != []:
            glColor3f(0., .7, 0.)
            for mine in self.minesDetected:
                GLCircle(mine)


    def drawMinesWrong(self):
        if self.minesWrong != []:
            glColor3f(.701, 0., .909)
            for mine in self.minesWrong:
                GLCircle(mine)


    def drawMinesExploded(self):
        if self.minesExploded != []:
            glColor3f(1., 0., .0)
            for mine in self.minesExploded:
                GLCircle(mine)


    def drawRobot(self):
        if len(self.path) == 0:
            return
        x, y, th = self.path[-1]
        th = rad2deg(th)

        glTranslatef(x, y, 0.)
        glRotatef(th-90,0.,0.,1.)


        #robot base (and contour)
        glColor3f(.0, .0, .0)
        glBegin(GL_POLYGON)
        glVertex3f(-.23, -.515, 0.)
        glVertex3f(-.23, .515, 0.)
        glVertex3f(.23, .515, 0.)
        glVertex3f(.23, -.515, 0.)
        glVertex3f(-.23, -.515, 0.)
        glEnd()

        glBegin(GL_POLYGON)
        glColor3f(.89, .73, .1)
        glVertex3f(-.2725, -.495, 0.)
        glColor3f(.89, .8, .41)
        glVertex3f(-.2725, .495, 0.)
        glVertex3f(.2725, .495, 0.)
        glColor3f(.89, .73, .1)
        glVertex3f(.2725, -.495, 0.)
        glVertex3f(-.2725, -.495, 0.)
        glEnd()


        #wheels
        glColor3f(.0, .0, .0)

        for wheel in [[.2725, .277], [-.2725, .277], [-.2725, -.277], [.2725, -.277]]:
            x, y = wheel
            glTranslatef(x, y, 0.)

            glBegin(GL_POLYGON)
            glVertex3f(-.0625, -.165, 0.)
            glVertex3f(.0625, -.165, 0.)
            glVertex3f(.0625, .165, 0.)
            glVertex3f(-.0625, .165, 0.)
            glVertex3f(-.0625, -.165, 0.)
            glEnd()

            glTranslatef(-x, -y, 0.)

        glRotatef(-th+90,0.,0.,1.)


        #arm
        glColor3f(0., 0., 0.)
        glBegin(GL_LINES)

        x,y = self.coilsPose[1]
        glVertex3f(0., 0., 0.)
        glVertex3f(x, y, 0)

        glEnd()

        #sensor
        for c in self.coilsPose:
            GLCircle(c,0.1)

        glTranslatef(-x, -y, 0.)


