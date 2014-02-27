#!/usr/bin/python
 
from PyQt4 import QtGui, QtCore
import sys, math, rospy
 
from gui.minewindow import *
from gui.mineconfig import *
from core.judgedredd import *
from core.config import *
 
class MineDetection(QtGui.QMainWindow):
    config = None
    mineWindow = None

    def __init__(self):
        QtGui.QMainWindow.__init__(self)

        self.config = Config()
        self.mineWindow = MineWindow(self,self.config)
        self.configWindow = MineDetectionConfig(self.mineWindow)
        self.judge = JudgeDredd(self,self.config)


    def showConfig(self):
        self.configWindow.show(self.config)


    def close(self):
        self.mineWindow.stop()
        self.judge.stop()
        self.configWindow.close()
        super(MineDetection,self).close()


    def updateConfig(self,config):
        self.config = config


    def main(self):
        rospy.init_node('HRATC_FW')

        self.connect( self.mineWindow, QtCore.SIGNAL("openConfig()"), self.showConfig)
        self.connect( self.mineWindow, QtCore.SIGNAL("finished()"), self.close)
        self.connect( self.configWindow, QtCore.SIGNAL("updateConfig(PyQt_PyObject)"), self.mineWindow.updateConfig)
        self.connect( self.configWindow, QtCore.SIGNAL("updateConfig(PyQt_PyObject)"), self.judge.updateConfig)
        self.connect( self.configWindow, QtCore.SIGNAL("updateConfig(PyQt_PyObject)"), self.updateConfig)
        self.connect( self.judge, QtCore.SIGNAL("receivedRobotPos(PyQt_PyObject)"), self.mineWindow.updateRobotPos)
        self.connect( self.judge, QtCore.SIGNAL("receivedMinePos(PyQt_PyObject)"), self.mineWindow.addMinePos)
        self.connect( self.judge, QtCore.SIGNAL("receivedMineWrongPos(PyQt_PyObject)"), self.mineWindow.addMineWrongPos)
        self.connect( self.judge, QtCore.SIGNAL("receivedMineExplodedPos(PyQt_PyObject)"),
                                                                        self.mineWindow.addMineExplodedPos)
        self.connect( self.judge, QtCore.SIGNAL("emitCoilSignal(PyQt_PyObject)"), self.mineWindow.addCoils)

        self.connect( self.judge, QtCore.SIGNAL("emitMap(PyQt_PyObject)"), self.mineWindow.updateMap)
        self.connect( self.mineWindow, QtCore.SIGNAL("resetScore()"), self.judge.resetScore)

        self.mineWindow.showMaximized()
        self.judge.start()

 
if __name__=='__main__':
    app = QtGui.QApplication(sys.argv)
    mineDetection = MineDetection()
    mineDetection.main()
    app.exec_()

