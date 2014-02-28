# -*- coding:utf8 -*-

from PyQt4 import QtCore, QtGui
from ui.mineconfig_ui import Ui_ConfigDialog
from core.config import *

class MineDetectionConfig(QtGui.QDialog, Ui_ConfigDialog):

    updateConfig = QtCore.pyqtSignal(Config)
    config = None

    def __init__(self, parent=None):
        super(MineDetectionConfig, self).__init__(parent)

        self.setupUi(self)

        self.addBtn.clicked.connect(self.addTWMinesRow)
        self.minusBtn.clicked.connect(self.minusTWMinesRow)

        self.resetBtn.clicked.connect(self.resetClicked)
        self.cancelBtn.clicked.connect(self.close)
        self.okBtn.clicked.connect(self.okClicked)


    def resetClicked(self):
        self.config.load()
        self.unpackConfig(self.config)


    def okClicked(self):
        config = self.packConfig()
        if config != None:
            config.save()
            self.updateConfig.emit(config)
            self.close()

    
    def packConfig(self):
        config = Config()
        config.mapWidth = self.spinMapWidth.value()
        config.mapHeight = self.spinMapHeight.value()

        config.randomMines = self.radioButtonRandomMines.isChecked()
        if config.randomMines:
            config.numMines = self.spinMineNumber.value()
        else:
            config.numMines = self.TWMinesPos.rowCount()
            config.mines = []
            invalids = []
            for i in range(config.numMines):
                try:
                    config.mines.append([float(self.TWMinesPos.item(i,0).text()), float(self.TWMinesPos.item(i,1).text())])
                except:
                    invalids.append(i)

            if invalids != []:
                errorWindow = QtGui.QMessageBox(self)
                msg = "\nThe follow fixed mines positions have a invalid position:\n\n"
                for index in invalids:
                    msg += "\t{0} - {1} {2}\n".format(index+1,
                        self.TWMinesPos.item(index,0).text(),
                        self.TWMinesPos.item(index,1).text())
                msg += "\n\n"
                errorWindow.setText(QtCore.QString(msg))
                errorWindow.show()
                return
        config.minDistDetection = self.spinBoxMinDistDetection.value()/100.
        config.maxDistExplosion = self.spinBoxMaxDistExplosion.value()/100.

        return config

    
    def unpackConfig(self,config):
        self.spinMapWidth.setValue(config.mapWidth)
        self.spinMapHeight.setValue(config.mapHeight)

        self.spinMineNumber.setValue(config.numMines)

        self.spinBoxMinDistDetection.setValue(config.minDistDetection*100)
        self.spinBoxMaxDistExplosion.setValue(config.maxDistExplosion*100)

        if not config.randomMines:
            self.radioButtonFixedMines.setChecked(True)
            self.TWMinesPos.setRowCount(config.numMines)
            for i in range(config.numMines):
                item = QtGui.QTableWidgetItem()
                item.setText(QtCore.QString(str(config.mines[i][0])))
                self.TWMinesPos.setItem(i,0,item)
                item = QtGui.QTableWidgetItem()
                item.setText(QtCore.QString(str(config.mines[i][1])))
                self.TWMinesPos.setItem(i,1,item)


    def addTWMinesRow(self):
        self.TWMinesPos.setRowCount(self.TWMinesPos.rowCount() + 1)


    def minusTWMinesRow(self):
        if self.TWMinesPos.rowCount() != 0:
            self.TWMinesPos.setRowCount(self.TWMinesPos.rowCount() - 1)


    def show(self,config):
        self.unpackConfig(config)
        self.config = config

        super(MineDetectionConfig,self).show()


    def close(self):
        self.hide()
        

