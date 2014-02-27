# -*- coding:utf8 -*-

from PyQt4 import QtCore, QtGui
from ui.newCompetitorDialog import Ui_newCompetitorDialog
from core.config import *

class NewCompetitorDialog(QtGui.QDialog, Ui_newCompetitorDialog):

    newCompetitor = QtCore.pyqtSignal("QString")

    def __init__(self, parent=None):
        super(NewCompetitorDialog, self).__init__(parent)
        self.setupUi(self)

    def accept(self):
        competitorName = str(self.competitorNameLE.text())
        self.newCompetitor.emit(competitorName)
        super(NewCompetitorDialog,self).accept()
