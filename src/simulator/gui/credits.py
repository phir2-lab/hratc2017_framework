# -*- coding:utf8 -*-

from PyQt4 import QtCore, QtGui
from ui.credits import Ui_Credits
import webbrowser

class Credits(QtGui.QDialog, Ui_Credits):

    def __init__(self, parent=None):
        super(Credits, self).__init__(parent)

        self.setupUi(self)
        self.connect(self.hractLink, QtCore.SIGNAL("clicked()"), self.openurl)

    def openurl(self):
        webbrowser.open("http://www2.isr.uc.pt/~embedded/events/HRATC2014/Welcome.html")


