# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'credits.ui'
#
# Created: Fri Feb 28 17:58:09 2014
#      by: PyQt4 UI code generator 4.9.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_Credits(object):
    def setupUi(self, Credits):
        Credits.setObjectName(_fromUtf8("Credits"))
        Credits.setWindowModality(QtCore.Qt.ApplicationModal)
        Credits.resize(830, 281)
        Credits.setStyleSheet(_fromUtf8("QDialog\n"
"{\n"
" background: url(:/gui/ui/credits.png) no-repeat ;\n"
" background-position:50% 50%;\n"
"}"))
        self.hractLink = QtGui.QPushButton(Credits)
        self.hractLink.setGeometry(QtCore.QRect(380, 180, 98, 27))
        palette = QtGui.QPalette()
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.ButtonText, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.ButtonText, brush)
        brush = QtGui.QBrush(QtGui.QColor(159, 158, 158))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.ButtonText, brush)
        self.hractLink.setPalette(palette)
        self.hractLink.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.hractLink.setAutoDefault(False)
        self.hractLink.setFlat(True)
        self.hractLink.setObjectName(_fromUtf8("hractLink"))

        self.retranslateUi(Credits)
        QtCore.QMetaObject.connectSlotsByName(Credits)

    def retranslateUi(self, Credits):
        Credits.setWindowTitle(QtGui.QApplication.translate("Credits", "Credits", None, QtGui.QApplication.UnicodeUTF8))
        self.hractLink.setText(QtGui.QApplication.translate("Credits", "HRACT 2014", None, QtGui.QApplication.UnicodeUTF8))

import resource_rc
