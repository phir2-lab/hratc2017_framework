# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'newCompetitorDialog.ui'
#
# Created: Tue Jan 14 15:49:40 2014
#      by: PyQt4 UI code generator 4.9.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_newCompetitorDialog(object):
    def setupUi(self, newCompetitorDialog):
        newCompetitorDialog.setObjectName(_fromUtf8("newCompetitorDialog"))
        newCompetitorDialog.setWindowModality(QtCore.Qt.WindowModal)
        newCompetitorDialog.resize(400, 111)
        self.verticalLayout = QtGui.QVBoxLayout(newCompetitorDialog)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.label = QtGui.QLabel(newCompetitorDialog)
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setObjectName(_fromUtf8("label"))
        self.verticalLayout.addWidget(self.label)
        self.competitorNameLE = QtGui.QLineEdit(newCompetitorDialog)
        self.competitorNameLE.setObjectName(_fromUtf8("competitorNameLE"))
        self.verticalLayout.addWidget(self.competitorNameLE)
        self.buttonBox = QtGui.QDialogButtonBox(newCompetitorDialog)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtGui.QDialogButtonBox.Cancel|QtGui.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName(_fromUtf8("buttonBox"))
        self.verticalLayout.addWidget(self.buttonBox)
        spacerItem = QtGui.QSpacerItem(20, 30, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)

        self.retranslateUi(newCompetitorDialog)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL(_fromUtf8("accepted()")), newCompetitorDialog.accept)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL(_fromUtf8("rejected()")), newCompetitorDialog.reject)
        QtCore.QMetaObject.connectSlotsByName(newCompetitorDialog)

    def retranslateUi(self, newCompetitorDialog):
        newCompetitorDialog.setWindowTitle(QtGui.QApplication.translate("newCompetitorDialog", "New team", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("newCompetitorDialog", "Team name:", None, QtGui.QApplication.UnicodeUTF8))

