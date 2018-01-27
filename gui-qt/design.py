# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'example2.ui'
#
# Created by: PyQt5 UI code generator 5.6
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.mpl = MplWidget(self.centralwidget)
        self.mpl.setGeometry(QtCore.QRect(350, 120, 321, 251))
        self.mpl.setObjectName("mpl")
        self.pushButton_plotData = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_plotData.setGeometry(QtCore.QRect(130, 340, 80, 23))
        self.pushButton_plotData.setObjectName("pushButton_plotData")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 20))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton_plotData.setText(_translate("MainWindow", "PushButton"))

from mplwidget import MplWidget
