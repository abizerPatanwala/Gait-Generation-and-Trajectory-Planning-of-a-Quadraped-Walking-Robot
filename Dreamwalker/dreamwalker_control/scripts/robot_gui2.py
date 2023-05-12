#!/usr/bin/env python

# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'robot_gui.ui'
#
# Created by: PyQt4 UI code generator 4.12.1
#
# WARNING! All changes made in this file will be lost!

import rospy
import roslib
from PyQt4 import QtCore, QtGui
from std_msgs.msg import String, Float32, Int32
from sensor_msgs.msg import Range
from dreamwalker_control.srv import Service_GUI_Command, Service_GUI_CommandResponse
import sys
import threading

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):

    #service client
    rospy.wait_for_service('command_service')
    commanded_service = rospy.ServiceProxy('command_service', Service_GUI_Command)

    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(420, 639)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.pushButton_forward = QtGui.QPushButton(self.centralwidget)
        self.pushButton_forward.setGeometry(QtCore.QRect(160, 40, 100, 80))
        self.pushButton_forward.setObjectName(_fromUtf8("pushButton_forward"))
        self.pushButton_backward = QtGui.QPushButton(self.centralwidget)
        self.pushButton_backward.setGeometry(QtCore.QRect(160, 220, 100, 80))
        self.pushButton_backward.setObjectName(_fromUtf8("pushButton_backward"))
        self.pushButton_right = QtGui.QPushButton(self.centralwidget)
        self.pushButton_right.setGeometry(QtCore.QRect(290, 120, 80, 100))
        self.pushButton_right.setObjectName(_fromUtf8("pushButton_right"))
        self.pushButton_left = QtGui.QPushButton(self.centralwidget)
        self.pushButton_left.setGeometry(QtCore.QRect(50, 120, 80, 100))
        self.pushButton_left.setObjectName(_fromUtf8("pushButton_left"))
        self.pushButton_stop = QtGui.QPushButton(self.centralwidget)
        self.pushButton_stop.setGeometry(QtCore.QRect(170, 130, 80, 80))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_stop.setFont(font)
        self.pushButton_stop.setObjectName(_fromUtf8("pushButton_stop"))
        self.textBrowser = QtGui.QTextBrowser(self.centralwidget)
        self.textBrowser.setGeometry(QtCore.QRect(30, 350, 360, 200))
        self.textBrowser.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOn)
        self.textBrowser.setObjectName(_fromUtf8("textBrowser"))
        self.pushButton_spinLEFT = QtGui.QPushButton(self.centralwidget)
        self.pushButton_spinLEFT.setGeometry(QtCore.QRect(50, 50, 90, 50))
        self.pushButton_spinLEFT.setObjectName(_fromUtf8("pushButton_spinLEFT"))
        self.pushButton_spinRIGHT = QtGui.QPushButton(self.centralwidget)
        self.pushButton_spinRIGHT.setGeometry(QtCore.QRect(280, 50, 90, 50))
        self.pushButton_spinRIGHT.setObjectName(_fromUtf8("pushButton_spinRIGHT"))
        self.label = QtGui.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(30, 320, 80, 20))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setObjectName(_fromUtf8("label"))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 420, 22))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QObject.connect(self.pushButton_backward, QtCore.SIGNAL(_fromUtf8("clicked()")), self.goBackward)
        QtCore.QObject.connect(self.pushButton_forward, QtCore.SIGNAL(_fromUtf8("clicked()")), self.goForward)
        QtCore.QObject.connect(self.pushButton_right, QtCore.SIGNAL(_fromUtf8("clicked()")), self.goRight)
        QtCore.QObject.connect(self.pushButton_left, QtCore.SIGNAL(_fromUtf8("clicked()")), self.goLeft)
        QtCore.QObject.connect(self.pushButton_stop, QtCore.SIGNAL(_fromUtf8("clicked()")), self.stopMovement)
        QtCore.QObject.connect(self.pushButton_spinRIGHT, QtCore.SIGNAL(_fromUtf8("clicked()")), self.spinRight)
        QtCore.QObject.connect(self.pushButton_spinLEFT, QtCore.SIGNAL(_fromUtf8("clicked()")), self.spinLeft)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.pushButton_forward.setText(_translate("MainWindow", "FORWARD", None))
        self.pushButton_backward.setText(_translate("MainWindow", "BACKWARD", None))
        self.pushButton_right.setText(_translate("MainWindow", "RIGHT", None))
        self.pushButton_left.setText(_translate("MainWindow", "LEFT", None))
        self.pushButton_stop.setText(_translate("MainWindow", "STOP", None))
        self.pushButton_spinLEFT.setText(_translate("MainWindow", "SPIN LEFT", None))
        self.pushButton_spinRIGHT.setText(_translate("MainWindow", "SPIN RIGHT", None))
        self.label.setText(_translate("MainWindow", "Log info", None))

    def goBackward(self):
        command = self.commanded_service("BACKWARD")
        obtained_feedback = command.response
        self.textBrowser.append(obtained_feedback)

    def goForward(self):
        command = self.commanded_service("FORWARD")
        obtained_feedback = command.response
        self.textBrowser.append(obtained_feedback)

    def goRight(self):
        command = self.commanded_service("RIGHT")
        obtained_feedback = command.response
        self.textBrowser.append(obtained_feedback)

    def goLeft(self):
        command = self.commanded_service("LEFT")
        obtained_feedback = command.response
        self.textBrowser.append(obtained_feedback)

    def spinRight(self):
        command = self.commanded_service("SPIN_RIGHT")
        obtained_feedback = command.response
        self.textBrowser.append(obtained_feedback)

    def spinLeft(self):
        command = self.commanded_service("SPIN_LEFT")
        obtained_feedback = command.response
        self.textBrowser.append(obtained_feedback)

    def stopMovement(self):
        command = self.commanded_service("STOP")
        obtained_feedback = command.response
        self.textBrowser.append(obtained_feedback)



if __name__ == "__main__":
    rospy.init_node('gui_node')
    r = rospy.Rate(100)
    rospy.loginfo("GUI initialized, let's get started!")

    #x = threading.Thread(target=)

    app = QtGui.QApplication(sys.argv)
    MainWindow = QtGui.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
