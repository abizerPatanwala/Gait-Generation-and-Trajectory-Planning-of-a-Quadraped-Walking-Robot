#!/usr/bin/env python

# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'test_gui_v2.ui'
#
# Created by: PyQt4 UI code generator 4.12.1
#
# WARNING! All changes made in this file will be lost!

import rospy
import roslib
from std_msgs.msg import String, Float64
from PyQt4 import QtCore, QtGui
import sys

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

    #publishers
    chosenLeg_pub = rospy.Publisher('chosenLeg', String, queue_size=10)
    shoulder_joint_pub = rospy.Publisher('shoulder_value', String, queue_size=10)
    arm_joint_pub = rospy.Publisher('limb_value', String, queue_size=10)
    knee_joint_pub = rospy.Publisher('knee_value', String, queue_size=10)
    reset_pub = rospy.Publisher("reset_position", String, queue_size=10)


    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(600, 400)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.pushButtonPublish = QtGui.QPushButton(self.centralwidget)
        self.pushButtonPublish.setGeometry(QtCore.QRect(240, 250, 89, 25))
        self.pushButtonPublish.setObjectName(_fromUtf8("pushButtonPublish"))
        self.pushButtonStop = QtGui.QPushButton(self.centralwidget)
        self.pushButtonStop.setGeometry(QtCore.QRect(240, 290, 89, 25))
        self.pushButtonStop.setObjectName(_fromUtf8("pushButtonStop"))
        self.LegLabel = QtGui.QLabel(self.centralwidget)
        self.LegLabel.setGeometry(QtCore.QRect(40, 40, 141, 20))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.LegLabel.setFont(font)
        self.LegLabel.setObjectName(_fromUtf8("LegLabel"))
        self.lineEdit = QtGui.QLineEdit(self.centralwidget)
        self.lineEdit.setGeometry(QtCore.QRect(400, 62, 100, 25))
        self.lineEdit.setObjectName(_fromUtf8("lineEdit"))
        self.lineEdit_2 = QtGui.QLineEdit(self.centralwidget)
        self.lineEdit_2.setGeometry(QtCore.QRect(400, 107, 100, 25))
        self.lineEdit_2.setObjectName(_fromUtf8("lineEdit_2"))
        self.lineEdit_3 = QtGui.QLineEdit(self.centralwidget)
        self.lineEdit_3.setGeometry(QtCore.QRect(400, 150, 100, 25))
        self.lineEdit_3.setObjectName(_fromUtf8("lineEdit_3"))
        self.labelShoulderJointValue = QtGui.QLabel(self.centralwidget)
        self.labelShoulderJointValue.setGeometry(QtCore.QRect(290, 67, 96, 17))
        self.labelShoulderJointValue.setObjectName(_fromUtf8("labelShoulderJointValue"))
        self.labelLimbJointValue = QtGui.QLabel(self.centralwidget)
        self.labelLimbJointValue.setGeometry(QtCore.QRect(290, 107, 69, 17))
        self.labelLimbJointValue.setObjectName(_fromUtf8("labelLimbJointValue"))
        self.labelKneeJointValue = QtGui.QLabel(self.centralwidget)
        self.labelKneeJointValue.setGeometry(QtCore.QRect(290, 150, 70, 17))
        self.labelKneeJointValue.setObjectName(_fromUtf8("labelKneeJointValue"))
        self.splitter = QtGui.QSplitter(self.centralwidget)
        self.splitter.setGeometry(QtCore.QRect(40, 80, 123, 92))
        self.splitter.setOrientation(QtCore.Qt.Vertical)
        self.splitter.setObjectName(_fromUtf8("splitter"))
        self.radioButtonFrontLeft = QtGui.QRadioButton(self.splitter)
        self.radioButtonFrontLeft.setObjectName(_fromUtf8("radioButtonFrontLeft"))
        self.radioButtonFrontRight = QtGui.QRadioButton(self.splitter)
        self.radioButtonFrontRight.setObjectName(_fromUtf8("radioButtonFrontRight"))
        self.radioButtonBackLeft = QtGui.QRadioButton(self.splitter)
        self.radioButtonBackLeft.setObjectName(_fromUtf8("radioButtonBackLeft"))
        self.radioButtonbackRight = QtGui.QRadioButton(self.splitter)
        self.radioButtonbackRight.setObjectName(_fromUtf8("radioButtonbackRight"))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 600, 22))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QObject.connect(self.radioButtonFrontLeft, QtCore.SIGNAL(_fromUtf8("toggled(bool)")), self.leg1Ready)
        QtCore.QObject.connect(self.radioButtonFrontRight, QtCore.SIGNAL(_fromUtf8("toggled(bool)")), self.leg2Ready)
        QtCore.QObject.connect(self.radioButtonBackLeft, QtCore.SIGNAL(_fromUtf8("toggled(bool)")), self.leg3Ready)
        QtCore.QObject.connect(self.radioButtonbackRight, QtCore.SIGNAL(_fromUtf8("toggled(bool)")), self.leg4Ready)
        QtCore.QObject.connect(self.lineEdit, QtCore.SIGNAL(_fromUtf8("textEdited(QString)")), self.readValues)
        QtCore.QObject.connect(self.lineEdit_2, QtCore.SIGNAL(_fromUtf8("textEdited(QString)")), self.readValues)
        QtCore.QObject.connect(self.lineEdit_3, QtCore.SIGNAL(_fromUtf8("textEdited(QString)")), self.readValues)
        QtCore.QObject.connect(self.pushButtonPublish, QtCore.SIGNAL(_fromUtf8("clicked()")), self.publishChanges)
        QtCore.QObject.connect(self.pushButtonStop, QtCore.SIGNAL(_fromUtf8("clicked()")), self.release)
        
        
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.pushButtonPublish.setText(_translate("MainWindow", "Publish", None))
        self.pushButtonStop.setText(_translate("MainWindow", "STOP", None))
        self.LegLabel.setText(_translate("MainWindow", "Choose your leg !", None))
        self.labelShoulderJointValue.setText(_translate("MainWindow", "ShoulderJoint", None))
        self.labelLimbJointValue.setText(_translate("MainWindow", "LimbJoint", None))
        self.labelKneeJointValue.setText(_translate("MainWindow", "KneeJoint", None))
        self.radioButtonFrontLeft.setText(_translate("MainWindow", "FrontLeft leg", None))
        self.radioButtonFrontRight.setText(_translate("MainWindow", "FrontRight leg", None))
        self.radioButtonBackLeft.setText(_translate("MainWindow", "BackLeft leg", None))
        self.radioButtonbackRight.setText(_translate("MainWindow", "BackRight leg", None))


    def leg1Ready(self, radiobutton):
        if radiobutton == True:
            self.chosenLeg_pub.publish("FL")

    def leg2Ready(self, radiobutton):
        if radiobutton == True:
            self.chosenLeg_pub.publish("FR")
    
    def leg3Ready(self, radiobutton):
        if radiobutton == True:
            self.chosenLeg_pub.publish("BL")

    def leg4Ready(self, radiobutton):
        if radiobutton == True:
            self.chosenLeg_pub.publish("BR")

    def readValues(self):
        self.value1 = str(self.lineEdit.text())
        self.value2 = str(self.lineEdit_2.text())
        self.value3 = str(self.lineEdit_3.text())

    def publishChanges(self):
        print self.value1, self.value2, self.value3
        self.shoulder_joint_pub.publish(self.value1)
        self.arm_joint_pub.publish(self.value2)
        self.knee_joint_pub.publish(self.value3)

    def release(self):
        self.reset_pub.publish("HALT")


if __name__ == "__main__":
    rospy.init_node('GUI')
    r = rospy.Rate(100)
    rospy.loginfo("GUI initialized, let's get started!")

    app = QtGui.QApplication(sys.argv)
    MainWindow = QtGui.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

