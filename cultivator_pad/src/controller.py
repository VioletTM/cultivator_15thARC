# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'cultivator_controller.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(600, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.mappingButton = QtWidgets.QPushButton(self.centralwidget)
        self.mappingButton.setGeometry(QtCore.QRect(50, 350, 150, 50))
        self.mappingButton.setObjectName("mappingButton")
        self.cultivateButton = QtWidgets.QPushButton(self.centralwidget)
        self.cultivateButton.setGeometry(QtCore.QRect(50, 425, 150, 50))
        self.cultivateButton.setObjectName("cultivateButton")
        self.autoButton = QtWidgets.QPushButton(self.centralwidget)
        self.autoButton.setGeometry(QtCore.QRect(50, 500, 150, 50))
        self.autoButton.setObjectName("autoButton")
        self.diggingButton = QtWidgets.QPushButton(self.centralwidget)
        self.diggingButton.setGeometry(QtCore.QRect(225, 500, 150, 50))
        self.diggingButton.setObjectName("diggingButton")
        self.moveableButton = QtWidgets.QPushButton(self.centralwidget)
        self.moveableButton.setGeometry(QtCore.QRect(400, 500, 150, 50))
        self.moveableButton.setObjectName("moveableButton")
        self.emgStopButton = QtWidgets.QPushButton(self.centralwidget)
        self.emgStopButton.setGeometry(QtCore.QRect(250, 350, 100, 100))
        self.emgStopButton.setText("")
        self.emgStopButton.setObjectName("emgStopButton")
        self.frontButton = QtWidgets.QPushButton(self.centralwidget)
        self.frontButton.setGeometry(QtCore.QRect(450, 325, 50, 50))
        self.frontButton.setObjectName("frontButton")
        self.backButton = QtWidgets.QPushButton(self.centralwidget)
        self.backButton.setGeometry(QtCore.QRect(450, 425, 50, 50))
        self.backButton.setObjectName("backButton")
        self.leftButton = QtWidgets.QPushButton(self.centralwidget)
        self.leftButton.setGeometry(QtCore.QRect(400, 375, 50, 50))
        self.leftButton.setObjectName("leftButton")
        self.rightButton = QtWidgets.QPushButton(self.centralwidget)
        self.rightButton.setGeometry(QtCore.QRect(500, 375, 50, 50))
        self.rightButton.setObjectName("rightButton")
        self.stopButton = QtWidgets.QPushButton(self.centralwidget)
        self.stopButton.setGeometry(QtCore.QRect(450, 375, 50, 50))
        self.stopButton.setObjectName("stopButton")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 600, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.frontButton.pressed.connect(MainWindow.go_front)
        self.backButton.pressed.connect(MainWindow.go_back)
        self.leftButton.pressed.connect(MainWindow.go_rotate_left)
        self.rightButton.pressed.connect(MainWindow.go_rotate_right)
        self.stopButton.clicked.connect(MainWindow.stop)
        self.emgStopButton.clicked['bool'].connect(MainWindow.emergency_stop)
        self.mappingButton.clicked['bool'].connect(MainWindow.mapping)
        self.cultivateButton.clicked['bool'].connect(MainWindow.cultivate)
        self.autoButton.clicked['bool'].connect(MainWindow.auto_flag)
        self.diggingButton.clicked['bool'].connect(MainWindow.digging)
        self.moveableButton.clicked['bool'].connect(MainWindow.moveable_flag)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.mappingButton.setText(_translate("MainWindow", "Mapping"))
        self.cultivateButton.setText(_translate("MainWindow", "Cultivate"))
        self.autoButton.setText(_translate("MainWindow", "Auto"))
        self.diggingButton.setText(_translate("MainWindow", "Digging"))
        self.moveableButton.setText(_translate("MainWindow", "Moveable"))
        self.frontButton.setText(_translate("MainWindow", "↑"))
        self.backButton.setText(_translate("MainWindow", "↓"))
        self.leftButton.setText(_translate("MainWindow", "←"))
        self.rightButton.setText(_translate("MainWindow", "→"))
        self.stopButton.setText(_translate("MainWindow", "■"))
