#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy

from std_srvs.srv import SetBool
from geometry_msgs import msg
import sys
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QMainWindow,  QWidget, QVBoxLayout,  QHBoxLayout, QGroupBox
from PyQt5.QtWidgets import QLabel, QPushButton, QSlider
from PyQt5.QtCore import QSize, Qt
import tf


class BoolServiceClient(QWidget):
    def __init__(self, parent, service_string, alias):
        super(BoolServiceClient, self).__init__(parent)
        self.service_string = service_string
        self.alias = alias

        self.service_handle = rospy.ServiceProxy(self.service_string, SetBool)
        # Group Box widget and its inner layout
        groupBox = QGroupBox(alias + " caller")
        groupBoxLayout = QVBoxLayout()
        groupBox.setLayout(groupBoxLayout)

        # create button
        btn_start = QPushButton(alias + " Start", self)
        btn_start.resize(100, 32)
        btn_start.clicked.connect(lambda: self.service_call(True))

        # create button
        btn_stop = QPushButton(alias + " Stop", self)
        btn_stop.clicked.connect(lambda: self.service_call(False))
        btn_stop.resize(100, 32)

        # add button to group box layout
        groupBoxLayout.addWidget(btn_start)
        groupBoxLayout.addWidget(btn_stop)

        # Horizontal Layout
        self.layout = QHBoxLayout(self)
        self.layout.addWidget(groupBox)
        self.layout.addWidget(groupBox)

    def service_call(self, arg):
        rospy.wait_for_service(self.service_string)
        reply = self.service_handle(arg)
        rospy.loginfo("calling service : " + self.service_string)
        rospy.loginfo(reply)


class OrientationSliders(QWidget):
    def __init__(self, parent=None):
        super(OrientationSliders, self).__init__(parent)

        groupBoxLayout = QVBoxLayout()

        RollHLayout = QHBoxLayout()
        PitchHLayout = QHBoxLayout()
        YawHLayout = QHBoxLayout()

        groupBox = QGroupBox("Target Orientation")
        groupBox.setLayout(groupBoxLayout)
        groupBox.setFixedSize(320,150)

        self.pub = rospy.Publisher('/vscmg/target_orientation', msg.Quaternion, queue_size=10)

        self.slider_roll = QSlider(Qt.Horizontal)
        self.slider_roll.setMinimum(-180)
        self.slider_roll.setMaximum(180)
        self.slider_roll.setValue(0)
        self.slider_roll.setSingleStep(10)
        RollHLayout.addWidget(QLabel("x:"))
        RollHLayout.addWidget(self.slider_roll)
        self.slider_roll.valueChanged.connect(self.valuechange)

        self.slider_pitch = QSlider(Qt.Horizontal)
        self.slider_pitch.setMinimum(-180)
        self.slider_pitch.setMaximum(180)
        self.slider_pitch.setValue(0)
        self.slider_pitch.setSingleStep(10)
        PitchHLayout.addWidget(QLabel("y:"))
        PitchHLayout.addWidget(self.slider_pitch)
        self.slider_pitch.valueChanged.connect(self.valuechange)

        self.slider_yaw = QSlider(Qt.Horizontal)
        self.slider_yaw.setMinimum(-180)
        self.slider_yaw.setMaximum(180)
        self.slider_yaw.setValue(0)
        self.slider_yaw.setSingleStep(10)
        YawHLayout.addWidget(QLabel("z:"))
        YawHLayout.addWidget(self.slider_yaw)
        self.slider_yaw.valueChanged.connect(self.valuechange)

        groupBoxLayout.addLayout(RollHLayout)
        groupBoxLayout.addLayout(PitchHLayout)
        groupBoxLayout.addLayout(YawHLayout)
        self.layout=QHBoxLayout(self)
        self.layout.addWidget(groupBox)

    def valuechange(self):
        toRad = 3.1415927/180.0
        quaternion = tf.transformations.quaternion_from_euler( self.slider_roll.value() * toRad,self.slider_pitch.value() * toRad,self.slider_yaw.value()* toRad)
        self.pub.publish(quaternion[0],quaternion[1],quaternion[2],quaternion[3])
        



class MainWindow(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.setMinimumSize(QSize(340, 160))
        self.setWindowTitle("VSCMG GUI control")


        _home_widget = OrientationSliders()

        self.setCentralWidget(_home_widget)


if __name__ == "__main__":
    try:
        rospy.init_node('service_client')
        app = QtWidgets.QApplication(sys.argv)
        mainWin = MainWindow()
        mainWin.show()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass
