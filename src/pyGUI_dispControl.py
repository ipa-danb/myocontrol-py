#!/usr/bin/python

# Import PySide classes
import sys
import matplotlib
matplotlib.use('Qt4Agg')
matplotlib.rcParams['backend.qt4']='PySide'

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas

import math
import rospy
from myo_msgs.srv import SetReference
from myo_msgs.srv import SetClutch
import logger

import pylab
from PySide.QtCore import *
from PySide.QtGui import *

class MatplotlibWidget(FigureCanvas):

    def __init__(self, parent=None,xlabel='x',ylabel='y',title='Title'):
        super(MatplotlibWidget, self).__init__(Figure())

        self.setParent(parent)
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.axes = self.figure.add_subplot(111)

        self.axes.set_xlabel(xlabel)
        self.axes.set_ylim([0,160])
        self.axes.set_xlim([0,1000])
        self.axes.set_ylabel(ylabel)
        self.axes.set_title(title)
        self.before = (0,0)

    def plotDataPoints(self,x,y):
        self.axes.plot([self.before[0],x],[self.before[1] ,y],'b-')
        self.before = (x,y)
        self.draw()

class OrthosisGUI:
    def __init__(self,geo):
        self.i          = 0
        self.before     = 5

        self.app = QApplication(sys.argv)
        self.DataPlot = MatplotlibWidget(xlabel='xv2',ylabel='yv2',title='')

        self.label = QLabel("<h1><b> 0 mNm </b></h1>")
        self.label.setMinimumSize(QSize(100,10))

        self.widget = QWidget()
        self.widget.setMinimumSize(QSize(600,600))
        self.widget.setWindowTitle('DisplacementControl')

        self.bar = QProgressBar()
        self.bar.setMinimumSize(QSize(75,450))
        self.bar.setOrientation(Qt.Vertical)
        self.bar.setMaximum(150)
        self.bar.setMinimum(5)
        self.bar.setAlignment(Qt.AlignRight)

        self.button = QDial()
        self.button.setNotchesVisible(1)
        self.button.setMinimum(5)
        self.button.setMaximum(150)
        self.button.setValue(5)
        self.button.valueChanged.connect(self.onChange)

        self.clutchLabel = QLabel("<b>ClutchState</b>")

        self.clutch = QPushButton("Disconnect")
        self.clutch.setMaximumSize(QSize(150,60))
        self.clutch.clicked.connect(self.onClick)
        self.state = 1

        self.log = QPushButton("Log")
        self.log.setMaximumSize(QSize(150,60))
        self.log.clicked.connect(self.onClickLogger)
        self.logState = 0

        self.vbox = QVBoxLayout()
        self.vbox.addWidget(self.label)
        self.vbox.addWidget(self.bar)

        self.vbox2 = QVBoxLayout()
        self.vbox2.addStretch(1)
        self.vbox2.addWidget(self.clutchLabel)
        self.vbox2.addWidget(self.clutch)
        self.vbox2.addWidget(self.log)
        self.vbox2.addStretch(1)

        self.hbox  = QHBoxLayout()
        self.hbox.addLayout(self.vbox)
        self.hbox.addWidget(self.button)
        self.hbox.addSpacing(100)
        self.hbox.addLayout(self.vbox2)


        self.widget.setLayout(self.hbox)
        self.widget.setGeometry(*geo)



    def show(self):
        self.widget.show()
        self.app.exec_()
        sys.exit()

    def setReference(self,reference):
        """Calls RosService /myo/myo_muscle0_controller/set_reference to set Displacement """
        rospy.wait_for_service('/myo/myo_muscle0_controller/set_ref')
        try:
            sDsp = rospy.ServiceProxy('/myo/myo_muscle0_controller/set_ref',SetReference)
            sDsp(reference)
        except rospy.ServiceException, e:
            print " "

    def setClutch(self):
        rospy.wait_for_service('/myo/myo_muscle0_controller/set_clt')
        try:
            serv = rospy.ServiceProxy('/myo/myo_muscle0_controller/set_clt', SetClutch)
            self.state = serv(not(self.state)).clutchState
        except rospy.ServiceException, e:
            sys.exit("Error! Cant Set Clutch")

    def onClickLogger(self):
        if self.logState == False:
            self.logIt = logger.simpleLogger("guiLog",bufferSize=1000)
            self.log.setText("Running...")
            self.logState = True
        elif self.logState == True:
            self.log.setText("Log")
            self.logIt.__exit__(None,None,None)
            delattr(self,"logIt")
            self.logState = False


    def onClick(self):
        self.setClutch()
        if self.state == 0:
            self.clutch.setText("Connect")
        else:
            self.clutch.setText("Disconnect")

    def onChange(self):
        self.i += 1
        #print "value requested ", button.value()
        if abs(self.before -self.button.value()) > 5 :
            self.button.setValue(self.before - math.copysign(5.0,(self.before-self.button.value() ) ) )
        self.bar.setValue(self.button.value())

        self.DataPlot.plotDataPoints(self.i,self.button.value())
        self.before =self.button.value()
        self.label.setText("<h1><b>" + str(self.button.value()*(15.0/1000.0 *93.0 *66.0 *0.5)) + " mNm </b></h1>")
        self.setReference(self.button.value())


if __name__ == '__main__':
    gui = OrthosisGUI([300,300,300,300])
    gui.show()

    # Enter Qt application main loop
