#!/usr/bin/python

# Import PySide classes
import sys

import math
import rospy
import myo_msgs.srv
import logger

import PySide.QtGui
import PySide.QtCore

class OrthosisGUI:
    """Simple GUI for Orthosis Displacement Controller

    Attributes
    -----------

    increment : int
        Increments allowed for the Dial Button [default: 10]
    maxDisp : int
        Maximum Displacement counts that can be set as target [default: 150]
    minDisp : int
        Minimum Displacement counts that can be set as target [default: 5]


    .. warning:: Be careful with those, especially with minDisp as there is no build in security measure to prevent damage to the orthosis

    Attributes
    -----------
    clutchState : bool
        Variable to store clutch state
    logState : bool
        Variable to store state of logger


    Parameters
    -----------
    geo : [x y width height]
        list that defines geometry of GUI window at startup.

        - `x`, `y` relative position of window
        - `width`, `height` size of window

    """
    def __init__(self,geo):

        # TODO check if there is a better way of arranging all elements
        # TODO switch to QML

        # Set increments for Dial Knob to default
        self.increment  = 10
        self.maxDisp = 150
        self.minDisp = 5    # TODO create Setter that enforces values > 0

        # Create Qt App
        self.app = PySide.QtGui.QApplication(sys.argv)

        # Create Label for progressbar and force minimal size
        self.label = PySide.QtGui.QLabel("<h1><b> 0 mNm </b></h1>")
        self.label.setMinimumSize(PySide.QtCore.QSize(100,10))

        # Create the Core window widget for the app
        self.widget = PySide.QtGui.QWidget()
        self.widget.setMinimumSize(PySide.QtCore.QSize(600,600)) # force a certain Size
        self.widget.setWindowTitle('DisplacementControl')        # Name it

        # Create Progressbar as visualization for Displacement
        self.bar = PySide.QtGui.QProgressBar()
        self.bar.setMinimumSize(PySide.QtCore.QSize(75,450))
        self.bar.setOrientation(PySide.QtCore.Qt.Vertical) # TODO does this even have an effect?
        self.bar.setMaximum(self.maxDisp)
        self.bar.setMinimum(self.minDisp)
        self.bar.setAlignment(PySide.QtCore.Qt.AlignRight) # TODO does this even have an effect?

        # Create Dial Knob
        self.button = PySide.QtGui.QDial()
        self.button.setNotchesVisible(1) # TODO is this necessary?
        self.button.setMinimum(self.minDisp)
        self.button.setMaximum(self.maxDisp)
        self.button.setValue(self.minDisp) # start value set to minimum Displacement
        self.button.valueChanged.connect(self._onChange) # connect QDial to function

        # Create a Label to make clutchstatus more visable
        self.clutchLabel = PySide.QtGui.QLabel("<b>ClutchState</b>")

        #  Pushbutton for Clutch - This should change its
        self.clutch = PySide.QtGui.QPushButton("Disconnect")
        self.clutch.setMaximumSize(PySide.QtCore.QSize(150,60)) # force minimal side
        self.clutch.clicked.connect(self._onClick)
        self._clutchState = 1 # TODO introduce magic property and create a setter for this

        # Pushbutton for Logger
        self.log = PySide.QtGui.QPushButton("Log")
        self.log.setMaximumSize(PySide.QtCore.QSize(150,60))
        self.log.clicked.connect(self._onClickLogger)
        self._logState = 0  # TODO introduce magic property and create a setter for this


        # Layout stuff
        self.vbox = PySide.QtGui.QVBoxLayout()
        self.vbox.addWidget(self.label)
        self.vbox.addWidget(self.bar)

        self.vbox2 = PySide.QtGui.QVBoxLayout()
        self.vbox2.addStretch(1)
        self.vbox2.addWidget(self.clutchLabel)
        self.vbox2.addWidget(self.clutch)
        self.vbox2.addWidget(self.log)
        self.vbox2.addStretch(1)

        self.hbox  = PySide.QtGui.QHBoxLayout()
        self.hbox.addLayout(self.vbox)
        self.hbox.addWidget(self.button)
        self.hbox.addSpacing(100)
        self.hbox.addLayout(self.vbox2)

        # Set start geometry
        self.widget.setLayout(self.hbox)
        self.widget.setGeometry(*geo)

        # private variables
        self._before     = 5

    def show(self):
        """Function to actually draw the app and execute it

        This needs to run in its own thread and is responsible for the callbacks

        """
        self.widget.show()
        self.app.exec_()
        sys.exit()

    def setReference(self,reference):
        """Calls RosService /myo/myo_muscle0_controller/set_reference to set Displacement """
        rospy.wait_for_service('/myo/myo_muscle0_controller/set_ref')
        try:
            sDsp = rospy.ServiceProxy('/myo/myo_muscle0_controller/set_ref',myo_msgs.srv.SetReference)
            sDsp(reference)
        except(rospy.ServiceException, e):
            print(" ")

    def setClutch(self):
        rospy.wait_for_service('/myo/myo_muscle0_controller/set_clt')
        try:
            serv = rospy.ServiceProxy('/myo/myo_muscle0_controller/set_clt', myo_msgs.srv.SetClutch)
            self._clutchState = serv(not(self._clutchState)).clutchState
        except(rospy.ServiceException, e):
            sys.exit("Error! Cant Set Clutch")

    def _onClickLogger(self):
        if self._logState == False:
            self.logIt = logger.simpleLogger("guiLog",bufferSize=1000)
            self.log.setText("Running...")
            self._logself._clutchState = True
        elif self._logState == True:
            self.log.setText("Log")
            self.logIt.__exit__(None,None,None)
            delattr(self,"logIt")
            self._logState = False


    def _onClick(self):
        self.setClutch()
        if self._clutchState == 0:
            self.clutch.setText("Connect")
        else:
            self.clutch.setText("Disconnect")

    def _onChange(self):
        #print "value requested ", button.value()
        if abs(self._before -self.button.value()) > self.increment :
            self.button.setValue(self._before - math.copysign(self.increment,(self._before-self.button.value() ) ) )
        self.bar.setValue(self.button.value())

        self._before =self.button.value()
        self.label.setText("<h1><b>" + str(self.button.value()*(15.0/1000.0 *93.0 *66.0 *0.5)) + " mNm </b></h1>")
        self.setReference(self.button.value())


if __name__ == '__main__':
    gui = OrthosisGUI([300,300,300,300])
    gui.show()

    # Enter Qt application main loop
