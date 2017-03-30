#!/usr/bin/python

# Import PySide classes
import sys

import math
import rospy
import myo_msgs.srv
import logger
import csv
import trajectoryDisp as td

import PySide.QtGui
import PySide.QtCore

class MyThread(PySide.QtCore.QThread):
    updateProgress = PySide.QtCore.Signal(int)
    abortFlag = False
    bufferSize = 10000

    def __init__(self,mover,logfileName = None):
        PySide.QtCore.QThread.__init__(self)
        self._mover = mover
        self.logfileName = logfileName

    def abort(self):
        self.abortFlag = True

    def run(self):
        it = self._mover.traj()
        if self.logfileName == None:
            for i in it:
                if self.abortFlag:
                    break
                self.updateProgress.emit(i)
        else:
            with logger.simpleLogger(self.logfileName,self.bufferSize) as log:
                for i in it:
                    if self.abortFlag:
                        break
                    self.updateProgress.emit(i)



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

        # Create widget for Trajectory driver
        self.driveWidget = PySide.QtGui.QDialog()
        self.driveWidget.setMinimumSize(PySide.QtCore.QSize(300,150)) # force a certain Size
        self.driveWidget.setWindowTitle('Drive Control')        # Name it
        self.driveWidget.finished.connect(self._onDialogClosed)

        # Create widget for Trajectory progressbar
        self.bar2 = PySide.QtGui.QProgressBar()
        self.bar2.setMaximum(100)
        self.bar2.setMinimum(0)
        self.bar2.setValue(0)

        # Create CancelButton for Trajectory driver
        self.driveDialogButton = PySide.QtGui.QPushButton("Cancel")
        self.driveDialogButton.clicked.connect(self._onTrajCancel)

        # Layout for Trajectory Driver
        self.hboxDrive = PySide.QtGui.QHBoxLayout()
        self.hboxDrive.setAlignment(PySide.QtCore.Qt.AlignRight)
        self.hboxDrive.addWidget(self.driveDialogButton)

        self.vboxDrive = PySide.QtGui.QVBoxLayout()
        self.vboxDrive.addWidget(self.bar2)
        self.vboxDrive.addLayout(self.hboxDrive)

        self.driveWidget.setLayout(self.vboxDrive)


        # Create the Core window widget for the app
        self.widget = PySide.QtGui.QWidget()
        self.widget.setMinimumSize(PySide.QtCore.QSize(900,600)) # force a certain Size
        self.widget.setWindowTitle('DisplacementControl')        # Name it

        # Create Progressbar as visualization for Displacement
        self.bar = PySide.QtGui.QProgressBar()
        self.bar.setMinimumSize(PySide.QtCore.QSize(75,450))
        self.bar.setOrientation(PySide.QtCore.Qt.Vertical)
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

        # Create a Label to make clutchstatus more visable
        self.fileLabel = PySide.QtGui.QLabel("<b>Trajectory</b>")

        # Pushbutton for FileDialog
        self.fileButton = PySide.QtGui.QPushButton("Open Trajectory")
        self.fileButton.setMaximumSize(PySide.QtCore.QSize(150,60))
        self.fileButton.clicked.connect(self._onClickDialog)
        self.dialog = PySide.QtGui.QFileDialog(directory="/home/ronexros/workspace/scripts")
        self.dialog.fileSelected.connect(self._onFileChange)

        # Pushbutton for FileDialog
        self.driveTrajButton = PySide.QtGui.QPushButton("Drive Trajectory")
        self.driveTrajButton.setMaximumSize(PySide.QtCore.QSize(150,60))
        self.driveTrajButton.clicked.connect(self._onClickDrive)

        # Textbrowser
        self.textBrowser = PySide.QtGui.QPlainTextEdit()
        self.textBrowser.setReadOnly(True)


        self.tableWidget = PySide.QtGui.QTableWidget()
        self.tableWidget.setColumnCount(3)

        self.checkBox =  PySide.QtGui.QCheckBox("Trajectory Log")

        # Layout stuff
        self.vbox = PySide.QtGui.QVBoxLayout()
        self.vbox.addWidget(self.label)
        self.vbox.addWidget(self.bar)

        self.vbox3 = PySide.QtGui.QVBoxLayout()
        self.vbox3.addStretch(1)
        self.vbox3.addWidget(self.clutchLabel)
        self.vbox3.addWidget(self.clutch)
        self.vbox3.addWidget(self.log)
        self.vbox3.addWidget(self.fileLabel)
        self.vbox3.addWidget(self.fileButton)
        self.vbox3.addWidget(self.driveTrajButton)
        self.vbox3.addWidget(self.checkBox)
        self.vbox3.addStretch(1)

        self.vbox2 = PySide.QtGui.QVBoxLayout()
        self.vbox2.addWidget(self.button)

        self.vbox4 = PySide.QtGui.QVBoxLayout()
        self.vbox2.addWidget(self.textBrowser)

        self.hbox  = PySide.QtGui.QHBoxLayout()
        self.hbox.addLayout(self.vbox)
        self.hbox.addLayout(self.vbox2)
        self.hbox.addSpacing(30)
        self.hbox.addLayout(self.vbox3)
        self.hbox.addSpacing(30)
        self.hbox.addLayout(self.vbox4)

        # Set start geometry
        self.widget.setLayout(self.hbox)
        self.widget.setGeometry(*geo)

        # private variables
        self._before        = 5
        self._trajfile      = None
        self._bufferSize    = 1000

        self._commands = list()
        self._value    = list()
        self._waittime = list()


    def _onFileChange(self):
        """Callback when File is selected in FileDialog"""
        self.tableWidget.clear()
        del self._commands[:]
        del self._value[:]
        del self._waittime[:]

        self._trajfile = self.dialog.selectedFiles()[0]
        try:
            self._mover = td.TrajectoryMover( bufferSize=self._bufferSize, fileName=self._trajfile )
            assert self._mover.checkFile()

            with open(self._trajfile, 'rb') as text:
                self.textBrowser.setPlainText(text.read())

        except AssertionError:
            print "AssertionError!"
            self.errorWindow = PySide.QtGui.QMessageBox()
            self.errorWindow.setText(self._trajfile + " is not a proper trajectory File")
            self.errorWindow.show()

        print self._trajfile

    def _onClickDialog(self):
        # use exec_ vs show to block access to underlying window
        self.dialog.exec_()

    def _onTrajCancel(self):
        self.driveWidget.close()

    def _onDialogClosed(self):
        self.workerThread.abort()
        self.driveDialogButton.setText('Cancel')
        self.bar2.setValue(0)

    def _setTrajButton(self):
        if not self.workerThread.abortFlag:
            self.driveDialogButton.setText('Done')
        else:
            self.driveDialogButton.setText('Cancel')

    def _onClickDrive(self):
        """Function to Drive Trajectory prior selected
        """
        self.errorWindow = PySide.QtGui.QMessageBox()

        if self._trajfile == None:
            self.errorWindow.setText("No Trajectory File was choosen")
            self.errorWindow.show()

        elif self._mover.correctFlag:
            self.errorWindow.setText("Finished driving " + self._trajfile )
            if self.checkBox.isChecked():
                logfileName = '/home/ronexros/workspace/data/derp'
            else:
                logfileName = None

            self.workerThread = MyThread(self._mover,logfileName)


            self.workerThread.updateProgress.connect(self.setProgress)
            self.workerThread.finished.connect(self._setTrajButton)
            self.workerThread.start()
            self.driveWidget.exec_()

        else:
            self.errorWindow.setText(self._trajfile + " is not a proper trajectory File")
            self.errorWindow.setInformativeText("Please choose a valid ")
            self.errorWindow.show()


    def setProgress(self,progress):
        self.bar2.setValue(progress)

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
        """Calls RosService /myo/myo_muscle0_controller/set_reference to set ClutchState """
        rospy.wait_for_service('/myo/myo_muscle0_controller/set_clt')
        try:
            serv = rospy.ServiceProxy('/myo/myo_muscle0_controller/set_clt', myo_msgs.srv.SetClutch)
            self._clutchState = serv(not(self._clutchState)).clutchState
        except(rospy.ServiceException, e):
            sys.exit("Error! Cant Set Clutch")

    def _onClickLogger(self):
        """Start & Create or Stop & Delete Datalogger """
        if self._logState == False:
            self.logIt = logger.simpleLogger("guiLog",bufferSize=1000)
            self.log.setText("Running...")
            self._logState = True
        elif self._logState == True:
            self.log.setText("Log")
            self.logIt.__exit__(None,None,None)
            delattr(self,"logIt")
            self._logState = False

    def _onClick(self):
        """Callback for button to set clutchState """
        self.setClutch()
        if self._clutchState == 0:
            self.clutch.setText("Connect")
        else:
            self.clutch.setText("Disconnect")

    def _onChange(self):
        """Callback to change value of label that shows Torque"""
        #print "value requested ", button.value()
        if abs(self._before -self.button.value()) > self.increment :
            self.button.setValue(self._before - math.copysign(self.increment,(self._before-self.button.value() ) ) )
        self.bar.setValue(self.button.value())

        self._before =self.button.value()
        self.label.setText("<h2><b>" + str(self.button.value()*(15.0/1000.0 *93.0 *66.0 *0.5)) + " mNm </b></h2>")
        self.setReference(self.button.value())


if __name__ == '__main__':
    gui = OrthosisGUI([300,300,300,300])
    gui.show()

    # Enter Qt application main loop
