#!/usr/bin/env python
# -'''- coding: utf-8 -'''-

import sys

import curses
import rospy
from myo_msgs.srv import SetVelocity

#from PySide.QtCore import *
#from PySide.QtGui import *
#from PySide.QtDeclarative import QDeclarativeView

ref = 0

def main(stdscr):
    stdscr.nodelay(1)
    global ref
    while True:
        c = stdscr.getch()
        if c!= -1:
            if c == 259:
                ref += 1
            elif c == 258:
                ref += -1
            setReference(ref)
            stdscr.clear()
            stdscr.refresh()
            stdscr.move(0,0)
            print ref



def setReference(displacement):
    rospy.wait_for_service('/myo/myo_muscle0_controller/set_velocity')
    try:
        sDsp = rospy.ServiceProxy('/myo/myo_muscle0_controller/set_velocity',SetVelocity)
        sDsp(displacement)
    except rospy.ServiceException, e:
        print " "

if __name__ == "__main__":
    curses.wrapper(main)


# Create Qt application and the QDeclarative view
#app = QApplication(sys.argv)
#view = QDeclarativeView()
# Create an URL to the QML file
#url = QUrl('view.qml')
# Set the QML file and show
#view.setSource(url)
#view.show()
# Enter Qt main loop
#sys.exit(app.exec_())
