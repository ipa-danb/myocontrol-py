#!/usr/bin/env python
import rospy
import time
from myo_msgs.srv import SetReference
from myo_msgs.srv import SetClutch
import sys

def setReference(ref):
    """Calls RosService /myo/myo_muscle0_controller/set_reference to set reference """
    rospy.wait_for_service('/myo/myo_muscle0_controller/set_reference')
    try:
        sDsp = rospy.ServiceProxy('/myo/myo_muscle0_controller/set_reference',SetReference)
        sDsp(ref)
    except rospy.ServiceException, e:
        print " "


if __name__ == '__main__':
    # parameter
    maxRef  = 3998
    zRef    = 0
    wTime   = 8
    runs    = 4


    # set clutch to decoupling
    rospy.wait_for_service('/myo/myo_muscle0_controller/set_clutch')

    try:
        serv = rospy.ServiceProxy('/myo/myo_muscle0_controller/set_clutch', SetClutch)
        serv(0)
    except rospy.ServiceException, e:
        sys.exit("Error! Cant Set Clutch")

    print "starting to drive trajectory..."
    '''
    for i in range(0,runs):
        setReference(0)
        time.sleep(4)
        setReference(maxRef)
        time.sleep(4)

    setReference(0)
    '''
    setReference(-500)
    time.sleep(2)
    setReference(-1000)
    time.sleep(2)
    setReference(-2000)
    time.sleep(2)
    setReference(-2800)
    time.sleep(2)
    setReference(-3800)

    for i in range(-3800,3800,20):
        setReference(i)
        time.sleep(2)
    for i in range(3800,-3800,-20):
        setReference(i)
        time.sleep(2)

    setReference(-2000)
    time.sleep(2)
    setReference(0)
    time.sleep(2)


    setReference(-300)
    for k in range(0,runs):

        for i in range(-300,300,1):
            setReference(i)
            time.sleep(2)
        for i in range(300,-300,-1):
            setReference(i)
            time.sleep(2)


    print "finished with trajectory"
    time.sleep(2)
    setReference(0)
