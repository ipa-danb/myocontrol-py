#!/usr/bin/env python
import rospy
import time
from myo_msgs.srv import SetReference


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
    maxRef  = 1000
    zRef    = 0
    wTime   = 8
    runs    = 10

    print "starting to drive trajectory... "
    setReference(-2000)
    time.sleep(2)
    setReference(-3889)

    for i in range(-3800,3800,50):
        setReference(i)
        time.sleep(1)

    setReference(2000)
    time.sleep(2)
    setReference(0)
    time.sleep(2)
    setReference(-300)
    for i in range(-300,300):
        setReference(i)
        time.sleep(0.5)
    for i in range(300,-300):
        setReference(i)
        time.sleep(0.5)

    print "finished with trajectory"
    time.sleep(2)
    setReference(0)
