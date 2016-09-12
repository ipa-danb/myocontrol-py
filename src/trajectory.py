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
    maxRef  = 50
    zRef    = 0
    wTime   = 10
    runs    = 10

    print "starting to drive trajectory... "

    for i in range(0,runs):
        print "Run ", i , " of ", runs
        setReference(zRef)
        time.sleep(wTime)

        setReference(maxRef)
        time.sleep(wTime)

        setReference(zRef)
        time.sleep(wTime)

        setReference(-maxRef)
        time.sleep(wTime)

    setReference(zRef)
    print "finished with trajectory"
