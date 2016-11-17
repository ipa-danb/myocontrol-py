#!/usr/bin/env python
import rospy
import myo_msgs.srv

class rosServiceCaller:
    def __init__(self,controllerName='myo_muscle0_controller'):
        self.controllerName = controllerName
        self.path = '/myo/' + controllerName
        self.setVals = list()
        # check myo_msgs.srv for new stuff
        for i in dir(myo_msgs.srv):
            if "Set" in i and not any(sstr in i for sstr in ["Request","Response","_"]):
                self.setVals.append(i)

    def set(self,name,ref):
        """Calls RosService /myo/myo_muscle0_controller/set_reference to set reference """
        rospy.wait_for_service('/myo/myo_muscle0_controller/set_ref')
        try:
            sDsp = rospy.ServiceProxy('/myo/myo_muscle0_controller/set_ref',SetReference)
            sDsp(ref)
        except rospy.ServiceException, e:
            print " "
