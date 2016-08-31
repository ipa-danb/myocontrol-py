#!/usr/bin/env python
import rospy
from math import *
import csv
from myo_msgs.msg import statusMessage
from std_msgs.msg import Float64

class simpleListener:
    def __init__(self):
        self.lcalcVel   = 0
        self.lboardVel  = 0
        self.avgDiff    = 0
        self.posBefore  = 0
        self.errors     = 0
        self.count      = 0

    def callback(self,data):
        #print "velocity was ", data.velocity
        if self.posBefore == 0:
            self.posBefore = data.position
        if data.position == self.posBefore:
            #print "Error! position stays same!?"
            self.errors += 1

        self.lcalcVel   = (data.position - self.posBefore)#/(data.dt.nsecs/1e9 + data.dt.secs)
        self.lboardVel = 0.5 * self.lboardVel + 0.5*data.velocity
        self.pub.publish(self.lcalcVel)

        if self.count > 10000:
            print "Nr of Errors: ", self.errors
            self.count = 0
        self.posBefore = data.position

    def listener(self):
        rospy.init_node('listener',anonymous=True)
        print "starting up logger ..."

        self.pub = rospy.Publisher('VelDiff', Float64, queue_size=10)
        rospy.Subscriber("/myo/myo_muscle0_controller/DebugMessage",statusMessage,self.callback)

        rospy.spin()

if __name__ == '__main__':
    a = simpleListener()
    a.listener()
