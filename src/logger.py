#!/usr/bin/env python
import rospy
from myo_msgs.msg import statusMessage

class simpleListener:
    def __init__(self):
        self.maxdt = 0
        self.avgdt = 0
        self.count = 0
        self.avgVel = 0;
        self.maxVel = 0;

    def callback(self,data):
        #print "velocity was ", data.velocity
        self.avgdt = 0.01*data.dt.nsecs + 0.99*self.avgdt
        self.avgVel = 0.01*data.velocity + 0.99*self.avgVel
        if abs(data.dt.nsecs + data.dt.secs*1e9) >self.maxdt:
            self.maxdt = abs(data.dt.nsecs + data.dt.secs*1e9)
            print "New self.maxdt! ",self.maxdt/1e6 , " ms"
        if abs(data.velocity) >self.maxVel:
            self.maxVel = abs(data.velocity)
            print "New self.maxVel! ",self.maxVel , " Enc/s"
        if self.count>10000:
            print "Average: ", self.avgdt/1e6 , " ms | Maximum: ",self.maxdt/1e6 , " ms"
            print "Average: ", self.avgVel , " Enc/s | Maximum: ",self.maxVel , " Enc/s"
            self.count = 0;
        self.count = self.count +1

    def listener(self):
        rospy.init_node('listener',anonymous=True)


        rospy.Subscriber("/myo/myo_muscle0_controller/DebugMessage",statusMessage,self.callback)

        rospy.spin()

if __name__ == '__main__':
    a = simpleListener()
    a.listener()
