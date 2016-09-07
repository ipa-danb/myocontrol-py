#!/usr/bin/env python
import rospy
from math import *
import time
import csv
from myo_msgs.msg import statusMessage

class simpleLogger:
    def __init__(self):
        # create filename from date
        timestr = time.strftime("%Y%m%d-%H%M%S")
        self.fileName = "data-" + timestr
        with open(self.fileName,'wb') as csvfile:
            writer = csv.writer(csvfile,delimiter='\t', quotechar='\'',quoting=csv.QUOTE_MINIMAL)
            writer.writerow( [ "dt.nsecs", "dt.secs", "position", "velocity", "velocity_ref", "displacement", "displacement_ref", "current","commanded_effort","analogIN0",'clutchState' ] )
        self.bufferSize = 100
        self.count = 0
        self.buffer = []
    def callback(self,data):
        self.buffer.append((data.dt.nsecs,data.dt.secs,data.position,data.velocity,data.velocity_ref,data.displacement,data.displacement_ref,data.current,data.commanded_effort,data.analogIN0,data.clutchState))
        self.count +=1
        if self.count > self.bufferSize:
            # reset counter
            self.count = 0
            # write new data and append it
            with open(self.fileName,'a') as csvfile:
                writer = csv.writer(csvfile,delimiter='\t', quotechar='\'',quoting=csv.QUOTE_MINIMAL)
                print "logging data..."
                for row in self.buffer:
                    writer.writerow(row)

                # clear buffer
                self.buffer = []


    def listener(self):
        rospy.init_node('listener',anonymous=True)
        print "starting up logger ..."
        rospy.Subscriber("/myo/myo_muscle0_controller/DebugMessage",statusMessage,self.callback)
        rospy.spin()



if __name__ == '__main__':
    a = simpleLogger()
    a.listener()
