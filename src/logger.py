"""
.. module:: logger
    :platform: Unix
    :synopsis: A module containing
.. moduleauthoer:: Daniel Bargmann <daniel.bargmann@ipa.fraunhofer.de>

"""

import rospy
from math import *
import time
import csv
import sys
from myo_msgs.msg import statusMessage

'''
time     time
float64  position
float64  velocity
float64  displacement
float64  reference
float64  current
float64  commanded_effort
float64  analogIN0
int32    clutchState
'''

class simpleLogger:
    """Simple Logger class

    This class is a simple logger, which takes

    Attributes
    ----------
    fileName : string
    bufferSize : int
    count : int
    dataSize : int
    buffer : array



    """

    def __init__(self,name, bufferSize=1000, rosNode="/myo/myo_muscle0_controller/DebugMessage"):
        """

        """
        # create filename from date
        timestr = time.strftime("%Y%m%d-%H%M%S")
        try:
            os.mkdir("data")
        except Exception:
            pass
        self.fileName =  "data/" + name + "-" + timestr
        with open(self.fileName,'wb') as csvfile:
            writer = csv.writer(csvfile,delimiter='\t', quotechar='\'',quoting=csv.QUOTE_MINIMAL)
            writer.writerow( [ "time.nsecs", "time.secs", "position", "velocity", "displacement", "reference", "current","commanded_effort","analogIN0",'clutchState' ] )
        self.bufferSize = bufferSize
        self.dataSize = 0
        self.count = 0
        self.buffer = []

        rospy.init_node('logger',anonymous=True)
        print "\nstarting up logger ..."
        self.subsC = rospy.Subscriber(rosNode,statusMessage,self.callback)

    def __enter__(self):
        return self

    def __exit__(self,exc_type, exc_val, exc_tb):
        print "#####################"
        print "stopping logger ... \n"
        self.dataSize += self.count
        with open(self.fileName,'a') as csvfile:
            writer = csv.writer(csvfile,delimiter='\t', quotechar='\'',quoting=csv.QUOTE_MINIMAL)
            print '{:10s} {:10d}'.format('Final logged #Samples:',self.dataSize)
            for row in self.buffer:
                writer.writerow(row)
        print "#####################\n"
        self.subsC.unregister()


    def callback(self,data):
        self.buffer.append((data.time.nsecs,data.time.secs,data.position,data.velocity,data.displacement,data.reference,data.current,data.commanded_effort,data.analogIN0,data.clutchState))
        self.count +=1
        if self.count > self.bufferSize:
            # reset counter
            self.count = 0
            # add to total counter
            self.dataSize += self.bufferSize
            # write new data and append it
            with open(self.fileName,'a') as csvfile:
                writer = csv.writer(csvfile,delimiter='\t', quotechar='\'',quoting=csv.QUOTE_MINIMAL)
                for row in self.buffer:
                    writer.writerow(row)

                # clear buffer
                self.buffer = []


if __name__ == '__main__':
    if len(sys.argv) == 2:
        name = sys.argv[1]
    else:
        print "Using standard name: data-[...]"
        name = "data"

    a = simpleLogger(name)

    # keep process alive
    rospy.spin()
