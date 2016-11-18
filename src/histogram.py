#!/usr/bin/env python
import rospy
import csv
import myo_msgs.msg

class simpleListener:
    def __init__(self):
        self.maxdt = 0
        self.avgdt = 0
        self.count = 0
        self.avgVel = 0;
        self.maxVel = 0;
        self.nrBins = 100
        self.binRange = 100000
        self.targetT = 1000000

        # if binNr is not even, make it even
        if (self.nrBins % 2 == 1):
            self.nrBins += 1

        # calculate size of bin Target
        self.binSizeT = 2*self.binRange/self.nrBins

        # init list of bins
        self.bucket = [0] *(self.nrBins+1)

    def callback(self,data):
        #print "velocity was ", data.velocity
        self.avgdt = 0.01*data.dt.nsecs + 0.99*self.avgdt
        self.avgVel = 0.01*data.velocity + 0.99*self.avgVel

        # calculate binNr of sample
        tmp=((data.dt.nsecs + data.dt.secs*1000000000) - self.targetT)//self.binSizeT
        #print "Tmp was ", tmp
        if tmp < -self.nrBins//2:
            tmp = -self.nrBins//2
        elif tmp > self.nrBins//2:
            tmp = self.nrBins//2
        self.bucket[self.nrBins//2 + tmp] += 1

        if self.count > 10000:
            print "-------------------------------------"
            print "----------- New Histogram -----------"
            print "-------------------------------------"
            print " "
            self.count = 0
            for i in xrange(0,(self.nrBins)/2+1):
                print ((i-self.nrBins//2)*self.binSizeT/1000) , " us : " , self.bucket[i] , " | " , ( -1*(i-self.nrBins//2)*self.binSizeT/1000 ) , " us : " , self.bucket[self.nrBins -i]
            with open('data.csv','wb') as csvfile:
                writer = csv.writer(csvfile,delimiter='\t', quotechar='\'',quoting=csv.QUOTE_MINIMAL)
                writer.writerow(['NrBins',self.nrBins, 'binRange', self.binRange, 'Target',self.targetT])
                writer.writerow(['Bin', 'SampleNr'])
                for i in xrange(0,self.nrBins+1):
                    writer.writerow([i-self.nrBins//2, self.bucket[i] ])

        #print self.count
        self.count +=1





    def listener(self):
        rospy.init_node('listener',anonymous=True)
        print "starting up logger ..."

        rospy.Subscriber("/myo/myo_muscle0_controller/DebugMessage",myo_msgs.msg.statusMessage,self.callback)

        rospy.spin()

if __name__ == '__main__':
    a = simpleListener()
    a.listener()
