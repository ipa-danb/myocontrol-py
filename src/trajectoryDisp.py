#!/usr/bin/env python
import rospy
import time
from myo_msgs.srv import SetReference
from myo_msgs.srv import SetClutch
import sys, getopt
import threading
import logger
import csv

class TrajectoryMover:
    def __init__(self,bufferSize=1000,fileName="traj_1",directory="scripts",logFileName=None):
        self.trajFileName = fileName
        self.maxLines = 0
        self.checkFile()
        self.directory = directory
        self.bufferSize = bufferSize
        if logFileName == None:
            self.logFileName = fileName

    def setReference(self,ref):
        """Calls RosService /myo/myo_muscle0_controller/set_reference to set reference """
        rospy.wait_for_service('/myo/myo_muscle0_controller/set_ref')
        try:
            sDsp = rospy.ServiceProxy('/myo/myo_muscle0_controller/set_ref',SetReference)
            sDsp(ref)
        except rospy.ServiceException, e:
            print " "

    def checkFile(self):
        with open('scripts/' +self.trajFileName,'rb') as csvfile:
            trajReader = csv.reader(csvfile,delimiter=',',quotechar='|')
            self.maxLines = sum(1 for row in trajReader)
            for row in trajReader:
                if row[0] == 'clt':
                    if not (int(row[1]) == 0 or int(row[1])== 1):
                        raise Exception("Trajectory violated bounds at " + str(row))
                else:
                    row =  [int(row[0]), float(row[1])]
                    if not (int(row[0]) > 0 and int(row[0]) < 300 and int(row[1]) > 0):
                        raise Exception("Trajectory violated bounds at " + str(row))

    def setClutch(self,state):
        rospy.wait_for_service('/myo/myo_muscle0_controller/set_clt')
        try:
            serv = rospy.ServiceProxy('/myo/myo_muscle0_controller/set_clt', SetClutch)
            serv(state)
        except rospy.ServiceException, e:
            sys.exit("Error! Cant Set Clutch")

    def setTrajectoryFile(self, filename):
        self.trajFileName = filename
        self.checkFile()

    def driveTrajectory(self):
        self.checkFile()


        print "###############################"
        print "setting up logger with file " + name + " ..."
        with logger.simpleLogger(self.logFileName,self.bufferSize) as log:
            print "starting to drive trajectory..."
            print " "
            print "###############################"
            print "Transitions"
            print "###############################"
            with open(self.directory + '/' +self.trajFileName,'rb') as csvfile:
                trajReader = csv.reader(csvfile,delimiter=',',quotechar='|')
                for row in trajReader:
                    print('\rLines: {:4d} / {:4d} : {:20s}'.format(trajReader.line_num,self.maxLines,str(row) )),
                    sys.stdout.flush()
                    if row[0] == 'clt':
                        self.setClutch(int(row[1]))
                    else:
                        #print "Ref: " + str(row[0])
                        self.setReference(int(row[0]))
                        time.sleep(float(row[1]))
            print "\n"
            print "finished with trajectory"

    def changeDirectory(self,directory):
        self.directory = directory

if __name__ == '__main__':

    # default parameter
    name = ""
    trajFileName = "traj_1"
    bufferSize = 2000
    maxRef      = 120
    minRef      = 10
    t_total     = 60

    try:
        opts, args = getopt.getopt(sys.argv[1:],"hf:b:t:",["trajectoryFile=","help","file=","buffer="])
    except getopt.GetoptError:
        print 'test.py -i <inputfile> -o <outputfile>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'trajectoryDisp.py -t <trajectoryFile> -f <logfilename> -b <logbuffer>'
            sys.exit()
        elif opt in ("-f", "--file"):
            name = str(arg)
        elif opt in ("-b", "--buffer"):
            bufferSize = int(arg)
        elif opt in ("-t", "--trajectoryFile"):
            trajFileName = str(arg)

    t = TrajectoryMover(bufferSize=bufferSize,fileName=trajFileName)
    t.driveTrajectory()
