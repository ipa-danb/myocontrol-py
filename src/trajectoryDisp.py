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
        self.directory = directory
        self.bufferSize = bufferSize
        self.commands = {"mov": ('c',(5,200),self.setReference), "clt" : ('s',(0,1),self.setClutch) }
        self.logFileName = logFileName
        self.checkFile()


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
            for row in trajReader:
                # check if command is known
                if not self.commands.has_key(row[0]):
                    raise Exception("command unknown: " + str(row[0]) + " at Line " + str(trajReader.line_num))

                # check if command is in bounds
                bounds = self.commands.get(row[0])
                if bounds[0] == 'c':
                    if not ( float(row[1]) >= bounds[1][0] and float(row[1]) <= bounds[1][1] ):
                        raise Exception("command not in bound: " + str(row) + " vs bounds " + str(bounds[1][0:2]) + " in Line " + str(trajReader.line_num) )
                elif bounds[0] == 's':
                    if not ( float(row[1]) in bounds[1] ):
                        raise Exception("command not accepted value: " + str(row[1]) + " not in " + str(bounds[1]) + " in Line " + str(trajReader.line_num) )

                # check if time is in bound
                if len(row) > 2:
                    if (float(row[2]) < 0):
                        raise Exception("wait time lower than 0 in Line: " + str(trajReader.line_num))
            self.maxLines = trajReader.line_num


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

    def traj(self):
        print " "
        print "###############################"
        print "Starting to drive trajectory..."
        print "###############################\n"
        with open(self.directory + '/' +self.trajFileName,'rb') as csvfile:
            trajReader = csv.reader(csvfile,delimiter=',',quotechar='|')
            for row in trajReader:
                print('\rLines: {:4d} / {:4d} : {:20s}'.format(trajReader.line_num,self.maxLines,str(row[0]) + " : " + str(row[1]) )),
                sys.stdout.flush()
                self.commands.get(row[0])[2](float(row[1]))
                if len(row) > 2:
                    time.sleep(float(row[2]))
        print "\n"
        print "###############################"
        print "finished with trajectory"
        print "###############################"
        print "\n"

    def driveTrajectory(self):
        self.checkFile()
        print "\n"
        if self.logFileName == None:
            print "###############################"
            print "No logger specified ..."
            print "###############################"
            self.traj()
        else:
            print "###############################"
            print "setting up logger with file " + self.logFileName + " ..."
            print "###############################"
            with logger.simpleLogger(self.logFileName,self.bufferSize) as log:
                self.traj()



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

    t = TrajectoryMover(bufferSize=bufferSize,fileName=trajFileName,logFileName=name )
    t.driveTrajectory()
