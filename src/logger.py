import rospy
import time
import csv
import sys
import myo_msgs.msg



class simpleLogger:
    """Simple Logger class

    Logger for rostopic message of type statusMessage (contained in MYO Robotics ROS Package)

    .. warning:: It logs data as long as it exists! It is recommended to only use it with a ressource manager.

    Example
    -------
    Usually you would call this logger by using the ressource manager:

    ::

        import logger
        with simpleLogger('FileName') as log:
            #do something here, e.g. time.wait(10)


    Notes
    -----
    statusMessage has the following structure:

    ::

        time     time
        float64  position
        float64  velocity
        float64  displacement
        float64  reference
        float64  current
        float64  commanded_effort
        float64  analogIN0
        int32    clutchState


    """

    def __init__(self,name, bufferSize=1000, rosNode="/myo/myo_muscle0_controller/DebugMessage"):
        """Init function for simpleLogger class

        Parameters
        ----------
        name : string
            Name for logging file
        bufferSize : int
            Size of logging buffer. After bufferSize samples the buffer is written to the file
        rosNode : string
            ROS Node that gets logged
            Notes
            -----
            Needs to be to be of type statusMessage

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
        self.subsC = rospy.Subscriber(rosNode,myo_msgs.msg.statusMessage,self.callback)

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
        """Callback function for a ROS Message

        This function fills a buffer up to Size `bufferSize` and writes them to their respective file

        .. warning:: This function gets called automatically as long as this object exists. Hence it is recommended to use it with a ressource manager


        Parameters
        ----------
        data : <statusMessage>
            writes `data` to buffer

        """
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
