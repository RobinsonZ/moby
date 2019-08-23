#!/usr/bin/python
import roslib
roslib.load_manifest("moby")
import rospy, navio.ms5611
from std_msgs.msg import Header
from sensor_msgs.msg import FluidPressure, Temperature

# Source for RingBuffer: https://www.oreilly.com/library/view/python-cookbook/0596001673/ch05s19.html
class RingBuffer:
    """ class that implements a not-yet-full buffer """
    def __init__(self,size_max):
        self.max = size_max
        self.data = []

    class __Full:
        """ class that implements a full buffer """
        def append(self, x):
            """ Append an element overwriting the oldest one. """
            self.data[self.cur] = x
            self.cur = (self.cur+1) % self.max
        def get(self):
            """ return list of elements in correct order """
            return self.data[self.cur:]+self.data[:self.cur]

    def append(self,x):
        """append an element at the end of the buffer"""
        self.data.append(x)
        if len(self.data) == self.max:
            self.cur = 0
            # Permanently change self's class from non-full to full
            self.__class__ = self.__Full

    def get(self):
        """ Return a list of elements from the oldest to the newest. """
        return self.data
        
    
def baro():
    rate = rospy.Rate(10)
    
    baro = navio.ms5611.MS5611()
    baro.initialize()
        
    pressure_pub = rospy.Publisher("baro/pressure", FluidPressure)
    temp_pub = rospy.Publisher("baro/temp", Temperature)
    ringbuf_size = rospy.get_param("~moving_avg_size", 50)
    presbuffer = RingBuffer(ringbuf_size)
    
    rospy.loginfo("Starting barometer")
    while not rospy.is_shutdown():
        baro.refreshPressure()
        rospy.sleep(0.01) # wait 10ms per Navio docs
        baro.readPressure()
        
        baro.refreshTemperature()
        rospy.sleep(0.01) # wait 10ms per Navio docs
        baro.readTemperature()
        
        baro.calculatePressureAndTemperature()
        
        temp = baro.TEMP
        pressure = baro.PRES * 100 # convert millibars to Pascals
        
        presbuffer.append(pressure)
        
        pressurelist = presbuffer.get()
        # average it out
        avg_pressure = sum(pressurelist) / len(pressurelist)
        
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        pressure_pub.publish(FluidPressure(header, avg_pressure, 0))
        temp_pub.publish(Temperature(header, temp, 0))
        
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node('baro')
    try:
        baro()
    except rospy.ROSInterruptException: 
        pass