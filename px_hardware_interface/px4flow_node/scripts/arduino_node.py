#!/usr/bin/env python
import sys
sys.path.append("../../lib")
import serial, time
import threading, thread
import roslib; roslib.load_manifest('px4flow_node')
import rospy
import serial
from std_msgs.msg import Float64
from std_msgs.msg import String

class Arduino():
    
    def __init__(self):
        self.portOpened = False
        self.previousInput = 0.0
        self.pub = rospy.Publisher('wheel_velocity', Float64)
        rospy.init_node('arduino_node')
        
    def run(self):
        self.portOpened = self.connect()
        time.sleep(4)
        if (self.portOpened):
            while not rospy.is_shutdown():
                inp = self.readData()
                self.publish(inp)
                time.sleep(0.01)

    def readData(self):
        inp = self.port.readline().rstrip('\n')
        rospy.loginfo(str(inp))
        try:
          inp = float(inp)
          self.previousInput= inp
        except:
          rospy.loginfo("not a valid float")
          return self.previousInput
        return inp
    def publish(self,msg):
            self.pub.publish(msg) 

    def connect(self):
        print "Connecting"
        if self.portOpened: self.close()
        # Loop through possible values of ACMX, and try to connect on each one
        for i in range(4):
            try:
                # Try to create the serial connection
                self.port=serial.Serial(port='/dev/ttyACM1', baudrate=9600, timeout=0.5)
                if self.port.isOpen():
                    time.sleep(2) # Wait for Arduino to initialize
                    print "Connected"
                    return True
            except:
                # Some debugging prints
                print "Arduino not connected on ACM{0}".format(i)
        print "Failed to connect"
        return False

if __name__ == '__main__':
   try:
      ard = Arduino()
      ard.run()
   except rospy.ROSInterruptException:
       pass
