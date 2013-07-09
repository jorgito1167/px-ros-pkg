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
from px4flow_node.msg import velocity
class Arduino():
    
    def __init__(self):
        self.portOpened = False
        self.inp = 0.0
        self.pub = rospy.Publisher('wheel_velocity', Float64)
        self.pub2 = rospy.Publisher('ratio', Float64)
        rospy.init_node('arduino_node')
        self.sub = rospy.Subscriber("true_velocity",velocity , self.ratio)
        
    def ratio(self, msg):
        rospy.loginfo('ok')
        if self.inp!=0:
            self.pub2.publish(msg.velocity_x/self.inp)
        else:
            self.pub2.publish(0.0)
    def run(self):
        self.portOpened = self.connect()
        time.sleep(4)
        if (self.portOpened):
            while not rospy.is_shutdown():
                self.readData()
                self.publish(self.inp)
                time.sleep(0.01)

    def readData(self):
        inp = self.port.readline().rstrip('\n')
        try:
          self.inp = float(inp)
        except:
          rospy.loginfo("not a valid float")
          
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
