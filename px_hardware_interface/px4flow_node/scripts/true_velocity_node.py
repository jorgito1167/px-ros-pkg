#!/usr/bin/env python
import roslib; roslib.load_manifest('px4flow_node')
import rospy
import Math
import time
from std_msgs.msg import Float64
from px_comm.msg import OpticalFlow
from px4flow_node.msg import velocity
default_focal_length = 0.016
class NodeClass():
  
  def convert(self,msg):
        ground_conversion = (rospy.get_param('~ground_distance')/msg.ground_distance)       
        focal_length_conversion = (default_focal_length/rospy.get_param('~focal_length'))

        true_velocity_x = msg.velocity_x*ground_conversion*focal_length_conversion

        true_velocity_y = msg.velocity_y*ground_conversion*focal_length_conversion
        self.pub.publish(true_velocity_x, true_velocity_y)


        return [true_velocity_x, true_velocity_y]


  def filterVel(self,msg):
        current = self.convert(msg)
        alpha = rospy.get_param('~alpha')
        if self.prev == None:
            self.prev = current
        else:
            for j in xrange(len(self.out)):
                self.out[j] = self.prev[j] + alpha*(current[j] - self.prev[j])
            self.prev = self.out
            self.pub1.publish(self.out[0], self.out[1])
            self.pub3.publish(math.degrees(math.atan(-self.out[1]/self.out[0])))

  def __init__(self):
      self.prev = None
      self.out = [0,0]
      self.pub = rospy.Publisher('true_velocity', velocity)
      self.pub1 = rospy.Publisher('filter_velocity', velocity)
      self.pub3 = rospy.Publisher('belt_angle', Float64)
      rospy.init_node('true_velocity_node')
      rospy.Subscriber('/px4flow/opt_flow',OpticalFlow, self.filterVel)
  
if __name__ == '__main__':
  node = NodeClass()
  while not rospy.is_shutdown():
      time.sleep(0.01)

