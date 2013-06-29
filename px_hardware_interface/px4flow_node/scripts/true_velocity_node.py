#!/usr/bin/env python
import roslib; roslib.load_manifest('px4flow_node')
import rospy
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


  def __init__(self):
      self.pub = rospy.Publisher('true_velocity', velocity)
      rospy.init_node('true_velocity_node')
      rospy.Subscriber('/px4flow/opt_flow',OpticalFlow, self.convert)
  
if __name__ == '__main__':
  node = NodeClass()
  while not rospy.is_shutdown():
      time.sleep(0.01)

