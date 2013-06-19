import roslib; roslib.load_manifest('px4flow_node')
import rospy
from std_msgs.msg import String


def callback(data):
    rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("px4flow/opt_flow", String, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
