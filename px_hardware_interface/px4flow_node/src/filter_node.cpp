#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <px_comm/OpticalFlow.h>

using namespace MatrixWrapper;
using namespace estimation;

class NodeClass
{
  public:
      
      ros::NodeHandle n;
      ros::Publisher filter_pub;
      ros::Subscriber filter_sub;
      std_msgs::Float32 newMsg;
      BeltEstimation beltEstimator;
  //Constructor
  NodeClass()
  {
      filter_pub = n.advertise<std_msgs::Float32>("ok",5);
      filter_sub = n.subscribe("/px4flow/opt_flow",5, &NodeClass::kalmanFilter,this);
  }
  //Destructor
  ~NodeClass()
  {
  }
    void kalmanFilter(px_comm::OpticalFlow msg)
    {
        
        newMsg.data= currentEstimate(2);
        filter_pub.publish(newMsg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "filter_node");
    NodeClass nodeClass;
    ros::spin();

    return 0;
}

