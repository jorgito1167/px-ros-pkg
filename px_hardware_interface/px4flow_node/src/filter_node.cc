#include "ros/ros.h"
#include "std_msgs/String.h"
#include <px_comm/OpticalFlow.h>
#include <bfl/filter/extendendkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/pdf/linearanalyticmeasurementmodel_gaussianuncertainty.h>

void kalmanFilter(px_comm::OpticalFlow msg)
{
  ROS_INFO("I heard:");
  }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "filter_node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/px4flow/opt_flow",1000, kalmanFilter);

    ros::spin();

    return 0;
}

