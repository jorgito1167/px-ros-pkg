

#ifndef __filter__
#define __filter__

#include "ros/ros.h"
// bayesian filtering
#include <filter/extendedkalmanfilter.h>
#include <wrappers/matrix/matrix_wrapper.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>


namespace estimation
{

class BeltEstimation
{
public:
  /// constructor
  BeltEstimation();

  MatrixWrapper::ColumnVector update(const MatrixWrapper::ColumnVector meas);

private:
  // pdf / model / filter
  BFL::LinearAnalyticSystemModelGaussianUncertainty*       sys_model_;
  BFL::LinearAnalyticConditionalGaussian*                    sys_pdf_;

  BFL::LinearAnalyticConditionalGaussian*                 belt_meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty*  belt_meas_model_;
  BFL::Gaussian*                                          prior_;
  BFL::ExtendedKalmanFilter*                              filter_;
  MatrixWrapper::SymmetricMatrix                          odom_covariance_, imu_covariance_, vo_covariance_;

  // vars
  BFL::Pdf<MatrixWrapper::ColumnVector> * posterior;
  
  ros::Time filter_time_old_;


}; // class

}; // namespace

#endif
