

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
  MatrixWrapper::ColumnVector filter_estimate_old_vec_;
  
  MatrixWrapper::ColumnVector belt_meas_, belt_meas_old_;
  bool filter_initialized_, belt_initialized_;


}; // class

}; // namespace

#endif
