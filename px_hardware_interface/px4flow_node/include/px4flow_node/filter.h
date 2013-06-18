

#ifndef __filter__
#define __filter__

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

  /// destructor
  virtual ~BeltEstimation();

  /** update the extended Kalman filter
   * \param filter_time update the ekf up to this time
   * \param diagnostics_res returns false if the diagnostics found that the sensor measurements are inconsistent
   * returns true on successful update
   */
  bool update(const ros::Time& filter_time, bool& diagnostics_res);

  /** initialize the extended Kalman filter
   * \param prior the prior robot pose
   * \param time the initial time of the ekf
   */
  void initialize(const MatrixWrapper::ColumnVector& prior, const ros::Time& time);

  /** check if the filter is initialized
   * returns true if the ekf has been initialized already
   */
  bool isInitialized() {return filter_initialized_;};

  /** get the filter posterior
   * \param estimate the filter posterior as a columnvector
   */
  void getEstimate(MatrixWrapper::ColumnVector& estimate);


  /** Add a sensor measurement to the measurement buffer
   * \param meas the measurement to add
   */
  void addMeasurement(const MatrixWrapper::ColumnVector& meas, const ros::Time& time);

private:

  // pdf / model / filter
  BFL::LinearAnalyticSystemModelGaussianUncertainty            sys_model_;
  BFL::LinearAnalyticConditionalGaussian                 sys_pdf_;

  BFL::LinearAnalyticConditionalGaussian                 belt_meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty belt_meas_model_;
  BFL::Gaussian*                                          prior_;
  BFL::ExtendedKalmanFilter*                              filter_;
  MatrixWrapper::SymmetricMatrix                          odom_covariance_, imu_covariance_, vo_covariance_;

  // vars
  MatrixWrapper::ColumnVector vel_desi_, filter_estimate_old_vec_;
  tf::Transform filter_estimate_old_;
  MatrixWrapper::ColumnVector belt_meas_, belt_meas_old_;
  ros::Time filter_time_old_;
  bool filter_initialized_, belt_initialized_;

  // diagnostics
  double diagnostics_odom_rot_rel_;


}; // class

}; // namespace

#endif
