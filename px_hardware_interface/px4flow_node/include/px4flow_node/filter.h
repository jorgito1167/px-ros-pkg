

#ifndef __filter__
#define __filter__

// bayesian filtering
#include <filter/extendedkalmanfilter.h>
#include <wrappers/matrix/matrix_wrapper.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>
#include "nonlinearanalyticconditionalgaussianodo.h"


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
  void initialize(const tf::Transform& prior, const ros::Time& time);

  /** check if the filter is initialized
   * returns true if the ekf has been initialized already
   */
  bool isInitialized() {return filter_initialized_;};

  /** get the filter posterior
   * \param estimate the filter posterior as a columnvector
   */
  void getEstimate(MatrixWrapper::ColumnVector& estimate);

  /** get the filter posterior
   * \param time the time of the filter posterior
   * \param estimate the filter posterior as a tf transform
   */
  void getEstimate(ros::Time time, tf::Transform& estiamte);

  /** get the filter posterior
   * \param time the time of the filter posterior
   * \param estimate the filter posterior as a stamped tf transform
   */
  void getEstimate(ros::Time time, tf::StampedTransform& estiamte);

  /** get the filter posterior
   * \param estimate the filter posterior as a pose with covariance
   */
  void getEstimate(geometry_msgs::PoseWithCovarianceStamped& estimate);

  /** Add a sensor measurement to the measurement buffer
   * \param meas the measurement to add
   */
  void addMeasurement(const tf::StampedTransform& meas);

  /** Add a sensor measurement to the measurement buffer
   * \param meas the measurement to add
   * \param covar the 6x6 covariance matrix of this measurement, as defined in the PoseWithCovariance message
   */
  void addMeasurement(const tf::StampedTransform& meas, const MatrixWrapper::SymmetricMatrix& covar);

private:
  /// correct for angle overflow
  void angleOverflowCorrect(double& a, double ref);

  // decompose Transform into x,y,z,Rx,Ry,Rz
  void decomposeTransform(const tf::StampedTransform& trans,
			  double& x, double& y, double&z, double&Rx, double& Ry, double& Rz);
  void decomposeTransform(const tf::Transform& trans,
			  double& x, double& y, double&z, double&Rx, double& Ry, double& Rz);


  // pdf / model / filter
  BFL::AnalyticSystemModelGaussianUncertainty*            sys_model_;
  BFL::NonLinearAnalyticConditionalGaussianOdo*           sys_pdf_;
  BFL::LinearAnalyticConditionalGaussian*                 odom_meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* odom_meas_model_;
  BFL::LinearAnalyticConditionalGaussian*                 imu_meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* imu_meas_model_;
  BFL::LinearAnalyticConditionalGaussian*                 vo_meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* vo_meas_model_;
  BFL::Gaussian*                                          prior_;
  BFL::ExtendedKalmanFilter*                              filter_;
  MatrixWrapper::SymmetricMatrix                          odom_covariance_, imu_covariance_, vo_covariance_;

  // vars
  MatrixWrapper::ColumnVector vel_desi_, filter_estimate_old_vec_;
  tf::Transform filter_estimate_old_;
  tf::StampedTransform odom_meas_, odom_meas_old_, imu_meas_, imu_meas_old_, vo_meas_, vo_meas_old_;
  ros::Time filter_time_old_;
  bool filter_initialized_, odom_initialized_, imu_initialized_, vo_initialized_;

  // diagnostics
  double diagnostics_odom_rot_rel_, diagnostics_imu_rot_rel_;

  // tf transformer
  tf::Transformer transformer_;

}; // class

}; // namespace

#endif
