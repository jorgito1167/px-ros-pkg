#include <px4flow_node/filter.h>

using namespace std;
using namespace MatrixWrapper;
using namespace ros;
using namespace BFL;



namespace estimation
{
  //constructor
  BeltEstimation::BeltEstimation():
    prior_(NULL),
    filter_(NULL),
    filter_initialized_(false),
    belt_initialized_(false)
  {
    //create the SYSTEM MODEL

    Matrix A(2,2);
    A(1,1) = 1.0;
    A(1,2) = 0.0;
    A(2,1) = 0.0;
    A(2,2) = 1.0;

    Matrix B(2,2);
    B(1,1) = 0.0;
    B(1,2) = 0.0;
    B(2,1) = 0.0;
    B(2,2) = 0.0;

    vector<Matrix> AB(2);

    AB[0] = A;
    AB[1] = B;

    ColumnVector sysNoise_Mu(2);
    sysNoise_Mu(1) = 0.0;
    sysNoise_Mu(2) = 0.0;
     
    SymmetricMatrix sysNoise_Cov(2);
    sysNoise_Cov = 0.0;
    sysNoise_Cov(1,1) = 0.0;
    sysNoise_Cov(1,2) = 0.0;
    sysNoise_Cov(2,1) = 0.0;
    sysNoise_Cov(2,2) = 0.0;

    Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);

    LinearAnalyticConditionalGaussian sys_pdf_(AB, system_Uncertainty);
    LinearAnalyticSystemModelGaussianUncertainty sys_model_(&sys_pdf);

    //create MEASUREMENT MODEL

    Matrix H(2,2);
    B(1,1) = 1.0;
    B(1,2) = 0.0;
    B(2,1) = 0.0;
    B(2,2) = 1.0;


    ColumnVector measNoise_Mu(2);
    measNoise_Mu(1) = 0.0;
    measNoise_Mu(2) = 0.0;
     
    SymmetricMatrix measNoise_Cov(2);
    measNoise_Cov = 0.0;
    measNoise_Cov(1,1) = 0.55;
    measNoise_Cov(1,2) = 0.0;
    measNoise_Cov(2,1) = 0.0;
    measNoise_Cov(2,2) = 0.55;

    Gaussian measurement_Uncertainty(measNoise_Mu, measNoise_Cov);

    LinearAnalyticConditionalGaussian belt_meas_pdf_(H, measurement_Uncertainty);
    LinearAnalyticSystemModelGaussianUncertainty belt_meas_model_(&meas_pdf);

    };

   //destructor
   BeltEstimation::~BeltEstimation(){
      if (filter_) delete filter_;
      if (prior_)  delete prior_;
      delete belt_meas_model_;
      delete belt_meas_pdf_;
      delete sys_pdf_;
      delete sys_model_;
  };

  void BeltEstimation::initialize(const ColumnVector& prior, const Time& time)
    {
      ColumnVector prior_Mu(2);
      prior_Mu(1) = 0.0; 
      prior_Mu(2) = 0.0;

      SymmetricMatrix prior_Cov(2);
      prior_Cov(1,1) = 0.0;
      prior_Cov(1,2) = 0.0;
      prior_Cov(2,1) = 0.0;
      prior_Cov(2,2) = 0.0;

      prior_  = new Gaussian(prior_Mu,prior_Cov);
      filter_ = new ExtendedKalmanFilter(prior_);

      // remember prior
      addMeasurement(prior,time)
      filter_estimate_old_vec_ = prior_Mu;
      filter_estimate_old_ = prior;
      filter_time_old_     = time;

      // filter initialized
      filter_initialized_ = true;
    };


  // get latest filter posterior as vector
  void BeltEstimation::getEstimate(ColumnVector& estimate)
  {
    estimate = filter_estimate_old_vec_;
  };

  bool BeltEstimation::update(bool belt_active, const Time&  filter_time)
    {
      // only update filter when it is initialized
      if (!filter_initialized_){
        ROS_INFO("Cannot update filter when filter was not initialized first.");
        return false;
      }

      // only update filter for time later than current filter time
      double dt = (filter_time - filter_time_old_).toSec();
      if (dt == 0) return false;
      if (dt <  0){
        ROS_INFO("Will not update robot pose with time %f sec in the past.", dt);
        return false;
      }
      ROS_DEBUG("Update filter at time %f with dt %f", filter_time.toSec(), dt);


      // system update filter
      // --------------------
      filter_->Update(&sys_model_, &belt_meas_model,);

      
      // process belt measurement
      // ------------------------
      ROS_DEBUG("Process belt meas");
      if (belt_active){
      // remember last estimate
      filter_estimate_old_vec_ = filter_->PostGet()->ExpectedValueGet();
        }
      }

      return true;
  };



};

