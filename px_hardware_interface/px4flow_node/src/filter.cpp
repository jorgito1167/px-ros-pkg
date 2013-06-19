#include <px4flow_node/filter.h>

using namespace MatrixWrapper;
using namespace BFL;
using namespace std;
using namespace ros;


namespace estimation
{
  //constructor
  BeltEstimation::BeltEstimation()
  {
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

      // create gaussian
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

      // create the model
      sys_pdf = new LinearAnalyticConditionalGaussian(AB,system_Uncertainty);
      sys_model = new LinearAnalyticSystemModelGaussianUncertainty(sys_pdf);


      /*********************************
       * Initialise measurement model *
       ********************************/

      // create matrix H for linear measurement model
      Matrix H(1,2);
      H(1,1) = 1.0;
      H(1,2) = 0.0;
      H(2,1) = 0.0;
      H(2,2) = 1.0;

      // Construct the measurement noise (a scalar in this case)
      ColumnVector measNoise_Mu(2);
      measNoise_Mu(1) = 0.0;
      measNoise_Mu(2) = 0.0;

      SymmetricMatrix measNoise_Cov(1);
      measNoise_Cov(1,1) = 0.55;
      measNoise_Cov(1,2) = 0.0;
      measNoise_Cov(2,1) = 0.0;
      measNoise_Cov(2,2) = 0.55;
      Gaussian measurement_Uncertainty(measNoise_Mu, measNoise_Cov);

      // create the model
      belt_meas_pdf = new LinearAnalyticConditionalGaussian(H, measurement_Uncertainty);
      belt_meas_model = new LinearAnalyticMeasurementModelGaussianUncertainty(belt_meas_pdf);

      /****************************
       * Linear prior DENSITY     *
       ***************************/
       // Continuous Gaussian prior (for Kalman filters)
      ColumnVector prior_Mu(2);
      prior_Mu(1) = 0.0;
      prior_Mu(2) = 0.0;
      SymmetricMatrix prior_Cov(2);
      prior_Cov(1,1) = 0.0;
      prior_Cov(1,2) = 0.0;
      prior_Cov(2,1) = 0.0;
      prior_Cov(2,2) = 0.0;
      prior = new Gaussian(prior_Mu,prior_Cov);

      /******************************
       * Construction of the Filter *
       ******************************/
     filter = new ExtendedKalmanFilter(prior);

      input(1) = 0.0;
      input(2) = 0.0;
 };

  // update filter
  ColumnVector BeltEstimation::update(const ColumnVector meas)
  {
    ROS_INFO("%f",meas(1));
    filter->Update(sys_model,input, belt_meas_model,meas);
    posterior = filter->PostGet();
    mean = posterior -> ExpectedValueGet();
    return mean;
};


};

