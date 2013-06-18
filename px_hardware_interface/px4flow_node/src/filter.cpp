#include <px4flow_node/filter.h>

using namespace MatrixWrapper;
using namespace BFL;
using namespace std;
using namespace ros;


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

    sys_pdf_ = new LinearAnalyticConditionalGaussian(A, system_Uncertainty);
    sys_model_ = new LinearAnalyticSystemModelGaussianUncertainty(sys_pdf_);

    //create MEASUREMENT MODEL

    Matrix H(2,2);
    H(1,1) = 1.0;
    H(1,2) = 0.0;
    H(2,1) = 0.0;
    H(2,2) = 1.0;


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

    belt_meas_pdf_ = new LinearAnalyticConditionalGaussian(H, measurement_Uncertainty);
    belt_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(belt_meas_pdf_);

    };

  
};

