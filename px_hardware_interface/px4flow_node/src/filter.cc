#include <px4flow_node/filter.h>

using namespace std;
using namespace MatrixWrapper;
using namespace ros;
using namespace BFL;


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

LinearAnalyticConditionalGaussian sys_pdf(AB, system_Uncertainty);
LinearAnalyticSystemModelGaussianUncertainty sys_model(&sys_pdf);



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

LinearAnalyticConditionalGaussian meas_pdf(H, measurement_Uncertainty);
LinearAnalyticSystemModelGaussianUncertainty meas_model(&meas_pdf);

ColumnVector prior_Mu(2);
prior_Mu(1) = 0.0; 
prior_Mu(2) = 0.0;

SymmetricMatrix prior_Cov(2);
prior_Cov(1,1) = 0.0;
prior_Cov(1,2) = 0.0;
prior_Cov(2,1) = 0.0;
prior_Cov(2,2) = 0.0;

Gaussian prior(prior_Mu,prior_Cov);

ExtendedKalmanFilter filter(&prior);



