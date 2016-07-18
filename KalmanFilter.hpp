#include "PTPparameters.hpp"
#include "PTPUtil.hpp"

namespace soildynamics{

class KalmanFilter{
	
public:
	KalmanFilter();
	~KalmanFilter();
	
	vecD kalman(vecD input, int s, PTPUtil* ut);
	double kalmanD(double input, int s);
	
};

}// end of namespace soildynamics
