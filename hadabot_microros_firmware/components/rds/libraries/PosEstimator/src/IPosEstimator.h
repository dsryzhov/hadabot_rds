#ifndef __IPOS_ESTIMATOR
#define __IPOS_ESTIMATOR

class IPosEstimator {
public:
    virtual void positionUpdateCallback(double measure_time, double measure_delta_time);    
};

#endif