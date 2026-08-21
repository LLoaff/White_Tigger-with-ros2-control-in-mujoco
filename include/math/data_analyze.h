#ifndef DATA_ANALYZE_H
#define DATA_ANALYZE_H
#include "math/mathTypes.h"

class param
{
public:
    Mat3 _Kpp;
    Mat3 _Kdp;
    double _kpw;
    Mat3 _Kdw;
    Mat3 _KpSwing;
    Mat3 _KdSwing;
};

class data_analyze
{
public:
    data_analyze();
    void sendComPos(double sim_time, const Eigen::Vector3d& pcom, const Eigen::Vector3d& rpy);

    bool sendParamData(const param& param_data);
    bool getParamData(param& param_data);
    param _param;
private:
    double _lastSendTime;
};

#endif
