#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "low/LowState.h"
#include "eigen3/Eigen/Dense"
#include "math/mathtool.h"
#include "FSM/EnumClassList.h"
#include "math/Kenimatics_normal_solution.h"
#include "math/mathTypes.h"

using namespace std;

class Estimator
{
public:
    Estimator(LowState* lowstate,Eigen::Matrix<int, 4, 1>* contact,Eigen::Matrix<float, 4, 1> * phase,float dt);
    void init();
    Eigen::Matrix<float, 3, 1>  getPosition();
    Eigen::Matrix<float, 3, 1>  getVelocity();
    Eigen::Matrix<float, 3, 1>  getFootPos(int i);
    Eigen::Matrix<float, 3, 4> getFeetPos();
    Eigen::Matrix<float, 3, 4> getFeetVel();
    Eigen::Matrix<float, 3, 4> getPosFeet2BGlobal();
    void run();
    LowState* getLowState();

    Eigen::Matrix<float,3,4>   iPb;
private:

    Eigen::Matrix<float, 18, 1>  _xhat;            // The state of estimator, position(3)+velocity(3)+feet position(3x4)
    Vec3 _u;                                        // The input of estimator
    Eigen::Matrix<float, 28,  1> _y;               // The measurement value of output y
    Eigen::Matrix<float, 28,  1> _yhat;            // The prediction of output y
    Eigen::Matrix<float, 18, 18> _A;               // The transtion matrix of estimator
    Eigen::Matrix<float, 18, 3>  _B;               // The input matrix
    Eigen::Matrix<float, 28, 18> _C;               // The output matrix
    // Covariance Matrix
    Eigen::Matrix<float, 18, 18> _P;               // Prediction covariance
    Eigen::Matrix<float, 18, 18> _Ppriori;         // Priori prediction covariance
    Eigen::Matrix<float, 18, 18> _Q;               // Dynamic simulation covariance
    Eigen::Matrix<float, 28, 28> _R;               // Measurement covariance
    Eigen::Matrix<float, 18, 18> _QInit;           // Initial value of Dynamic simulation covariance
    Eigen::Matrix<float, 28, 28> _RInit;           // Initial value of Measurement covariance
    Vec18 _Qdig;                                    // adjustable process noise covariance
    Mat3 _Cu;                                       // The covariance of system input u
    // Output Measurement
    Eigen::Matrix<float, 12, 1>  _feetPos2Body;    // The feet positions to body, in the global coordinate
    Eigen::Matrix<float, 12, 1>  _feetVel2Body;    // The feet velocity to body, in the global coordinate
    Eigen::Matrix<float,  4, 1>  _feetH;           // The Height of each foot, in the global coordinate
    Eigen::Matrix<float, 28, 28> _S;               // _S = C*P*C.T + R
    Eigen::PartialPivLU<Eigen::Matrix<float, 28, 28>> _Slu;    // _S.lu()
    Eigen::Matrix<float, 28,  1> _Sy;              // _Sy = _S.inv() * (y - yhat)
    Eigen::Matrix<float, 28, 18> _Sc;              // _Sc = _S.inv() * C
    Eigen::Matrix<float, 28, 28> _SR;              // _SR = _S.inv() * R
    Eigen::Matrix<float, 28, 18> _STC;             // _STC = (_S.transpose()).inv() * C
    Eigen::Matrix<float, 18, 18> _IKC;             // _IKC = I - KC

    RotMat _rotMatB2G;                              // Rotate Matrix: from body to global
    Vec3 _g;
    Vec34 _feetPosGlobalKine, _feetVelGlobalKine;

    LowState* _lowState;
    Vec4 *_phase;
    VecInt4 *_contact;
    float _dt;
    float _trust;
    float _largeVariance;

    // Low pass filters
    // LPFilter *_vxFilter, *_vyFilter, *_vzFilter;

    AvgCov *_RCheck;
    AvgCov *_uCheck;
    std::string _estName;
};

#endif