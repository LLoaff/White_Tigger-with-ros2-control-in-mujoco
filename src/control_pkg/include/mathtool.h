#ifndef MATHTOOL_H
#define MATHTOOL_H

#include "FSM/EnumClassList.h"
#include "eigen3/Eigen/Dense"
#include <iostream>

#define _labad_   0.08785 // l1
#define _lhip_    0.12    // l2
#define _lknee_   0.1358  // l3

#define _length_  0.1842  // hx
#define _weigh_   0.04    // hy

#define MPC_T   0.01
#define _mpc_steps    20

/*反归一化*/
template<typename T0, typename T1, typename T2>
inline T1 invNormalize(const T0 value, const T1 min, const T2 max, const float minLim = -1, const float maxLim = 1){
	return (value-minLim)*(max-min)/(maxLim-minLim) + min;
}


/*roll 转旋转矩阵*/
inline Eigen::Matrix<double, 3, 3> rotx(const float &theta) {
    float s = std::sin(theta);
    float c = std::cos(theta);

    Eigen::Matrix<double, 3, 3> R;
    R << 1, 0, 0, 0, c, -s, 0, s, c;
    return R;
}

/*yaw 转旋转矩阵*/
inline Eigen::Matrix<double, 3, 3> rotz(const float &theta) {
    float s = std::sin(theta);
    float c = std::cos(theta);

    Eigen::Matrix<double, 3, 3> R;
    R << c, -s, 0, s, c, 0, 0, 0, 1;
    return R;
}

/*pitch 转旋转矩阵*/
inline Eigen::Matrix<double, 3, 3> roty(const float &theta) {
    float s = std::sin(theta);
    float c = std::cos(theta);

    Eigen::Matrix<double, 3, 3> R;
    R << c, 0, s, 0, 1, 0, -s, 0, c;
    return R;
}
/* 欧拉角转3x3旋转矩阵 */
inline Eigen::Matrix<double, 3, 3> Rpy2RotMat(const float& row, const float& pitch, const float& yaw) {
    Eigen::Matrix<double, 3, 3> m = rotz(yaw) * roty(pitch) * rotx(row);
    return m;
}
/* 旋转矩阵转欧拉角 */
inline Eigen::Matrix<float, 3, 1> rotMatToRPY(const Eigen::Matrix<float, 3, 3>& R) {
    Eigen::Matrix<float, 3, 1> rpy;
    rpy(0) = atan2(R(2,1),R(2,2));
    rpy(1) = asin(-R(2,0));
    rpy(2) = atan2(R(1,0),R(0,0));
    return rpy;
}

/* 旋转矩阵 与 pos 构造4x4齐次变换矩阵 */
inline Eigen::Matrix<float, 4, 4> homoMatrix(Eigen::Matrix<float, 3, 1> p, Eigen::Matrix<float, 3, 3> m){
    Eigen::Matrix<float, 4, 4> homoM;
    homoM.setZero();
    homoM.topLeftCorner(3, 3) = m;
    homoM.topRightCorner(3, 1) = p;
    homoM(3, 3) = 1;
    return homoM;
}
/*齐次变换矩阵求逆*/
inline Eigen::Matrix<float, 4, 4> homoMatrixInverse(Eigen::Matrix<float, 4, 4> homoM){
    Eigen::Matrix<float, 4, 4> homoInv;
    homoInv.setZero();
    homoInv.topLeftCorner(3, 3) = homoM.topLeftCorner(3, 3).transpose();
    homoInv.topRightCorner(3, 1) = -homoM.topLeftCorner(3, 3).transpose() * homoM.topRightCorner(3, 1);
    homoInv(3, 3) = 1;
    return homoInv;
}

/* 把3x1的位置向量 转换成 4x1向量 */
inline Eigen::Matrix<float, 4, 1> homoVec(Eigen::Matrix<float, 3, 1> v3){
    Eigen::Matrix<float, 4, 1> v4;
    v4.block(0, 0, 3, 1) = v3;
    v4(3) = 1;
    return v4;
}

/*把4x1的位置向量 转换成 3x1向量*/
inline Eigen::Matrix<float, 3, 1> noHomoVec(Eigen::Matrix<float, 4, 1> v4){
    Eigen::Matrix<float, 3, 1> v3;
    v3 = v4.block(0, 0, 3, 1);
    return v3;
}

/* 四元数转旋转矩阵 */
inline Eigen::Matrix<float, 3, 3>Quat2RotMat(Eigen::Matrix<float, 4, 1> q){
    double e0 = q(0);
    double e1 = q(1);
    double e2 = q(2);
    double e3 = q(3);
    Eigen::Matrix<float, 3, 3> R;
    R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
            2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
            1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
            2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
            1 - 2 * (e1 * e1 + e2 * e2);
    return R;
}
//四元数转欧拉角
inline Eigen::Matrix<float,3,1> Quat2Euler(Eigen::Matrix<float, 4, 1> q){
    Eigen::Matrix<float,3,1> euler;
    float w = q[0];
    float x = q[1];
    float y = q[2];
    float z = q[3];

    float sinr_cosp = 2 * (w * x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    float roll = atan2(sinr_cosp, cosr_cosp);

    float sinp = 2 * (w * y - z * x);
    float pitch;
    pitch = asin(sinp);
    

    float siny_cosp = 2 * (w * z + x * y);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    float yaw = atan2(siny_cosp, cosy_cosp);

    euler << roll, pitch, yaw;
    return euler;
}
/*平衡所用旋转矩阵*/
inline Eigen::Matrix<double, 3, 3> BalanceRPY(Eigen::Matrix<float, 4, 1> q){
    Eigen::Matrix<double,3,1> rpy;
    Eigen::Matrix<double, 3, 3> rotmat;
    rpy=Quat2Euler(q).cast<double>();
    rotmat = roty(-rpy(1)) * rotx(-rpy(0));
    return rotmat;
}

template<typename T1, typename T2, typename T3>  
inline void UpdateCovariance(T1 &cov, T2 expPast, T3 newValue, double n){
    if( (cov.rows()!=cov.cols()) || (cov.rows() != expPast.rows()) || (expPast.rows()!=newValue.rows())){
        std::cout << "The size of updateCovariance is error" << std::endl;
        exit(-1);
    }
    if(fabs(n - 1) < 0.1){
        cov.setZero();
    }else{
        cov = cov*(n-1)/n + (newValue-expPast)*(newValue-expPast).transpose()*(n-1)/(n*n); // 协方差递归公式 
    }
}

template<typename T1, typename T2>
inline void UpdateAverage(T1 &exp, T2 newValue, double n){
    if(exp.rows()!=newValue.rows()){
        std::cout << "The size of updateAverage is error" << std::endl;
        exit(-1);
    }
    if(fabs(n - 1) < 0.001){
        exp = newValue;
    }else{
        exp = exp + (newValue - exp)/n; // 平均值
    }
}

template<typename T1, typename T2, typename T3>
inline void  UpdateAvgCov(T1 &cov, T2 &exp, T3 newValue, double n){
    UpdateCovariance(cov, exp, newValue, n);
    UpdateAverage(exp, newValue, n);

}
class AvgCov{
public:
    AvgCov(unsigned int size , std::string name,bool avfonly = false , unsigned int showperiod = 500,unsigned int waitcount=5000,double zoomfactor=1)
    : _size(size),_valuename(name),_avgonly(avfonly),_showperiod(showperiod),_waitcount(waitcount),_zoomfactor(zoomfactor){
    _exp.resize(size);
    _cov.resize(size,size);
    _defaultweight.resize(size, size);
    _defaultweight.setIdentity();
    _measurecount = 0;
    }
    // 传入的是Xn
    void Measure(Eigen::Matrix<double, Eigen::Dynamic, 1> newvalue){
        ++_measurecount;
        if(_measurecount > _waitcount){
            UpdateAvgCov(_cov, _exp, newvalue, _measurecount-_waitcount);
            if(_measurecount % _showperiod == 0){
                std::cout << "******" << _valuename << " measured count: " << _measurecount-_waitcount << "******" << std::endl;
                std::cout << _zoomfactor << " Times Average of " << _valuename << std::endl << (_zoomfactor*_exp).transpose() << std::endl;
                if(!_avgonly){
                    std::cout << _zoomfactor << " Times Covariance of " << _valuename << std::endl << _zoomfactor*_cov << std::endl;
                }
            }
        }
    }

private:
    Eigen::Matrix<double, Eigen::Dynamic, 1>              _exp;     // 动态向量
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _cov;     // 动态矩阵
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _defaultweight;  // 默认宽度
    bool _avgonly;                                                  // 决定是否输出协方差
    unsigned int _size;
    unsigned int _measurecount;
    unsigned int _showperiod;
    unsigned int _waitcount;
    double _zoomfactor;                                             // 数值缩放因子
    std::string _valuename;
};

// 求反对称矩阵 ， 用于叉乘
inline Eigen::Matrix<double, 3, 3> skew(const Eigen::Matrix<double, 3, 1>& v) {
    Eigen::Matrix<double, 3, 3> m;
        m << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
    return m;
}

// 0<=x<windowRatio-> 0线性升到1      windowRatio ≤ x ≤ 1-windowRatio-> 恒等于 1    1-windowRatio < x ≤ 1 ->从 1 线性下降到 0
template<typename T>
inline T windowFunc(const T x, const T windowRatio, const T xRange=1.0, const T yRange=1.0){
    if((x < 0)||(x > xRange)){
        std::cout << "[ERROR][windowFunc] The x=" << x << ", which should between [0, xRange]" << std::endl;
    }
    if((windowRatio <= 0)||(windowRatio >= 0.5)){
        std::cout << "[ERROR][windowFunc] The windowRatio=" << windowRatio << ", which should between [0, 0.5]" << std::endl;
    }

    if(x/xRange < windowRatio){
        return x * yRange / (xRange * windowRatio);
    }
    else if(x/xRange > 1 - windowRatio){
        return yRange * (xRange - x)/(xRange * windowRatio);
    }
    else{
        return yRange;
    }
}

inline Eigen::Matrix<double, 3, 4> vec12ToVec34(Eigen::Matrix<double, 12, 1> vec12){
    Eigen::Matrix<double, 3, 4> vec34;
    for(int i(0); i < 4; ++i){
        vec34.col(i) = vec12.segment(3*i, 3);
    }
    return vec34;
}

inline Eigen::Matrix<float, 12, 1> vec34ToVec12(Eigen::Matrix<float, 3, 4> vec34){
    Eigen::Matrix<float, 12, 1> vec12;
    for(int i(0); i < 4; ++i){
        vec12.segment(3*i, 3) = vec34.col(i);
    }
    return vec12;
}

// 旋转矩阵转指数坐标 ******
inline Eigen::Matrix<double, 3, 1> rotMatToExp(const Eigen::Matrix<double, 3, 3>& rm){
    double cosValue = rm.trace()/2.0-1/2.0;
    if(cosValue > 1.0f){
        cosValue = 1.0f;
    }else if(cosValue < -1.0f){
        cosValue = -1.0f;
    }

    double angle = acos(cosValue);
    Eigen::Matrix<double, 3, 1> exp;
    if (fabs(angle) < 1e-5){
        exp=Eigen::Matrix<double, 3, 1>(0,0,0);
    }
    else if (fabs(angle - M_PI) < 1e-5){
        exp = angle * Eigen::Matrix<double, 3, 1>(rm(0,0)+1, rm(0,1), rm(0,2)) / sqrt(2*(1+rm(0, 0)));
    }
    else{
        exp=angle/(2.0f*sin(angle))*Eigen::Matrix<double, 3, 1>(rm(2,1)-rm(1,2),rm(0,2)-rm(2,0),rm(1,0)-rm(0,1));
    }
    return exp;
}

template<typename T>
inline T saturation(const T a, Eigen::Matrix<double,2,1> limits){
    T lowLim, highLim;
    if(limits(0) > limits(1)){
        lowLim = limits(1);
        highLim= limits(0);
    }else{
        lowLim = limits(0);
        highLim= limits(1);
    }

    if(a < lowLim){
        return lowLim;
    }
    else if(a > highLim){
        return highLim;
    }
    else{
        return a;
    }
}


#endif