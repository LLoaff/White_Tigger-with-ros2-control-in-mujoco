#include "math/Reversal_solution.h"
#include <algorithm>
#include <cmath>
inline double clamp(double val, double min_val, double max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}
bool limitFootTargetBody(int legId, Vec3& pBody){
    const bool isFront = (legId == 0 || legId == 1);
    const bool isRight = (legId == 0 || legId == 2);

    /*
     * 机身中心到髋关节原点的偏移。
     *
     * 前腿 x 为正，后腿 x 为负；
     * 左腿 y 为正，右腿 y 为负。
     */
    Vec3 hipOffset;

    hipOffset(0) = isFront ? _length_ : -_length_;
    hipOffset(1) = isRight ? -_weigh_ : _weigh_;
    hipOffset(2) = 0.0;

    /*
     * 把机身坐标系中的足端目标转换到髋关节坐标系。
     */
    Vec3 pHip = pBody - hipOffset;

    /*
     * 如果输入已经出现 NaN 或无穷大，
     * 直接换成该腿的安全站立位置。
     */
    bool limited = false;

    const double side = isRight ? -1.0 : 1.0;

    if (!pHip.allFinite()) {
        pHip(0) = 0.0;
        pHip(1) = side * _labad_;
        pHip(2) = -0.19;

        limited = true;
    }

    const Vec3 original = pHip;

    /*
     * 第一层：限制足端相对髋关节的前后距离。
     *
     * 这可以防止落脚点因为速度、转向或估计误差，
     * 跑到身体前方或后方太远。
     */
    constexpr double xMax = 0.08;

    pHip(0) = clamp(pHip(0), -xMax, xMax);

    /*
     * 第二层：保证 q1_ik() 中的根号合法。
     *
     * q1_ik() 中存在：
     *
     * sqrt(py² + pz² - labad²)
     *
     * 因此必须保证：
     *
     * py² + pz² >= labad²
     */
    constexpr double radialMargin = 0.005;

    const double radialMinimum =
        _labad_ + radialMargin;

    double radialYZ =
        std::sqrt(
            pHip(1) * pHip(1) +
            pHip(2) * pHip(2)
        );

    if (radialYZ < radialMinimum) {
        if (radialYZ > 1e-9) {
            const double scale =
                radialMinimum / radialYZ;

            pHip(1) *= scale;
            pHip(2) *= scale;
        }
        else {
            pHip(1) = side * _labad_;
            pHip(2) = -0.10;
        }

        limited = true;
    }

    /*
     * 根据你原来 q1_ik() 的公式计算侧摆关节角。
     */
    const double signedL1 =
        isRight ? -_labad_ : _labad_;

    const double yzInside =
        pHip(1) * pHip(1) +
        pHip(2) * pHip(2) -
        _labad_ * _labad_;

    const double L =
        std::sqrt(std::max(0.0, yzInside));

    double q1 = std::atan2(
        pHip(2) * signedL1 + pHip(1) * L,
        pHip(1) * signedL1 - pHip(2) * L
    );

    /*
     * 模型中的 q1 关节范围是 ±0.8 rad。
     * 这里先使用更保守的 ±0.45 rad，
     * 防止足端跑到机身内部或横向伸得太远。
     */
    constexpr double q1SafeLimit = 0.45;

    const double limitedQ1 =
        clamp(q1, -q1SafeLimit, q1SafeLimit);

    if (std::fabs(limitedQ1 - q1) > 1e-9) {
        limited = true;
    }

    q1 = limitedQ1;

    /*
     * a1 是去掉侧摆连杆之后，足端在腿平面内
     * 向下方向的距离。
     *
     * 这和你 q2_ik() 里的 a1 定义完全一致。
     */
    double a1 =
        pHip(1) * std::sin(q1) -
        pHip(2) * std::cos(q1);

    /*
     * 保证脚始终位于髋关节下方一定距离。
     *
     * 正常站立时大约是 0.19m；
     * 摆腿最高点大约是 0.15m；
     * 所以 0.10m 是比较保守的最低值。
     */
    constexpr double downwardMinimum = 0.10;

    if (a1 < downwardMinimum) {
        a1 = downwardMinimum;
        limited = true;
    }

    /*
     * b 是大腿和小腿构成的平面二连杆距离：
     *
     * b = sqrt(x² + a1²)
     */
    double b =
        std::sqrt(
            pHip(0) * pHip(0) +
            a1 * a1
        );

    constexpr double foldMargin = 0.015;
    constexpr double extensionMargin = 0.015;

    const double bMinimum =
        std::fabs(_lhip_ - _lknee_) +
        foldMargin;

    const double bMaximum =
        _lhip_ + _lknee_ -
        extensionMargin;

    const double limitedB =
        clamp(b, bMinimum, bMaximum);

    /*
     * 如果二连杆距离超出安全范围，
     * 等比例缩放 x 和 a1。
     *
     * 这样不会改变足端在腿平面内的方向，
     * 只会把目标拉回安全圆环内。
     */
    if (std::fabs(limitedB - b) > 1e-9) {
        if (b > 1e-9) {
            const double scale =
                limitedB / b;

            pHip(0) *= scale;
            a1 *= scale;
        }
        else {
            pHip(0) = 0.0;
            a1 = bMinimum;
        }

        limited = true;
    }

    /*
     * 使用限制后的 q1 和 a1，
     * 重新构造髋关节坐标系下的 y、z。
     */
    pHip(1) =
        signedL1 * std::cos(q1) +
        a1 * std::sin(q1);

    pHip(2) =
        signedL1 * std::sin(q1) -
        a1 * std::cos(q1);

    if ((pHip - original).norm() > 1e-9) {
        limited = true;
    }

    /*
     * 从髋关节坐标系变回机身坐标系。
     */
    pBody = hipOffset + pHip;

    return limited;
}
double q1_ik(double py, double pz, double l1){
    double q1;
    double L = sqrt(pow(py,2)+pow(pz,2)-pow(l1,2));
    q1 = atan2(pz*l1+py*L, py*l1-pz*L);
    return q1;
}

double q3_ik(double b3z, double b4z, double b){
    double q3, temp;
    temp = (pow(b3z, 2) + pow(b4z, 2) - pow(b, 2))/(2*fabs(b3z*b4z));
    if(temp>1) temp = 1;
    if(temp<-1) temp = -1;
    q3 = acos(temp);
    q3 = -(M_PI - q3); //0~180
    return q3;
}

double q2_ik(double q1, double q3, double px, double py, double pz, double b3z, double b4z){
    double q2, a1, a2, m1, m2;
    
    a1 = py*sin(q1) - pz*cos(q1);
    a2 = px;
    m1 = b4z*sin(q3);
    m2 = b3z + b4z*cos(q3);
    q2 = atan2(m1*a1+m2*a2, m1*a2-m2*a1);
    return q2;
}

Eigen::Matrix<double,3,1> Reversal_Solution_Update(uint8_t group , double x , double y , double z){
    Eigen::Matrix<double,3,1> pEe2H;
    pEe2H(0) = x;
    pEe2H(1) = y;
    pEe2H(2) = z;

    double q1, q2, q3;
    Eigen::Matrix<double,3,1> qResult;
    double px, py, pz;
    double b2y, b3z, b4z, a, b, c;

    px = pEe2H(0);
    py = pEe2H(1);
    pz = pEe2H(2);

    b2y = _labad_ ;
    b3z = -_lhip_;
    b4z = -_lknee_;
    a = _labad_;
    if(group == 0 || group == 2)
        b2y = -_labad_;
    c = sqrt(pow(px, 2) + pow(py, 2) + pow(pz, 2)); // whole length
    b = sqrt(pow(c, 2) - pow(a, 2)); // distance between shoulder and footpoint

    q1 = q1_ik(py, pz, b2y);
    q3 = q3_ik(b3z, b4z, b);
    q2 = q2_ik(q1, q3, px, py, pz, b3z, b4z);

    qResult(0) = q1;
    qResult(1) = q2;
    qResult(2) = q3;
    return qResult;
}

Eigen::Matrix<double,3,1> Reversal_Update_B(uint8_t group , double x , double y , double z){
    Eigen::Matrix<double,3,1> pEe2B;
    double length = _length_;
    double weigh = _weigh_;

    if(group == 0){
        weigh = -weigh;
    }
    else if(group == 3){
        length = -length;
    }
    else if(group == 2){
        weigh = -weigh;
        length = -length;
    }
    
    pEe2B(0) = x - length;
    pEe2B(1) = y - weigh;
    pEe2B(2) = z;

    double q1, q2, q3;
    Eigen::Matrix<double,3,1> qResult;
    double px, py, pz;
    double b2y, b3z, b4z, a, b, c;

    px = pEe2B(0);
    py = pEe2B(1);
    pz = pEe2B(2);

    b2y = _labad_ ;
    b3z = -_lhip_;
    b4z = -_lknee_;
    a = _labad_;
    if(group == 0 || group == 2)
        b2y = -_labad_;
    c = sqrt(pow(px, 2) + pow(py, 2) + pow(pz, 2)); // whole length
    b = sqrt(pow(c, 2) - pow(a, 2)); // distance between shoulder and footpoint

    q1 = q1_ik(py, pz, b2y);
    q3 = q3_ik(b3z, b4z, b);
    q2 = q2_ik(q1, q3, px, py, pz, b3z, b4z);

    qResult(0) = q1;
    qResult(1) = q2;
    qResult(2) = q3;
    return qResult;
}

Vec12 Reversal_GetQ(const Vec34 &vecP, FrameType frame){
    Vec12 q;
    Eigen::Matrix<double,3,4> vecp = vecP;
    if(frame == FrameType::BODY){
        for(int i(0); i < 4; ++i){
            q.segment(3*i, 3) = Reversal_Update_B(i,vecp(0,i),vecp(1,i),vecp(2,i));
        }
    }
    else if(frame == FrameType::HIP){
        for(int i(0); i < 4; ++i){
            q.segment(3*i, 3) = Reversal_Solution_Update(i,vecp(0,i),vecp(1,i),vecp(2,i));
        }
    }
    
    return q;
}


Vec3 calcQd(int legid,Vec3 pEe, Vec3 vEe, FrameType frame){
    Vec3 q;
    if(frame == FrameType::BODY){
         q = Reversal_Update_B(legid,pEe(0),pEe(1),pEe(2));
    }
    const Mat3 J = calcJaco(legid, q);
    constexpr double damping = 0.02;
    const Mat3 regularized =
        J * J.transpose()
        + damping * damping * Mat3::Identity();
    Vec3 qd =
        J.transpose()
        * regularized.ldlt().solve(vEe);
    constexpr double qdLimit = 20.0;
    for(int i = 0; i < 3; ++i){
        if(!std::isfinite(qd(i))){
            qd(i) = 0.0;
        }
        else{
            qd(i) = clamp(
                qd(i),
                -qdLimit,
                qdLimit
            );
        }
    }
    // return calcJaco(legid,q).inverse() * vEe;
    return qd;
} 

Vec12 Reversal_GetQd(const Vec34 &pos, const Vec34 &vel, FrameType frame){
    Vec12 qd;
    for(int i(0); i < 4; ++i){
        qd.segment(3*i, 3) = calcQd(i,pos.col(i), vel.col(i), frame);
    }
    return qd;
}
