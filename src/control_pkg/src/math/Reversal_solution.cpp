#include "Reversal_solution.h"

inline float clamp(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

float q1_ik(float py, float pz, float l1){
    float q1;
    float L = sqrt(pow(py,2)+pow(pz,2)-pow(l1,2));
    q1 = atan2(pz*l1+py*L, py*l1-pz*L);
    return q1;
}

float q3_ik(float b3z, float b4z, float b){
    float q3, temp;
    temp = (pow(b3z, 2) + pow(b4z, 2) - pow(b, 2))/(2*fabs(b3z*b4z));
    if(temp>1) temp = 1;
    if(temp<-1) temp = -1;
    q3 = acos(temp);
    q3 = -(M_PI - q3); //0~180
    return q3;
}

float q2_ik(float q1, float q3, float px, float py, float pz, float b3z, float b4z){
    float q2, a1, a2, m1, m2;
    
    a1 = py*sin(q1) - pz*cos(q1);
    a2 = px;
    m1 = b4z*sin(q3);
    m2 = b3z + b4z*cos(q3);
    q2 = atan2(m1*a1+m2*a2, m1*a2-m2*a1);
    return q2;
}

Eigen::Matrix<float,3,1> Reversal_Solution_Update(uint8_t group , float x , float y , float z){
    Eigen::Matrix<float,3,1> pEe2H;
    pEe2H(0) = x;
    pEe2H(1) = y;
    pEe2H(2) = z;

    float q1, q2, q3;
    Eigen::Matrix<float,3,1> qResult;
    float px, py, pz;
    float b2y, b3z, b4z, a, b, c;

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

Eigen::Matrix<float,3,1> Reversal_Update_B(uint8_t group , float x , float y , float z){
    Eigen::Matrix<float,3,1> pEe2B;
    float length = _length_;
    float weigh = _weigh_;

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
    // std::cout<< " group: \n" << (int)group<< std::endl;
    // std::cout<< " weigh: \n" << weigh<< std::endl;
    // std::cout<< " length: \n" << length<< std::endl;

    // std::cout<< " pEe2B: \n" << pEe2B<< std::endl;
    float q1, q2, q3;
    Eigen::Matrix<float,3,1> qResult;
    float px, py, pz;
    float b2y, b3z, b4z, a, b, c;

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
    Eigen::Matrix<float,3,4> vecp = vecP.cast<float>();
    if(frame == FrameType::BODY){
        for(int i(0); i < 4; ++i){
            q.segment(3*i, 3) = Reversal_Update_B(i,vecp(0,i),vecp(1,i),vecp(2,i)).cast<double>();
        }
    }
    else if(frame == FrameType::HIP){
        for(int i(0); i < 4; ++i){
            q.segment(3*i, 3) = Reversal_Solution_Update(i,vecp(0,i),vecp(1,i),vecp(2,i)).cast<double>();
        }
    }
    
    return q;
}

// Eigen::Matrix<double,3,3> calcJaco(int legid , Eigen::Matrix<float,3,1> q){
//     Eigen::Matrix<double,3,3> Jac;
//     float l1 = _labad_;
//     float l2 = -_lhip_;
//     float l3 = -_lknee_;
    
//     if(legid == 0 || legid == 2)
//         l1 = -_labad_;
//     // 计算雅可比 

//     float s1 = std::sin(q(0));
//     float s2 = std::sin(q(1));
//     float s3 = std::sin(q(2));

//     float c1 = std::cos(q(0));
//     float c2 = std::cos(q(1));
//     float c3 = std::cos(q(2));

//     float c23 = c2 * c3 - s2 * s3;
//     float s23 = s2 * c3 + c2 * s3;
//     Jac(0, 0) = 0;
//     Jac(1, 0) = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1;
//     Jac(2, 0) = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1;
//     Jac(0, 1) = l3 * c23 + l2 * c2;
//     Jac(1, 1) = l3 * s1 * s23 + l2 * s1 * s2;
//     Jac(2, 1) = -l3 * c1 * s23 - l2 * c1 * s2;
//     Jac(0, 2) = l3 * c23;
//     Jac(1, 2) = l3 * s1 * s23;
//     Jac(2, 2) = -l3 * c1 * s23;
//     return Jac;
// }

Vec3 calcQd(int legid,Vec3 pEe, Vec3 vEe, FrameType frame){
    Vec3 q;
    if(frame == FrameType::BODY){
         q = Reversal_Update_B(legid,(float)pEe(0),(float)pEe(1),(float)pEe(2)).cast<double>();
    }
    return calcJaco(legid,q.cast<float>()).inverse() * vEe;
} 

Vec12 Reversal_GetQd(const Vec34 &pos, const Vec34 &vel, FrameType frame){
    Vec12 qd;
    for(int i(0); i < 4; ++i){
        qd.segment(3*i, 3) = calcQd(i,pos.col(i), vel.col(i), frame);
    }
    return qd;
}
