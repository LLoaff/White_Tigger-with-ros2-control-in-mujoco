#ifndef CONVEXMPC
#define CONVEXMPC

#include "math/mathtool.h"
#include "math/mathTypes.h"
#include "FSM/ControlComponent.h"
#include "Gait/GaitGenerator.h"
#include "MPC/GraphSearch.h"
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <assert.h>
#include "osqp.h"


struct BblockID {
  u32 foot;
  u32 timestep;
};

struct OsqpCSC {
  u32 nnz, m, n;
  c_float* values = nullptr;
  c_int* colPtrs = nullptr;
  c_int* rowIdx = nullptr;

  void alloc(u32 _n, u32 _nnz) {
    nnz = _nnz;
    n = _n;
    m = _n;
    values = new c_float[nnz];
    colPtrs = new c_int[n + 1];
    rowIdx = new c_int[nnz];
  }

  void freeAll() {
    delete[] values;
    delete[] colPtrs;
    delete[] rowIdx;
  }
};

struct SparseTriple {
  double value;
  u32 r, c;
};
class ConvexMPC
{
public:
    ConvexMPC(ControlComponent* comp);
    ~ConvexMPC();

    void MPCrun(Vec3& world_pos_des,Vec3 _vCmdGlobal,Vec3 _wCmdGlobal,float yaw_des);
    Vec12 getResult();

private:
    ControlComponent* _comp;
    void initSparseMPC();

    void solveSparseMPC(int *mpcTable);
    void updateMPCIfNeeded(int *mpcTable,Vec3 _vCmdGlobal,Vec3 _wCmdGlobal,float yaw_des);

    void setX0(Vec3 p, Vec3 v, Vec4 q, Vec3 w);
    void setContactTrajectory(ContactState* contacts, std::size_t length);
    void setStateTrajectory(vectorAligned<Vec12>& traj);
    void setFeet(Vec12& feet);

    void QP_run();

    void buildX0();
    void buildCT();
    void buildDT();
    void c2d(u32 trajIdx, u32 bBlockStartIdx, u32 block_count);

    void addX0Constraint();
    void addDynamicsConstraints();
    void addForceConstraints();
    void addFrictionConstraints();
    void addQuadraticStateCost();
    void addLinearStateCost();
    void addQuadraticControlCost();
    void runSolverOSQP();

    u32 getStateIndex(u32 trajIdx);
    u32 getControlIndex(u32 bBlockIdx);
    u32 addConstraint(u32 size);
    void addConstraintTriple(double value, u32 row, u32 col);


    void sortAndSumTriples(std::vector<SparseTriple>& triples);
    void sortTriples(std::vector<SparseTriple>& triples, bool checkDuplicates);
    void sumSortedTriples(std::vector<SparseTriple>& triples);
    bool checkSortedTripleDuplicates(std::vector<SparseTriple>& triples);
    OsqpCSC compress(std::vector<SparseTriple>& entries, u32 m, u32 n);
    
    Mat3 _Ibody;
    Vec12 _weights;
    double _mass, _maxForce, _mu, _alpha;
    Vec3 _p0, _v0, _w0, _rpy0;
    Vec4 _q0;
    Vec12 _x0;
    Vec12 _pFeet, _g;

    std::vector<ContactState> _contactTrajectory;
    vectorAligned<Vec12> _stateTrajectory;
    vectorAligned<Vec12> _sparseTrajectory;
    std::vector<double> _dtTrajectory;

  // intermediates
    vectorAligned<Mat12> _aMat;
    std::vector<BblockID> _bBlockIds;
    vectorAligned<Eigen::Matrix<double,12,3>> _bBlocks;
    std::vector<u32> _contactCounts;
    std::vector<u32> _runningContactCounts;

    std::vector<SparseTriple> _constraintTriples, _costTriples;
    std::vector<double> _lb, _ub, _linearCost;
 
    Eigen::Matrix<float, Eigen::Dynamic, 1> _result;

    //PID
    Mat3 _KpSwing, _KdSwing;
    Mat3 Kp, Kd, Kp_stance, Kd_stance;

    GaitGenerator*  _gait;

    Vec3 _rpy;
    Vec3 world_position_desired;

    int iterationsBetweenMPC;   // MPC运行周期
    int iterationCounter;       // 已经运行了的小周期
    int horizonLength;          // 预测段长度

    u32 _trajectoryLength;
    u32 _bBlockCount;
    u32 _constraintCount;

    float _dt;                 // 控制器运行周期
    float dtMPC;

    double stand_traj[6];       // 站立姿态
    double trajAll[12*36];

    bool firstRun;
};






#endif
