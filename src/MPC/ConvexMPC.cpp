#include "MPC/ConvexMPC.h"

ConvexMPC::ConvexMPC(ControlComponent* comp):_comp(comp),_dt(comp->dt),iterationsBetweenMPC(25)
,horizonLength(10){

    // Kp << 700, 0, 0,
    //     0, 700, 0,
    //     0, 0, 150;
    // Kp_stance = 0*Kp;

    // Kd << 7, 0, 0,
    //     0, 7, 0,
    //     0, 0, 7;
    // Kd_stance = Kd;

    iterationCounter = 0;
    world_position_desired.setZero();

    _comp->waveGen->set_nIterations(horizonLength);
    _comp->waveGen->set_leg_periods(Eigen::Array<int,4,1>(10,10,10,10));
    dtMPC = _dt*iterationsBetweenMPC;

    initSparseMPC();
}

void ConvexMPC::initSparseMPC(){
    if(use_go1_model == 1){
        _Ibody<<0.0792,0.0,0.0,
                0.0,0.2085,0.0,
                0.0,0.0,0.2265;
        _mass = 12;
    }
    else{
        _Ibody<<0.03316,0.0,0.0,
                0.0,0.14005,0.0,
                0.0,0.0,0.16444;
        _mass = 6.408;
    }
    
    
    _maxForce = 120;
    _dtTrajectory.clear();

    for(int i = 0; i < horizonLength; i++) {
        _dtTrajectory.push_back(dtMPC);
    }
    _mu = 0.4;
    _weights << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2;
    _alpha = 4e-5;
    _sparseTrajectory.resize(horizonLength);

}

void ConvexMPC::MPCrun(Vec3& world_pos_des,Vec3 _vCmdGlobal,Vec3 _wCmdGlobal,float yaw_des){
    _rpy(0) = _comp->_ioros->_state->_imu.getRoll();
    _rpy(1) = _comp->_ioros->_state->_imu.getPitch();
    _rpy(2) = _comp->_ioros->_state->_imu.getYaw();

    world_position_desired = world_pos_des;
    _comp->waveGen->setIterations(iterationsBetweenMPC, iterationCounter);

    Vec4 contactStates = _comp->waveGen->getContactState();
    Vec4 swingStates = _comp->waveGen->getSwingState();

    int* mpcTable = _comp->waveGen->getMpcTable(_comp->getWaveStatus());

    if((iterationCounter%iterationsBetweenMPC)==0){
        updateMPCIfNeeded(mpcTable,_vCmdGlobal,_wCmdGlobal,yaw_des); // mpc求解
    }
    iterationCounter++;

}

void ConvexMPC::updateMPCIfNeeded(int *mpcTable,Vec3 _vCmdGlobal,Vec3 _wCmdGlobal,float yaw_des){
   Vec3 p = _comp->_estimator->getPosition();

    const float max_pos_error = .1;

    float xStart = world_position_desired[0];
    float yStart = world_position_desired[1];
    float _body_height;
    if(use_go1_model == 1){
        _body_height = 0.32;
    }
    else{
        _body_height = 0.2;
    }
    
    float trajInitial[12] ={
        0, //roll
        0, //pitch
        yaw_des, //yaw
        xStart,
        yStart,
        _body_height,
        0,
        0,
        _wCmdGlobal(2),
        _vCmdGlobal(0),
        _vCmdGlobal(1),
        0
    };
    for(int i=0;i<horizonLength;i++){
        for(int j =0 ;j<12;j++){
            trajAll[12*i +j] = trajInitial[j];
        }
        if(i==0){
            trajAll[2] = _rpy(2);
        } 
        else{
            trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _wCmdGlobal(2);
            trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * _vCmdGlobal[0];
            trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * _vCmdGlobal[1];
        }
    }
    solveSparseMPC(mpcTable);
}

void ConvexMPC::solveSparseMPC(int *mpcTable){
    (void) mpcTable;
    std::vector<ContactState> contactStates;

    for(int i=0;i<horizonLength;i++){
        contactStates.emplace_back(mpcTable[4*i+0],mpcTable[4*i+1],mpcTable[4*i+2],mpcTable[4*i+3]);
        for(int j=0;j<12;j++){
            _sparseTrajectory[i][j] = trajAll[i*12 + j];
        }
    }
    Vec12 feet =vec34ToVec12( _comp->_estimator->getPosFeet2BGlobal() );

    /* 设置初始状态 */
    setX0(_comp->_estimator->getPosition(),_comp->_estimator->getVelocity(),_comp->_ioros->_state->_imu.GetQuat(), _comp->_ioros->_state->_imu.getGyroGlobal());
    /* 设置接触状态 */
    setContactTrajectory(contactStates.data(),contactStates.size());
    /* 设置轨迹traj */
    setStateTrajectory(_sparseTrajectory);
    /* 设置足端相对机身的位置向量 */
    setFeet(feet);
    QP_run();   
}

void ConvexMPC::setX0(Vec3 p, Vec3 v, Vec4 q, Vec3 w){
    _p0 = p;
    _v0 = v;
    _q0 = q;
    _w0 = w;
}
void ConvexMPC::setContactTrajectory(ContactState* contacts, std::size_t length){
    _contactTrajectory.resize(length);
    for(std::size_t i = 0; i < length; i++) {
        _contactTrajectory[i] = contacts[i];
    }
}
void ConvexMPC::setStateTrajectory(vectorAligned<Vec12>& traj){
    _stateTrajectory = traj;
}
void ConvexMPC::setFeet(Vec12& feet){
    _pFeet = feet;
}
void ConvexMPC::QP_run(){
    if((_stateTrajectory.size() != _contactTrajectory.size()) || (_contactTrajectory.size() != _dtTrajectory.size())) {
        throw std::runtime_error("SparseCMPC trajectory length error!");
    }
    _g.setZero();
    _g[11] = -9.81;
    _constraintCount = 0;
    _trajectoryLength = _stateTrajectory.size();
    _constraintTriples.clear();     // 约束三元组
    _costTriples.clear();           // cost三元组
    _ub.clear();
    _lb.clear();
    _contactCounts.clear();       /*表示在第[i]段有多少个接触点*/
    _runningContactCounts.clear();/*表示在[i]之前 有多少个接触点*/
    _bBlockCount = 0;

    buildX0();
    buildCT();
    buildDT();

    addX0Constraint();
    addDynamicsConstraints();
    addForceConstraints();
    addFrictionConstraints();
    addQuadraticStateCost();
    addLinearStateCost();
    addQuadraticControlCost();

    // Solve!
    //runSolver();
    runSolverOSQP();
}

void ConvexMPC::buildX0(){
    _rpy0 = _rpy;
    _x0 << _rpy0,_p0, _w0, _v0;
}

void ConvexMPC::buildCT(){
    _aMat.clear();
    _aMat.resize(_trajectoryLength);
    _bBlockIds.clear();
    _bBlocks.clear();

    for(uint32_t i=0;i<horizonLength;i++){
        Mat3 Ryaw = rotz(_rpy0[2]);

        //  转动惯量计算
        Mat3 Iworld = Ryaw.transpose() * _Ibody * Ryaw;
        Mat3 Iinv = Iworld.inverse();

        Mat12 &A = _aMat[i];
        A.setZero();
        A(3,9) = 1; // x position integration
        A(4,10) = 1; // y position integration
        A(5,11) = 1; // z position integration
        A.block(0,6,3,3) = Ryaw; // omega integration

        auto & contactState = _contactTrajectory[i];
        for(uint32_t foot =0;foot<4;foot++){
            if(contactState.contact[foot]){// 判断这个腿是否触地
                _bBlocks.emplace_back();    /*在_bBlocks末尾新增一个12x3的矩阵*/
                _bBlockIds.push_back({foot,i});

                auto &B = _bBlocks.back();/*把_bBlocks末尾的12x3矩阵，给B*/

                Vec3 pFoot = _pFeet.block(foot*3,0,3,1);/*获取这个站立腿在世界*/
                                                        /*坐标系下足端相对机身的位置向量*/
                B.setZero();
                B.block(6,0,3,3) = Iinv * skew(pFoot);        // r x f torque
                B.block(9,0,3,3) = Mat3::Identity() / _mass;  // f = ma

            }
        }
    }
    _bBlockCount = _bBlockIds.size();/*获取整个horizon的所有站立腿数量*/

}
void ConvexMPC::buildDT(){
    u32 runningContactCount = 0;
    for(u32 i = 0; i < _trajectoryLength; i++){
        u32 contactCount = 0;
        for(auto contact : _contactTrajectory[i].contact){
            if(contact) contactCount++; /*获取每一段horizon的接触点个数*/
        }
        _contactCounts.push_back(contactCount);/*需要知道每一段horizon有几个接触点*/
        _runningContactCounts.push_back(runningContactCount);

        /*把连续时间动力学 离散化*/
        c2d(i, runningContactCount, contactCount);
        runningContactCount += contactCount;/*已经累积了多少腿的B block，用于c2d的索引定位*/

    }
}

void ConvexMPC::c2d(u32 trajIdx, u32 bBlockStartIdx, u32 block_count){
    Eigen::Matrix<double, 24, 24> AB, expmm;
    AB.setZero();
    AB.block(0,0,12,12) = _aMat[trajIdx];

    for(u32 i = bBlockStartIdx;i<bBlockStartIdx+block_count;i++){
        BblockID id = _bBlockIds[i];
        if(id.timestep != trajIdx) throw std::runtime_error("c2d timestep error");
        AB.block(0,12 + 3 * id.foot, 12, 3) = _bBlocks[i];
    }
    AB *= _dtTrajectory[trajIdx];
    expmm = AB.exp();
    _aMat[trajIdx] = expmm.block(0,0,12,12);

    for(u32 i =bBlockStartIdx;i < bBlockStartIdx + block_count; i++){
        //BblockID id = _bBlockIds[i];
        //_bBlocks[i] = expmm.block(0,12 + 3 * id.foot, 12, 3);
        _bBlocks[i] *= _dtTrajectory[trajIdx];
    }
}
void ConvexMPC::addX0Constraint(){
    // x[0] = A[0] * X0 + B[0] * u[0] + g*dt;
    // x[0] - (B[0] * u[0]) = A[0]*X0 + g*dt;

    /*Sparse MPC 最后会把所有变量拼成一个长向量*/
    u32 state_idx = getStateIndex(0);
    /*这里取到的index也是0,12传进去只是为了累加_constraintCount*/
    u32 constraint_idx = addConstraint(12);
    /*这里是给z 里的前 12 个变量*/
    for(u32 i = 0; i < 12; i++) {
        /*会形成一个12x12的单位矩阵*/
        _constraintTriples.push_back({1,constraint_idx + i, state_idx + i});
    }
    if(_runningContactCounts[0]) throw std::runtime_error("contact count error!");

    u32 ctrl_cnt = _contactCounts[0];

    for(u32 i = 0; i < ctrl_cnt; i++) { // Bblocks within this b (contact feet)
        // select -B[0]*u[0] 
        /*第 0 个预测段里，第 i 条接触腿对应的 B block 在 _bBlocks 数组里的编号*/
        u32 bbIdx = _runningContactCounts[0] + i;
        for(u32 ax = 0; ax < 3; ax++) { // columns within the b block (forces axes)
            for(u32 row = 0; row < 12; row++) { // rows within the b block
                addConstraintTriple(-_bBlocks[bbIdx](row, ax), constraint_idx + row, getControlIndex(bbIdx) + ax);
            }
        }
    }

    // compute right hand side A[0]*X0 + g*dt.
    Vec12 rhs = _aMat[0] * _x0 + _g * _dtTrajectory[0];

    // add to problem.
    for(u32 i = 0; i < 12; i++) {
        _ub.push_back(rhs[i]);
        _lb.push_back(rhs[i]);
    }
}
void ConvexMPC::addDynamicsConstraints(){
      // x[n] = A[n] * x[n-1] + B[n] * u[n] + g * dt[n]
    for(u32 i = 1; i < _trajectoryLength; i++) {
        u32 next_state_idx = getStateIndex(i);
        u32 prev_state_idx = getStateIndex(i - 1);
        u32 constraint_idx = addConstraint(12);

        // get I * x[n]
        for(u32 j = 0; j < 12; j++) {
            _constraintTriples.push_back({1, constraint_idx + j, next_state_idx + j});
        }

        // get -A[n] * x[n-1]
        for(u32 r = 0; r < 12; r++) {
            for(u32 c = 0; c < 12; c++) {
                addConstraintTriple(-_aMat[i](r,c), constraint_idx + r, prev_state_idx + c);
            }
        }

        // get -B[n] * u[n]
        u32 contact_count = _contactCounts[i];
        u32 bb_idx = _runningContactCounts[i];
        for(u32 contact = 0; contact < contact_count; contact++) {
            for(u32 row = 0; row < 12; row++) {
                for(u32 col = 0; col < 3; col++) {
                addConstraintTriple(-_bBlocks[bb_idx + contact](row, col),
                    constraint_idx + row,
                    getControlIndex(bb_idx + contact) + col);
                }
            }
        }

        // rhs g*dt
        Vec12 rhs = _g * _dtTrajectory[i];
        for(u32 j = 0; j < 12; j++) {
        _ub.push_back(rhs[j]);
        _lb.push_back(rhs[j]);
        }

    }
}
void ConvexMPC::addForceConstraints(){
    // constrain all Z forces:
    for(u32 i = 0; i < _bBlockCount; i++) {
            _ub.push_back(_maxForce);
            _lb.push_back(0);
            addConstraintTriple(1, // directly get force
            addConstraint(1),    // new constraint
            getControlIndex(i) + 2 // ith contact's z force
        );
    }
}
void ConvexMPC::addFrictionConstraints(){
    double muInv = 1. / _mu;
    for(u32 i = 0; i < _bBlockCount; i++) {
        // four friction constraints
        u32 constraint_idx = addConstraint(4);
        u32 control_idx    = getControlIndex(i);
        for(u32 c = 0; c < 4; c++) {
            _lb.push_back(0);
            _ub.push_back(1e15); // inf.
        }

        // x/mu + z > 0
        addConstraintTriple(muInv, constraint_idx + 0, control_idx + 0);
        addConstraintTriple(1, constraint_idx + 0, control_idx + 2);

        //-x/mu + z > 0
        addConstraintTriple(-muInv, constraint_idx + 1, control_idx + 0);
        addConstraintTriple(1, constraint_idx + 1, control_idx + 2);

        // y/mu + z > 0
        addConstraintTriple(muInv, constraint_idx + 2, control_idx + 1);
        addConstraintTriple(1, constraint_idx + 2, control_idx + 2);

        //-y/mu + z > 0
        addConstraintTriple(-muInv, constraint_idx + 3, control_idx + 1);
        addConstraintTriple(1, constraint_idx + 3, control_idx + 2);
    }
}
void ConvexMPC::addQuadraticStateCost(){
    // xt Q x
    for(u32 i = 0; i < _trajectoryLength; i++) {
        u32 idx = getStateIndex(i);
        for(u32 j = 0; j < 12; j++) {
            _costTriples.push_back({_weights[j], idx + j, idx + j});
        }
    }
}
void ConvexMPC::addQuadraticControlCost(){
    for(u32 i = 0; i < _bBlockCount; i++) {
        u32 idx = getControlIndex(i);
        for(u32 j = 0; j < 3; j++) {
            _costTriples.push_back({_alpha, idx + j, idx + j});
        }
  }
}
void ConvexMPC::addLinearStateCost(){
    _linearCost.resize(12 * _trajectoryLength + 3 * _bBlockCount);
    for(auto& v : _linearCost) v = 0;
    // -2 * w * x_des
    for(u32 i = 0; i < _trajectoryLength; i++) {
        u32 idx = getStateIndex(i);
        for(u32 j = 0; j < 12; j++) {
            _linearCost.at(idx + j) = -1. * _stateTrajectory[i][j] * _weights[j];
        }
    }
}

void ConvexMPC::runSolverOSQP(){
    u32 varCount = 12 * _trajectoryLength + 3 * _bBlockCount;
    //printf("[SparseCMPC] Run with OSQP %d, %d\n", varCount, _constraintCount);
    assert(_constraintCount == _ub.size());
    assert(_constraintCount == _lb.size());
    assert(varCount == _linearCost.size());

    // Quadratic term
    sortAndSumTriples(_costTriples);
    OsqpCSC quadraticCostMatrix = compress(_costTriples, varCount, varCount);

    sortAndSumTriples(_constraintTriples);
    OsqpCSC constraintMatrix = compress(_constraintTriples, _constraintCount, varCount);

    OSQPSettings* settings = (OSQPSettings*)malloc(sizeof(OSQPSettings));
    OSQPWorkspace* workspace;
    (void)workspace;
    OSQPData* data = (OSQPData*)malloc(sizeof(OSQPData));

    data->n = varCount;
    data->m = _constraintCount;
    data->P = csc_matrix(varCount, varCount, quadraticCostMatrix.nnz,
        quadraticCostMatrix.values, quadraticCostMatrix.rowIdx, quadraticCostMatrix.colPtrs);
    data->q = _linearCost.data();
    data->A = csc_matrix(_constraintCount, varCount, constraintMatrix.nnz,
        constraintMatrix.values, constraintMatrix.rowIdx, constraintMatrix.colPtrs);
    data->l = _lb.data();
    data->u = _ub.data();

    osqp_set_default_settings(settings);
    settings->eps_abs = 1e-5;
    settings->eps_rel = 1e-5;
    workspace = osqp_setup(data, settings);

    osqp_solve(workspace);

    _result = Eigen::Matrix<float, Eigen::Dynamic, 1>(varCount);
    for(u32 i = 0; i < varCount; i++) {
        _result[i] = workspace->solution->x[i];
    }
    quadraticCostMatrix.freeAll();
    constraintMatrix.freeAll();
}

u32 ConvexMPC::getStateIndex(u32 trajIdx){
    assert(trajIdx < _trajectoryLength);
    return trajIdx * 12;
}
u32 ConvexMPC::getControlIndex(u32 bBlockIdx){
    assert(bBlockIdx < _bBlockCount);
    return (_trajectoryLength * 12) + (bBlockIdx * 3);
}
u32 ConvexMPC::addConstraint(u32 size){
    u32 rv = _constraintCount;
    _constraintCount += size;
    return rv;
}
void ConvexMPC::addConstraintTriple(double value, u32 row, u32 col){
    assert(col < 12 * _trajectoryLength + 3 * _bBlockCount);
    assert(row < _constraintCount);
    if(value != 0) {
        _constraintTriples.push_back({value, row, col});
    }
}

Vec12 ConvexMPC::getResult() {
    Vec12 result;
    result.setZero();
    for(u32 i = 0; i < _bBlockIds.size(); i++) {
        auto& id = _bBlockIds[i];
        if(id.timestep == 0) {
            for(u32 j = 0; j < 3; j++) {
                result[id.foot*3 + j] = _result[getControlIndex(i) + j];
            }
        }
    }

    return result;
}

void ConvexMPC::sortAndSumTriples(std::vector<SparseTriple>& triples){
    sortTriples(triples, false);
    sumSortedTriples(triples);
}
void ConvexMPC::sortTriples(std::vector<SparseTriple>& triples, bool checkDuplicates){
    std::sort(triples.begin(), triples.end(), [](SparseTriple& a, SparseTriple& b){
    if(a.c == b.c) {
      return a.r < b.r;
    } else {
      return a.c < b.c;
    }
    });

    if(checkDuplicates && checkSortedTripleDuplicates(triples)) {
        throw std::runtime_error("duplicate found");
    }
}
void ConvexMPC::sumSortedTriples(std::vector<SparseTriple>& triples){
    std::vector<SparseTriple> temp = triples;
    u64 oldSize = triples.size();
    triples.clear();
    triples.reserve(oldSize);

    u32 lastRow = UINT32_MAX, lastCol = UINT32_MAX;

    for(auto& triple : temp) {
        if(triple.value == 0.0) continue;
        if(triple.r == lastRow && triple.c == lastCol) {
            triples.back().value += triple.value;
        } else {
            triples.push_back(triple);
        }
        lastRow = triple.r;
        lastCol = triple.c;
    }
}
OsqpCSC ConvexMPC::compress(std::vector<SparseTriple>& entries, u32 m, u32 n){
    OsqpCSC result;
    u32 nnz = entries.size();
    result.alloc(n, nnz);
    result.m = m;

    u32 i = 0;
    result.colPtrs[0] = 0;
    for(u32 c = 0; c < n; c++) {
        u32 cNNZ = 0;
        while(entries[i].c == c && i < nnz){
            result.values[i] = entries[i].value;
            result.rowIdx[i] = entries[i].r;
            assert(entries[i].r < m);
            assert(entries[i].c < n);
            assert(i < result.nnz);
            i++;
            cNNZ++;
        }
        result.colPtrs[c + 1] = result.colPtrs[c] + cNNZ;
        assert(c+1 < result.n + 1);
        assert(i == nnz || entries[i].c > c);
    }

    assert(i == nnz);

    return result;
}
bool ConvexMPC::checkSortedTripleDuplicates(std::vector<SparseTriple>& triples){
    for(u64 i = 0; i < triples.size() - 1; i++) {
        auto& a = triples[i];
        auto& b = triples[i + 1];
        if(a.r == b.r && a.c == b.c) {
            return true;
        }
    }
    return false;
}

ConvexMPC::~ConvexMPC(){



}

