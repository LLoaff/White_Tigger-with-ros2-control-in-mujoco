#include "MPC/mpc.h"

static void setMPCRealtimePriority() {
    struct sched_param param;
    param.sched_priority = 90;
    
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        printf("MPC: 无法设置实时线程优先级,请以root权限运行 \n");
    }
}

mpc::mpc(CTP * ctp,Estimator* est,WaveGenerator* wave,Eigen::Matrix<int,4,1>& contact,bool&_is_wbc_run)
:_ctp(ctp),_est(est),_wave(wave),_contact(contact),_is_wbc_run(_is_wbc_run){
  Temp_A = Eigen::MatrixXd(13,13);
  A = Eigen::MatrixXd(13,13);
  Continue_B = Eigen::MatrixXd(13,13);

  
  vec = Eigen::VectorXd(13);
  temp = Eigen::MatrixXd(13,13);
  Umpc = Eigen::MatrixXd(12,1);
  
  A.setZero();
  A.setIdentity(13, 13);
  A.block(3, 9, 3, 3) = Eigen::Matrix3d::Identity() * MPC_T;
  A(11, 12) = 1 * MPC_T;

  Q.resize(13 * _mpc_steps, 13 * _mpc_steps);
  Q.setZero();

  Eigen::Vector4f q;
  q << 0,0.99975,-0.00625,0.01575;
  Eigen::Matrix3d Ro = Quat2RotMat(q).cast<double>();
  Eigen::Vector3d diaginertia(0.036518, 0.134130, 0.157041);

  float mm = 3.0f;// 躯干质量
  Eigen::Matrix3d I_principal = diaginertia.asDiagonal();
  Eigen::Vector3d P(0.008046, -0.001282, -0.001098);// 质心偏移

  BInertia = (Ro * I_principal * (Ro.transpose())) + mm * (P.dot(P) * Eigen::Matrix3d::Identity() - P * P.transpose());

  vec << 25, 25, 10, 1, 1, 100, 0, 0, 0.3, 0.2, 0.2, 20, 0;
  temp = vec.asDiagonal();

  for (int i = 0; i < _mpc_steps; ++i){
      Q.block(i * 13, i * 13, 13, 13) = temp;
  }

  reset();
  _running = true;
  _mpc_thread = std::thread(&mpc::MPC_Loop, this);
  printf("MPC线程启动成功\n");
}
void mpc::MPC_Loop(){
  setMPCRealtimePriority();
  while(_running.load()){
    if(_is_wbc_run){
      update();
    }
    usleep(8000);//100hz  10ms 10*1000us
    // absoluteWait(_fsm_ctrl->_ioros.get(),_start_time, (long long)(_fsm_ctrl->dt * 1000000));       // dt 需初始化时手动赋值
  }
}

void mpc::update(){
    MPCsFai << _contact(0),_contact(1),_contact(2),_contact(3);
    // std::cout<<"MPCsFai: \n"<<MPCsFai<<std::endl;
    // 过滤出有效支撑腿:
    std::vector<int> act_legs;
    for(int i(0);i<4;++i){
        if(MPCsFai[i] == 1){
            act_legs.push_back(i); // 存储触地腿的contact id
        }
    }
    n_st = act_legs.size(); // 获取当前支撑腿个数

    // 如果全悬空，则发0力矩
    if (n_st == 0){
        Umpc_st.resize(0);
        Umpc.setZero();
        return;
    }

    int n_vars = 3 * n_st * _mpc_steps; // QP优化变量数量 (60 或 120)
    int n_cons = 5 * n_st * _mpc_steps; // QP约束数量 (100 或 200)

    // 2. 动态分配底层矩阵的内存
    Aqp.resize(13 * _mpc_steps, 13);
    Bqp.resize(13 * _mpc_steps, n_vars);
    Bqp.setZero();

    R.resize(n_vars, n_vars);
    R.setIdentity();
    R *= 1e-5f * 5;

    // xyz力矩的上限、下限
    lb.resize(n_vars, 1);
    ub.resize(n_vars, 1);

    // 约束的常数相
    Ampc.resize(n_cons, n_vars);
    Ampc.setZero();
    uba.resize(n_cons, 1);

    lba.resize(n_cons, 1);
    lba.setZero();

    for (int i = 0; i < _mpc_steps; ++i){
        if (i == 0)
          Temp_A.setIdentity();
        A.block(0, 6, 3, 3) = (rotz(_ctp->desirex[i][2]).transpose()) * MPC_T;
        Temp_A = A * Temp_A;
        Aqp.block(13 * i, 0, 13, 13) = Temp_A;
      }

    float m = 5.5; // 总质量

    for (int i = 0; i < _mpc_steps; i++){
        Eigen::Matrix3d Ro = Rpy2RotMat(0, 0, _ctp->desirex[i][2]).cast<double>();

        PInertia = Ro * BInertia * (Ro.transpose());

        // 仅对有效支撑腿计算 B 矩阵映射
        Eigen::MatrixXd B_step = Eigen::MatrixXd::Zero(13, 3 * n_st);
        for (int leg_idx = 0; leg_idx < n_st; leg_idx++){
          int leg = act_legs[leg_idx];
          B_step.block(6, 3 * leg_idx, 3, 3) = (PInertia.inverse()) * skew(_wave->_FootPos.col(leg) - _est->getPcom()) * MPC_T;
          B_step.block(9, 3 * leg_idx, 3, 3) = Eigen::Matrix3d::Identity() * MPC_T / m;
        }

        for (int j = i + 1; j < _mpc_steps + 1; j++){
          if (j == i + 1)
            Temp_A.setIdentity();
          A.block(0, 6, 3, 3) = (rotz(_ctp->desirex[j][2]).transpose()) * MPC_T;
          Bqp.block((j - 1) * 13, i * 3 * n_st, 13, 3 * n_st) = Temp_A * B_step;
          Temp_A = A * Temp_A;
        }
      }

      // 构建力边界和摩擦锥约束
      Eigen::MatrixXd t(5, 3);
      t << -1, 0, fri, 0, -1, fri, 1, 0, fri, 0, 1, fri, 0, 0, 1;
      Eigen::VectorXd vub(5);
      vub << 1e8, 1e8, 1e8, 1e8, Fmax;

      for (int i = 0; i < _mpc_steps; i++){
        for (int leg_idx = 0; leg_idx < n_st; leg_idx++){
          // 设置 X Y Z 三个维度的上下界
          lb(i * 3 * n_st + 3 * leg_idx + 0) = -Fmax * fri;
          ub(i * 3 * n_st + 3 * leg_idx + 0) = Fmax * fri;
          lb(i * 3 * n_st + 3 * leg_idx + 1) = -Fmax * fri;
          ub(i * 3 * n_st + 3 * leg_idx + 1) = Fmax * fri;
          lb(i * 3 * n_st + 3 * leg_idx + 2) = 0.f;
          ub(i * 3 * n_st + 3 * leg_idx + 2) = Fmax;

          // 填入摩擦锥
          Ampc.block(i * 5 * n_st + 5 * leg_idx, i * 3 * n_st + 3 * leg_idx, 5, 3) = t;
          uba.block(i * 5 * n_st + 5 * leg_idx, 0, 5, 1) = vub;
        }
      }

    // 组装QP矩阵
    D = _ctp->D; // 提取当前的 D 矩阵
    H = 2 * (Bqp.transpose() * Q * Bqp + R);
    g = 2 * Bqp.transpose() * Q * (Aqp * _ctp->desirex[0] - D);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_qp = H;
    Eigen::VectorXd g_qp = g.cast<double>();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_qp = Ampc;
    Eigen::VectorXd lb_qp = lb;
    Eigen::VectorXd ub_qp = ub;
    Eigen::VectorXd lba_qp = lba;
    Eigen::VectorXd uba_qp = uba;

    // qpOASES 求解器生命周期 (仅在变量维度变化时重建求解器)
    if (n_vars != last_n_vars || qp_solver == nullptr){
        if (qp_solver != nullptr)
            delete qp_solver;
        qp_solver = new qpOASES::QProblem(n_vars, n_cons);
        qpOASES::Options option;
        option.setToMPC();
        option.printLevel = qpOASES::PL_NONE;
        qp_solver->setOptions(option);
        last_n_vars = n_vars;
    }
    int nWSR_actual = 100;
    qpOASES::returnValue status = qp_solver->init(
        H_qp.data(), g_qp.data(), A_qp.data(),
        lb_qp.data(), ub_qp.data(), lba_qp.data(), uba_qp.data(),
    nWSR_actual);

    if (status == qpOASES::SUCCESSFUL_RETURN){
        double xOpt[n_vars];
        qp_solver->getPrimalSolution(xOpt);

        // 直接提取完美对齐的支撑腿结果给 WBC
        Umpc_st.resize(3 * n_st);
        for (int i = 0; i < 3 * n_st; i++){
          Umpc_st[i] = xOpt[i];
        }

        // (可选) 填充 12 维的 Umpc 用于日志或外部可视化
        Umpc.setZero();
        for (int leg_idx = 0; leg_idx < n_st; leg_idx++){
          Umpc.block(3 * act_legs[leg_idx], 0, 3, 1) = Umpc_st.block(3 * leg_idx, 0, 3, 1);
        }
        std::cout << "Umpc:\n" << Umpc << std::endl;

    }
      else{
        std::cout << "MPC Solve Failed--------" << status << std::endl;
    }
}

void mpc::reset(){
    if (qp_solver != nullptr){
        delete qp_solver;
        qp_solver = nullptr;
      }
      last_n_vars = 0;
      n_st = 0;
      Umpc.setZero();
      Umpc_st.resize(0);
}

mpc::~mpc() {
    // 通知线程退出
    _running = false;
    
    // 等待线程结束
    if (_mpc_thread.joinable()) {
        _mpc_thread.join();
    }
    
    // 释放QP求解器资源
    if (qp_solver != nullptr) {
        delete qp_solver;
    }
    
    printf("MPC线程已退出\n");
}