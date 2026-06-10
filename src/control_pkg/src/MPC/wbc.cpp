#include "MPC/wbc.h"
static void setWBCRealtimePriority() {
    struct sched_param param;
    param.sched_priority = 91;
    
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        printf("MPC: 无法设置实时线程优先级,请以root权限运行 \n");
    }
}
wbc::wbc(Estimator* est,LowState* lowstate,FeetEndCal* feetcal,CTP* ctp,mpc* mpc,Eigen::Matrix<int,4,1>& contact,const std::shared_ptr<IORos>& ioros,bool& _is_wbc_run)
:_est(est),iPb(est->iPb)
,_imu(&lowstate->_imu),_lowstate(lowstate)
,_ctp(ctp)
,_FootdesirePos(feetcal->_FootdesirePos)
,_FootdesireVelocity(feetcal->_FootdesireVelocity)
,_mpc(mpc),_contact(contact),_ioros(ioros)
,_is_wbc_run(_is_wbc_run){
    CreatTree();
    Eigen::Matrix3d B2W;
    B2W = _imu->GetRotMat().cast<double>();

    qdot = Eigen::MatrixXd(18,1);
    qddot = Eigen::MatrixXd(18,1);
    qcmd = Eigen::MatrixXd(18,1);
    qdotcmd = Eigen::MatrixXd(18,1);
    qddotcmd = Eigen::MatrixXd(18,1);
    detqcmd = Eigen::MatrixXd(18,1);
    q = Eigen::MatrixXd(18,1);
    qdotcmde = Eigen::MatrixXd(18,1);
    qddotcmde = Eigen::MatrixXd(18,1);

    J2 = Eigen::MatrixXd(3,18);
    J3 = Eigen::MatrixXd(3,18);
    J2q = Eigen::MatrixXd(3,1);
    J3q = Eigen::MatrixXd(3,1);
    e = Eigen::MatrixXd(3,1);
    x = Eigen::MatrixXd(3,1);

    Q1 = Eigen::MatrixXd(6,6);
    Q2 = Eigen::MatrixXd(12,12);
    G = Eigen::MatrixXd(18,18);
    
    XCi = Eigen::MatrixXd(6,6);
    C = Eigen::MatrixXd(18,1);
    M = Eigen::MatrixXd(18,18);

    // 先为 vector 分配空间，总共 14 个刚体节点 (0~13)
    Si.resize(14);
    X02I.resize(14);
    X02If.resize(14);
    Vspace.resize(14);
    Aspace.resize(14);
    Aspaced.resize(14);
    fi.resize(14);
    Vji.resize(14);
    qidot.resize(14);
    I.resize(14);
    Ic.resize(14);

    // 腿部的相关参数，总共 4 条腿
    Jb.resize(4);
    Jbi.resize(4);
    XQi.resize(4);
    AspaceQ.resize(4);
    VspaceQ.resize(4);

    Si[0] = Eigen::MatrixXd::Zero(6, 6); // 没有 Si0 随便初始
    Si[1] = Eigen::MatrixXd::Identity(6, 6);

    for (int i = 2; i < 14; ++i){
        Si[i].setZero(6, 1);
    }
    Si[2] << 1, 0, 0, 0, 0, 0;
    Si[5] << 1, 0, 0, 0, 0, 0;
    Si[8] << 1, 0, 0, 0, 0, 0;
    Si[11] << 1, 0, 0, 0, 0, 0;

    Si[3] << 0, 1, 0, 0, 0, 0;
    Si[4] << 0, 1, 0, 0, 0, 0;
    Si[6] << 0, 1, 0, 0, 0, 0;
    Si[7] << 0, 1, 0, 0, 0, 0;
    Si[9] << 0, 1, 0, 0, 0, 0;
    Si[10] << 0, 1, 0, 0, 0, 0;
    Si[12] << 0, 1, 0, 0, 0, 0;
    Si[13] << 0, 1, 0, 0, 0, 0;

    X02I[0] = Eigen::MatrixXd::Identity(6, 6);
    X02If[0] = Eigen::MatrixXd::Identity(6, 6);
    Vspace[0].resize(6, 1);
    Aspace[0].resize(6, 1);
    Vspace[0].setZero(6, 1);
    Aspace[0].setZero(6, 1);
    Aspaced[0].resize(6, 1);
    Aspaced[0] << 0, 0, 0, 0, 0, 9.81;
    J2 << B2W, Eigen::MatrixXd::Zero(3, 15);
    J3 << Eigen::Matrix3d::Zero(), B2W, Eigen::MatrixXd::Zero(3, 12);

    pi.push_back(4);
    pi.push_back(7);
    pi.push_back(10);
    pi.push_back(13);
    Jb[0].resize(6, 18);
    Jb[1].resize(6, 18);
    Jb[2].resize(6, 18);
    Jb[3].resize(6, 18);
    Jbi[0].resize(3, 18);
    Jbi[1].resize(3, 18);
    Jbi[2].resize(3, 18);
    Jbi[3].resize(3, 18);
    XCi << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
        -skew(Eigen::Vector3d(0, 0, -_lknee_)), Eigen::Matrix3d::Identity();
    J2q << Eigen::MatrixXd::Zero(3, 1);
    J3q << Eigen::MatrixXd::Zero(3, 1);

    // //
    std::vector<Eigen::Vector3d> P;
    P.resize(13);
    std::vector<Eigen::Vector4d> quat;
    quat.resize(13);
    std::vector<double> mass;
    mass.resize(4);
    std::vector<Eigen::Vector3d> diagnertia;
    diagnertia.resize(4);

    P[0]<< 0.008046, -0.001282, -0.001098;
    // P[1] = 右前髋(fr_hip), P[2] = 右前大腿(fr_thigh), P[3] = 右前小腿(fr_calf)
    P[1] << 0.0513844f, 0.000260068f, 4.15397e-05f;
    P[2] << -0.00013781f, -0.033735f, -0.0050592f;
    P[3] << 0.00973774f, -1.29057e-05f, -0.0786146f;

    // P[4] = 左前髋(fl_hip), P[5] = 左前大腿(fl_thigh), P[6] = 左前小腿(fl_calf)
    P[4] << 0.051352f, -0.00026007f, 4.1551e-05f;
    P[5] << -8.81413e-05f, 0.0337409f, -0.00509346f;
    P[6] << 0.00976003f, -1.29059e-05f, -0.0786135f;

    // P[7] = 右后髋(br_hip), P[8] = 右后大腿(br_thigh), P[9] = 右后小腿(br_calf)
    P[7] << -0.05141f, 0.00025902f, 1.5824e-05f;
    P[8] << -0.000175864f, -0.0337353f, -0.00510918f;
    P[9] << 0.0097567f, -1.76484e-06f, -0.0786146f;

    // P[10] = 左后髋(bl_hip), P[11] = 左后大腿(bl_thigh), P[12] = 左后小腿(bl_calf)
    P[10] << -0.05141f, -0.00025986f, -1.6754e-05f;
    P[11] << -0.000174634f, 0.0337409f, -0.00510805f;
    P[12] << 0.00975672f, -1.76485e-06f, -0.0786143f;

    mass[0] = 3.0f; // 机身质量
    mass[1] = 0.38;//髋
    mass[2] = 0.36f;//大腿
    mass[3] = 0.1f;//小腿

    diagnertia[0] << 0.0028,0.004,0.00555;
    diagnertia[1] << 0.000173054f, 0.000127484f, 0.000101665f;
    diagnertia[2] << 0.000200990f, 0.000150242f, 8.67838e-05f;
    diagnertia[3] << 0.000105994f, 0.000104495f, 2.76945e-06f;

    // quat[1] = 右前髋, quat[2] = 右前大腿, quat[3] = 右前小腿
    quat[1] << 0.458209f, 0.457541f, 0.537925f, 0.539757f;
    quat[2] << 0.704089f, 0.0262749f, 0.0256555f, 0.709161f;
    quat[3] << 0.707407f, -0.0170098f, -0.0169404f, 0.706398f;

    // quat[4] = 左前髋, quat[5] = 左前大腿, quat[6] = 左前小腿
    quat[4] << 0.538038f, 0.539445f, 0.458794f, 0.457189f;
    quat[5] << 0.708738f, 0.025685f, 0.0260514f, 0.704523f;
    quat[6] << 0.707409f, -0.017015f, -0.0169456f, 0.706397f;

    // quat[7] = 右后髋, quat[8] = 右后大腿, quat[9] = 右后小腿
    quat[7] << 0.538006f, 0.539517f, 0.45866f, 0.457277f;
    quat[8] << 0.704224f, 0.0250301f, 0.0247206f, 0.709106f;
    quat[9] << 0.707196f, -0.0169897f, -0.0169353f, 0.70661f;

    // quat[10] = 左后髋, quat[11] = 左后大腿, quat[12] = 左后小腿
    quat[10] << 0.458667f, 0.457263f, 0.537993f, 0.539535f;
    quat[11] << 0.709125f, 0.0257793f, 0.0264288f, 0.704115f;
    quat[12] << 0.707196f, -0.0169897f, -0.0169352f, 0.70661f;

    for (int i = 0; i < 13; ++i){
                                    
        Eigen::Matrix3d R = Quat2RotMat(quat[i].cast<float>()).cast<double>();
        int r = (i == 0) ? 0 : ((i - 1) % 3 + 1);
        //  I[i] = (R * diagnertia[r].asDiagonal() * R.transpose()) + mass[r] * (P[r].transpose() * P[r] * Eigen::Matrix3f::Identity() - P[r] * P[r].transpose());
        //====================================================//====================================================
        Eigen::Matrix3d I_3x3 = (R * diagnertia[r].asDiagonal() * R.transpose()) +
                                mass[r] * (P[i].dot(P[i]) * Eigen::Matrix3d::Identity() - P[i] * P[i].transpose());
        // 计算质心偏移向量的反对称矩阵 c_x
        Eigen::Matrix3d c_cross = skew(P[i]);

        // 重设尺寸为 6x6，并依照公式 5.40 将 4 个块拼装成完整的六维空间惯量
        I[i].resize(6, 6);
        I[i] << I_3x3, mass[r] * c_cross,
            mass[r] * c_cross.transpose(), mass[r] * Eigen::Matrix3d::Identity();
        //====================================================//====================================================//====================================================
    }

    WBC_Reset();
    _running = true;
    _wbc_thread = std::thread(&wbc::WBC_Loop, this);
    printf("WBC线程启动成功\n");
}
void wbc::WBC_Loop(){
    setWBCRealtimePriority();
    while(_running.load()){
        con = _contact;
        // std::cout<<"con: \n"<<con<<std::endl;
        if(_is_wbc_run){
          Dynamcis_Update();
          WBC_Update();
          usleep(1500);//0.002  2ms  2000us
        }
        else
          usleep(2000);//0.002  2ms  2000us
    }
}

void wbc::WBC_Update(){

    Eigen::Matrix3d B2W;
    B2W = _imu->GetRotMat().cast<double>();
    Eigen::Vector3d Wb;
    Wb = _imu->GetGyro().cast<double>();

    Eigen::Vector3d vcom;
    vcom = _est->getVcom();

    Eigen::Vector3d pcom;
    pcom = _est->getPcom();

    float roll = _imu->getRoll();
    float pitch = _imu->getYaw();
    float yaw = _imu->getYaw();

    Eigen::Matrix<double,12,1> jointpos;
    Eigen::Matrix<double,12,1> jointvel;

    jointpos = _lowstate->getJointpos();
    jointvel = _lowstate->getJointvel();

      // std::cout<<"jointpos  \n"<<jointpos<<std::endl;

    J2.setZero(3, 18);
    J3.setZero(3, 18);
    J2.block<3, 3>(0, 0) = B2W;
    J3.block<3, 3>(0, 3) = B2W;

    J2q.setZero();
    Eigen::Vector3d v_body = B2W.transpose() * vcom;
    J3q = B2W * (Wb.cross(B2W.transpose() * vcom));
    //  std::cout << "WBC_Update" << std::endl;
    //===================================
    // 0. 准备当前状态 q 和 qdot
    q.block(0, 0, 3, 1) << roll, pitch, yaw;
    q.block(3, 0, 3, 1) = pcom;
    q.block(6, 0, 12, 1) = jointpos;

      qdot.block(0, 0, 3, 1) = Wb;
      qdot.block(3, 0, 3, 1) = B2W.transpose() * vcom;
      qdot.block(6, 0, 12, 1) = jointvel;

      // 初始化层级矩阵与零空间
      Eigen::MatrixXd J_prev = J1;
      Eigen::MatrixXd NA = Eigen::MatrixXd::Identity(18, 18) - WideInverse(J_prev) * J_prev;

      detqcmd.setZero();
      qdotcmde.setZero();
      qddotcmde = WideInverse(J1) * (-J1q);
      // ---- 准备错误记录数据容器 ----
      Eigen::Vector3d err_ori, err_angvel, err_pos, err_vel;
      // 由于摆动腿数量会变，足端误差我们先初始化为零
      Eigen::Vector3d err_footpos = Eigen::Vector3d::Zero();
      Eigen::Vector3d err_footvel = Eigen::Vector3d::Zero();
      // ---- 任务 2：机身姿态转动控制 ----
      tran2 << cos(_ctp->desirex[0][1]) * cos(_ctp->desirex[0][2]), -sin(_ctp->desirex[0][2]), 0,
          cos(_ctp->desirex[0][1]) * sin(_ctp->desirex[0][2]), cos(_ctp->desirex[0][2]), 0,
          -sin(_ctp->desirex[0][1]), 0, 1;
      Eigen::Vector3d dfai = {_ctp->desirex[1][0], _ctp->desirex[1][1], _ctp->desirex[1][2]};
      Eigen::Vector3d fai = {_ctp->desirex[0][0], _ctp->desirex[0][1], _ctp->desirex[0][2]};
      Eigen::Vector3d dwO = {_ctp->desirex[1][6], _ctp->desirex[1][7], _ctp->desirex[1][8]};

      float kp_fai = 70;
      float kd_w = 50;
      e = tran2 * (dfai - fai);
      x = kd_w * (dwO - B2W * Wb) + kp_fai * (e);
      //   x = kd_w * (dwO - Wb) + kp_fai * (e);

      err_ori = dfai - fai;                // 1. 姿态角误差
      err_angvel = dwO - B2W * Wb; // 2. 角速度误差
      // std::cout<<"err_ori: \n"<<err_ori << std::endl;
      // std::cout<<"err_angvel: \n"<<err_angvel << std::endl;
      detqcmd += WideInverse(J2 * NA) * (e - J2 * detqcmd);
      qdotcmde += WideInverse(J2 * NA) * (dwO - J2 * qdotcmde);
      qddotcmde += WideInverse(J2 * NA) * (x - J2q - J2 * qddotcmde);

      // 叠加任务 1 和任务 2 的雅可比求新零空间
      Eigen::MatrixXd JA2(J_prev.rows() + J2.rows(), 18);
      JA2 << J_prev, J2;
      J_prev = JA2;
      NA = Eigen::MatrixXd::Identity(18, 18) - WideInverse(J_prev) * J_prev;
      // std::cout<<"detqcmd:\n"<<detqcmd<<std::endl;
      // ---- 任务 3：机身平动控制 ----
      Eigen::Vector3d dPo = {_ctp->desirex[1][3], _ctp->desirex[1][4], _ctp->desirex[1][5]};
      Eigen::Vector3d dVO = {_ctp->desirex[1][9], _ctp->desirex[1][10], _ctp->desirex[1][11]};

      float kp_pos = 50;
      float kd_vel = 25;

      e = dPo - pcom;
      x = kd_vel * (dVO - vcom) + kp_pos * (dPo - pcom);
      err_pos = dPo - pcom; // 3. 躯干位置误差
      err_vel = dVO - vcom; // 4. 躯干速度误差

      detqcmd += WideInverse(J3 * NA) * (e - J3 * detqcmd);
      qdotcmde += WideInverse(J3 * NA) * (dVO - J3 * qdotcmde);
      qddotcmde += WideInverse(J3 * NA) * (x - J3q - J3 * qddotcmde);

      // 叠加任务 1、2、3 的雅可比求新零空间
      Eigen::MatrixXd JA3(J_prev.rows() + J3.rows(), 18);
      JA3 << J_prev, J3;
      J_prev = JA3;
      NA = Eigen::MatrixXd::Identity(18, 18) - WideInverse(J_prev) * J_prev;
      // std::cout<<"detqcmd:\n"<<detqcmd<<std::endl;

      // ---- 任务 4：摆动腿足端轨迹控制 (最低优先级) ----
      Eigen::VectorXd ee(3 * J4.rows() / 3), Pfoot(3 * J4.rows() / 3), dPfoot(3 * J4.rows() / 3);
      Eigen::VectorXd Vfoot(3 * J4.rows() / 3), dVfoot(3 * J4.rows() / 3);
      int num = 0;
      for (int j = 0; j < 4; ++j)
      {
        if (con(j) == 0)
        { // 找到摆动腿
          Pfoot.block(3 * num, 0, 3, 1) = pcom + B2W * iPb.col(j);
          Vfoot.block(3 * num, 0, 3, 1) = (XQi[j] * XCi * Vspace[pi[j]]).block(3, 0, 3, 1);
          dPfoot.block(3 * num, 0, 3, 1) = _FootdesirePos.col(j);
          dVfoot.block(3 * num, 0, 3, 1) = _FootdesireVelocity.col(j);
          // 仅记录第一只找到的摆动腿作为可视化参考
          if (num == 0)
          {
            err_footpos = _FootdesirePos.col(j) - (pcom + B2W * iPb.col(j)); // 5. 足端位置误差
            err_footvel = _FootdesireVelocity.col(j) - Vfoot.block(0, 0, 3, 1);      // 6. 足端速度误差
          }
          num++;
        }
      }

      if (num > 0)
      {
        ee = dPfoot - Pfoot;
        float kd_swing = 30;
        float kp_swing = 50;
        x = kd_swing * (dVfoot - Vfoot) + kp_swing * (ee);

        detqcmd += WideInverse(J4 * NA) * (ee - J4 * detqcmd);
        qdotcmd = qdotcmde + WideInverse(J4 * NA) * (dVfoot - J4 * qdotcmde);
        qddotcmd = qddotcmde + WideInverse(J4 * NA) * (x - J4q - J4 * qddotcmde);
      }
      else
      { // 如果没有摆动腿 (比如4足站立阶段)
        qdotcmd = qdotcmde;
        qddotcmd = qddotcmde;
      }
      // std::cout<<"qddotcmd:\n"<<qddotcmd<<std::endl;

      qcmd = q + detqcmd;
      // std::cout<<"qcmd:\n"<<qcmd<<std::endl;

      // ================= 数据可视化映射 (新增) =================
    //   auto logWBCErrors = [&]()
    //   {
    //     std::stringstream ss;
    //     ss << "{";
    //     // 1. 姿态角误差 (Roll, Pitch, Yaw)
    //     ss << "\"err_ori_r\":" << err_ori[0] << ",\"err_ori_p\":" << err_ori[1] << ",\"err_ori_y\":" << err_ori[2] << ",";
    //     // 2. 角速度误差
    //     ss << "\"err_angvel_x\":" << err_angvel[0] << ",\"err_angvel_y\":" << err_angvel[1] << ",\"err_angvel_z\":" << err_angvel[2] << ",";
    //     // 3. 躯干位置误差 (X, Y, Z)
    //     ss << "\"err_pos_x\":" << err_pos[0] << ",\"err_pos_y\":" << err_pos[1] << ",\"err_pos_z\":" << err_pos[2] << ",";
    //     // 4. 躯干速度误差
    //     ss << "\"err_vel_x\":" << err_vel[0] << ",\"err_vel_y\":" << err_vel[1] << ",\"err_vel_z\":" << err_vel[2] << ",";
    //     // 5. 摆动足位置误差 (以代表腿为例)
    //     ss << "\"err_footpos_x\":" << err_footpos[0] << ",\"err_footpos_y\":" << err_footpos[1] << ",\"err_footpos_z\":" << err_footpos[2] << ",";
    //     // 6. 摆动足速度误差
    //     ss << "\"err_footvel_x\":" << err_footvel[0] << ",\"err_footvel_y\":" << err_footvel[1] << ",\"err_footvel_z\":" << err_footvel[2];
    //     ss << "}";
    //     visualizer.sendData(ss.str());
    //   };

    //   logWBCErrors(); // 执行发送

      int n_st = 0;
      for (int i = 0; i < 4; i++)
      {
        if (con(i) == 1)
          n_st++;
      }

      // 必须有支撑腿才进行优化
      if (n_st > 0)
      {
        int n_var = 18 + 3 * n_st; // 优化变量 X =[delta_q(18); delta_f(3*n_st)] (式 4.45)

        quadprogpp::Matrix<double> G_qp, CE_qp, CI_qp;
        quadprogpp::Vector<double> g_qp, ce0_qp, ci0_qp, x_qp;

        G_qp.resize(n_var, n_var);
        g_qp.resize(n_var);
        for (int i = 0; i < n_var; i++)
        {
          g_qp[i] = 0.0; // g 向量为 0 (因为目标函数无一次项)
          for (int j = 0; j < n_var; j++)
            G_qp[i][j] = 0.0;
        }

        // 1. 构建 G 矩阵 (式 4.46)
        // Q1 = I (18x18), Q2 = 0.005 * I (3n_st x 3n_st)
        for (int i = 0; i < 18; i++)
          G_qp[i][i] = 1.0;
        for (int i = 0; i < 3 * n_st; i++)
          G_qp[18 + i][18 + i] = 0.005;

        // 2. 构建等式约束 CE^T * X + ce0 = 0 (式 4.49, 4.50)
        int n_eq = 6;
        CE_qp.resize(n_var, n_eq);
        ce0_qp.resize(n_eq);

        Eigen::MatrixXd Mf = M.block(0, 0, 6, 18);                       // 提取 M 矩阵前 6 行
        Eigen::MatrixXd Cf = C.block(0, 0, 6, 1);                        // 提取 C 向量前 6 行
        Eigen::MatrixXd Jcf_T = J1.block(0, 0, 3 * n_st, 6).transpose(); // 提取 J1 对应机身的前 6 列的转置

        Eigen::MatrixXd CE_eigen(6, n_var);
        CE_eigen.block(0, 0, 6, 18) = Mf;
        CE_eigen.block(0, 18, 6, 3 * n_st) = -Jcf_T;
        Eigen::VectorXd current_Umpc_st(3 * n_st);
        current_Umpc_st.setZero();

        int st_idx = 0;
        for (int i = 0; i < 4; i++)
        {
          if (con(i) == 1)
          { // 只挑出当前真正在地上的腿
            // 从 12 维的 Umpc 里，摘出这根腿对应的力
            current_Umpc_st.block(3 * st_idx, 0, 3, 1) = _mpc->Umpc.block(3 * i, 0, 3, 1);
            st_idx++;
          }
        }
        Eigen::VectorXd ce0_eigen = -Jcf_T * current_Umpc_st + Cf + Mf * qddotcmd;
        //  Eigen::VectorXf ce0_eigen = -Jcf_T * ConvexMPC::Umpc_st + Cf + Mf * qddotcmd;

        // quadprogpp 期望的等式约束矩阵维度为 (n_var x n_eq)，所以赋值时需要转置
        for (int i = 0; i < n_var; i++)
        {
          for (int j = 0; j < n_eq; j++)
          {
            CE_qp[i][j] = CE_eigen(j, i);
          }
        }
        for (int j = 0; j < n_eq; j++)
          ce0_qp[j] = ce0_eigen(j);

        // 3. 构建不等式约束 CI^T * X + ci0 >= 0 (式 4.56, 4.57)
        int n_ineq = 10 * n_st; // 每条支撑腿对应 10 个不等式约束
        CI_qp.resize(n_var, n_ineq);
        ci0_qp.resize(n_ineq);
        for (int i = 0; i < n_var; i++)
          for (int j = 0; j < n_ineq; j++)
            CI_qp[i][j] = 0.0;

        float mu = _mpc->fri; // 请确保 ConvexMPC::fri 已经被赋了初值 (如 0.5)
        Eigen::MatrixXd CA_leg(5, 3);
        CA_leg << -1, 0, mu,
            0, -1, mu,
            1, 0, mu,
            0, 1, mu,
            0, 0, 1;

        Eigen::VectorXd c_bar_leg(5), c_under_leg(5);
        c_bar_leg << 10000, 10000, 10000, 10000, _mpc->Fmax; // 摩擦锥上限设为很大的值
        c_under_leg << 0, 0, 0, 0, 0;

        Eigen::MatrixXd CA = Eigen::MatrixXd::Zero(5 * n_st, 3 * n_st);
        Eigen::VectorXd c_bar(5 * n_st), c_under(5 * n_st);

        for (int i = 0; i < n_st; i++)
        {
          CA.block(5 * i, 3 * i, 5, 3) = CA_leg;
          c_bar.segment(5 * i, 5) = c_bar_leg;
          c_under.segment(5 * i, 5) = c_under_leg;
        }

        Eigen::MatrixXd CI_T_eigen = Eigen::MatrixXd::Zero(10 * n_st, n_var);
        CI_T_eigen.block(0, 18, 5 * n_st, 3 * n_st) = -CA;
        CI_T_eigen.block(5 * n_st, 18, 5 * n_st, 3 * n_st) = CA;

        Eigen::VectorXd ci0_eigen(10 * n_st);
        ci0_eigen.segment(0, 5 * n_st) = c_bar - CA * current_Umpc_st;
        ci0_eigen.segment(5 * n_st, 5 * n_st) = CA * current_Umpc_st - c_under;

        // quadprogpp 期望矩阵维度为 (n_var x n_ineq)
        for (int i = 0; i < n_var; i++)
        {
          for (int j = 0; j < n_ineq; j++)
          {
            CI_qp[i][j] = CI_T_eigen(j, i);
          }
        }
        for (int j = 0; j < n_ineq; j++)
          ci0_qp[j] = ci0_eigen(j);

        // 4. 调用 quadprogpp 进行求解
        double cost = quadprogpp::solve_quadprog(G_qp, g_qp, CE_qp, ce0_qp, CI_qp, ci0_qp, x_qp);

        // 5. 提取松弛变量结果，计算最终电机的下发扭矩 (式 4.61)
        Eigen::VectorXd delta_q(18), delta_f(3 * n_st);
        for (int i = 0; i < 18; i++)
          delta_q(i) = x_qp[i];
        for (int i = 0; i < 3 * n_st; i++)
          delta_f(i) = x_qp[18 + i];

        Eigen::VectorXd qddot_final = qddotcmd + delta_q;    // 最终广义加速度
        Eigen::VectorXd f_final = current_Umpc_st + delta_f; // 最终足底力

        Eigen::MatrixXd Mj = M.block(6, 0, 12, 18);
        Eigen::MatrixXd Cj = C.block(6, 0, 12, 1);
        Eigen::MatrixXd Jcj_T = J1.block(0, 6, 3 * n_st, 12).transpose(); // 仅取关节部分 (后12列) 的雅可比转置

        // 最终计算出的关节力矩 (可直接结合 PD 控制器发送给电机)
        Eigen::VectorXd tau = Mj * qddot_final + Cj - Jcj_T * f_final;

        _ioros->SetTau(tau.cast<float>());
        _ioros->SetQ((qcmd.bottomRows(12)).cast<float>());
        _ioros->SetDq((qdotcmd.bottomRows(12)).cast<float>());
        // std::cout<<"tau  \n"<<tau<<std::endl;
        // std::cout<<"Q  \n"<<qcmd.bottomRows(12)<<std::endl;
        // std::cout<<"DQ  \n"<<qdotcmd.bottomRows(12)<<std::endl;
        // 此处你可以将 tau 与 PD 的计算结果相加

        // PDcontrol::PDcontrol(qcmd.bottomRows(12), qdotcmd.bottomRows(12), tau);
      }
    
}

void wbc::Dynamcis_Update(){
    M.setZero();
    C.setZero();

    Eigen::Matrix3d B2W;
    B2W = _imu->GetRotMat().cast<double>();

    Eigen::Vector3d Wb;
    Wb = _imu->GetGyro().cast<double>();

    Eigen::Vector3d vcom;
    vcom = _est->getVcom();

    int n_st = 0;
    for (int i = 0; i < 4; i++){
        if (con(i) == 1)
          n_st++;
    }
    int n_sw = 4 - n_st;

    n_sw = 4 - n_st;
    J1.resize(3 * n_st, 18);
    J1.setZero();
    J1q.resize(3 * n_st, 1);
    J1q.setZero();

    J4.resize(3 * n_sw, 18);
    J4.setZero();
    J4q.resize(3 * n_sw, 1);
    J4q.setZero();
    int J1num = 0;
    int J4num = 0;
      // Eigen 使用 << 逗号初始化前，必须提前分配好尺寸！
    qidot[0].resize(6, 1);
    qidot[0] << Wb, B2W.transpose() * vcom;

    for (int i = 0; i < 12; ++i){
        qidot[i + 1].resize(1, 1);
        qidot[i + 1] << _lowstate->_motor_state[i].dq;
    }

    for (int i = 1; i < 14; ++i){

        X02I[i] = Transform_P2C(Vnode[i]) * X02I[Vnode[i]->parent.lock()->num];
        X02If[i] = (X02I[i].inverse()).transpose();
        Vji[i - 1] = Si[i] * qidot[i - 1];
        Vspace[i] = Transform_P2C(Vnode[i]) * Vspace[Vnode[i]->parent.lock()->num] + Vji[i - 1];
        // 忽略了 qddot 为了求 J1q J4q

        Aspace[i] = Transform_P2C(Vnode[i]) * Aspace[Vnode[i]->parent.lock()->num] + Vcross(Vspace[i]) * (Si[i] * qidot[i - 1]);
        Aspaced[i] = Transform_P2C(Vnode[i]) * Aspaced[Vnode[i]->parent.lock()->num] + Vcross(Vspace[i]) * (Si[i] * qidot[i - 1]);
        // 旋转惯量 没写
        fi[i - 1] = I[i - 1] * Aspaced[i] + Fcross(Vspace[i]) * (I[i - 1] * Vspace[i]);
    }

      // 反推 更新C矩阵  更新 Ic 组合刚体 空间惯量
    for (int i = 13; i > 0; --i)
    {
        if (i == 1)
          C.block(0, 0, 6, 1) = Si[i].transpose() * fi[i - 1];
        else
          C.block(i + 4, 0, 1, 1) = Si[i].transpose() * fi[i - 1];
        if (Vnode[i]->parent.lock()->num != 0)
        {
          // fi[Vnode[i]->parent.lock()->num - 1] = fi[Vnode[i]->parent.lock()->num - 1] + Transform_C2P(Vnode[i]) * fi[i - 1];
          fi[Vnode[i]->parent.lock()->num - 1] = fi[Vnode[i]->parent.lock()->num - 1] + Transform_C2PF(Vnode[i]) * fi[i - 1];
        }

        Ic[i - 1] = I[i - 1];
        for (auto node : Vnode[i]->child)
        { // 这里 i 的child 是j 们， 所以i->j  和j->i 是P2C 或者 C2P 的关系 ，直接用

          Ic[i - 1] = Ic[i - 1] + Transform_C2PF(Vnode[node->num]) * Ic[node->num - 1] * Transform_P2C(Vnode[node->num]);
        }

        if (i == 1)
        {
          M.block(0, 0, 6, 6) = Si[i].transpose() * Ic[i - 1] * Si[i];
        }
        else
        {
          M.block(i + 4, i + 4, 1, 1) = Si[i].transpose() * Ic[i - 1] * Si[i];
        }
        int j = i;
        Eigen::MatrixXd Xt = Eigen::MatrixXd::Identity(6, 6);
        while (Vnode[j]->parent.lock()->num > 0)
        {
          Xt = Xt * Transform_P2C(Vnode[j]);
          if (Vnode[j]->parent.lock()->num == 1)
          {
            M.block(i + 4, 0, 1, 6) = Si[i].transpose() * Ic[i - 1] * Xt * Si[Vnode[j]->parent.lock()->num];

            M.block(0, i + 4, 6, 1) = M.block(i + 4, 0, 1, 6).transpose();
          }
          else
          {
            M.block(i + 4, Vnode[j]->parent.lock()->num + 4, 1, 1) = Si[i].transpose() * Ic[i - 1] * Xt * Si[Vnode[j]->parent.lock()->num];

            M.block(Vnode[j]->parent.lock()->num + 4, i + 4, 1, 1) = Si[i].transpose() * Ic[i - 1] * Xt * Si[Vnode[j]->parent.lock()->num];
          }

          j = Vnode[j]->parent.lock()->num;
        }
    }

      for (int i = 0; i < 4; ++i)
    {
        int j = pi[i];
        //
        XQi[i].resize(6, 6);
        XQi[i] << X02I[pi[i]].block(3, 3, 3, 3).transpose(), Eigen::Matrix3d::Zero(),
            Eigen::Matrix3d::Zero(), X02I[pi[i]].block(3, 3, 3, 3).transpose();
        Eigen::MatrixXd Xjpi = Eigen::MatrixXd::Identity(6, 6);
        // Jb 使用 .block 赋值前，必须分配 6x18 的尺寸并清零！
        Jb[i].resize(6, 18);
        Jb[i].setZero();
        Jb[i].block(0, pi[i] + 4, 6, 1) = Si[pi[i]];
        while (Vnode[j]->parent.lock()->num > 0)
        {
          Xjpi = Xjpi * Transform_P2C(Vnode[j]);
          j = Vnode[j]->parent.lock()->num;
          if (j == 1)
            Jb[i].block(0, 0, 6, 6) = Xjpi * Si[j];
          else
            Jb[i].block(0, j + 4, 6, 1) = Xjpi * Si[j];
        }
        //  转向定向足底坐标系
        Jbi[i] = (XQi[i] * XCi * Jb[i]).block(3, 0, 3, 18);
        AspaceQ[i] = (XQi[i] * XCi) * Aspace[pi[i]];
        VspaceQ[i] = (XQi[i] * XCi) * Vspace[pi[i]];

        //======================
        if (con(i) == 1) // 支撑腿
        {
          J1.block(3 * J1num, 0, 3, 18) = Jbi[i];
          // 显式转换为 Vector3f
          J1q.block(3 * J1num, 0, 3, 1) = AspaceQ[i].block(3, 0, 3, 1) + Eigen::Vector3d(VspaceQ[i].block(0, 0, 3, 1)).cross(Eigen::Vector3d(VspaceQ[i].block(3, 0, 3, 1)));
          J1num++;
        }
        else // 摆动腿
        {
          J4.block(3 * J4num, 0, 3, 18) = Jbi[i];
          // 显式转换为 Vector3f
          J4q.block(3 * J4num, 0, 3, 1) = AspaceQ[i].block(3, 0, 3, 1) + Eigen::Vector3d(VspaceQ[i].block(0, 0, 3, 1)).cross(Eigen::Vector3d(VspaceQ[i].block(3, 0, 3, 1)));
          J4num++;
        }

        //===================
    }
    // std::cout<<"J1.rows():\n"<<J1.rows()<<std::endl;
}


void wbc::WBC_Reset(){
    q.setZero();
    qdot.setZero();
    qddot.setZero();
    qcmd.setZero();
    qdotcmd.setZero();
    qddotcmd.setZero();
    qdotcmde.setZero();
    qddotcmde.setZero();
    detqcmd.setZero();
    M.setZero();
    C.setZero();
    J1.resize(0, 18);
    J4.resize(0, 18);
    J1q.resize(0, 1);
    J4q.resize(0, 1);
}


Eigen::MatrixXd wbc::WideInverse(const Eigen::MatrixXd &mat){
    if (mat.rows() == 0 || mat.cols() == 0)
        return Eigen::MatrixXd::Zero(mat.cols(), mat.rows());

    float lambda = 1e-4f; // 阻尼系数，保护奇点
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(mat.rows(), mat.rows());

    // J# = J^T * (J * J^T + lambda^2 * I)^-1
    return mat.transpose() * (mat * mat.transpose() + lambda * lambda * I).inverse();
}

Eigen::MatrixXd wbc::Vcross(const Eigen::MatrixXd &V){
    Eigen::MatrixXd Vx(6, 6);
    Vx << skew(Eigen::Vector3d(V.block(0, 0, 3, 1))), Eigen::Matrix3d::Zero(),
        skew(Eigen::Vector3d(V.block(3, 0, 3, 1))), skew(Eigen::Vector3d(V.block(0, 0, 3, 1)));
    return Vx;
}

Eigen::MatrixXd wbc::Fcross(const Eigen::MatrixXd &V){
    Eigen::MatrixXd Vx(6, 6);
    Vx << skew(Eigen::Vector3d(V.block(0, 0, 3, 1))), skew(Eigen::Vector3d(V.block(3, 0, 3, 1))),
        Eigen::Matrix3d::Zero(), skew(Eigen::Vector3d(V.block(0, 0, 3, 1)));
    return Vx;
}

Eigen::MatrixXd wbc::Transform_C2PF(std::shared_ptr<node> node){
    Eigen::MatrixXd TFMatrix(6, 6);
    Eigen::Matrix3d B2W;
    B2W = _imu->GetRotMat().cast<double>();
    if (node->num == 0)
    {
        return Eigen::MatrixXd::Identity(6, 6);
    }
    else if (node->num == 1)
    {
        TFMatrix << B2W,
            // B2W * skew(pcom),
            skew(_est->getPcom()) * B2W,
            Eigen::Matrix3d::Zero(),
            B2W;
        return TFMatrix;
    }
    else if (node->num == 2 || node->num == 5 || node->num == 8 || node->num == 11)
    {
        Eigen::Vector3d P;
        if (node->num == 2)
          P << _length_, -_weigh_, 0;
        else if (node->num == 5)
          P << _length_, _weigh_, 0;
        else if (node->num == 8)
          P << -_length_, -_weigh_, 0;
        else if (node->num == 11)
          P << -_length_, _weigh_, 0;

        TFMatrix << rotx(_lowstate->_motor_state[node->num - 2].q),
            rotx(_lowstate->_motor_state[node->num - 2].q) * skew(P),
            Eigen::Matrix3d::Zero(),
            rotx(_lowstate->_motor_state[node->num - 2].q);
        return TFMatrix;
    }
    else
    {
        Eigen::Vector3d P;
        if (node->num == 3)
          P << 0, -_labad_, 0;
        else if (node->num == 6)
          P << 0, _labad_, 0;
        else if (node->num == 9)
          P << 0, -_labad_, 0;
        else if (node->num == 12)
          P << 0, _labad_, 0;
        else
        {
          P << 0, 0, -_lhip_;
        }

        TFMatrix << roty(_lowstate->_motor_state[node->num - 2].q),
            roty(_lowstate->_motor_state[node->num - 2].q) * skew(P),
            Eigen::Matrix3d::Zero(),
            roty(_lowstate->_motor_state[node->num - 2].q);
        return TFMatrix;
    }
}


Eigen::MatrixXd wbc::Transform_C2P(std::shared_ptr<node> node){
    Eigen::MatrixXd TFMatrix(6, 6);
    Eigen::Matrix3d B2W;
    B2W = _imu->GetRotMat().cast<double>();
    if (node->num == 0)
    {
        return Eigen::MatrixXd::Identity(6, 6);
    }
    else if (node->num == 1)
    {
        // TFMatrix << B2W, Eigen::Matrix3f::Zero(),
        //     B2W * skew(pcom), B2W;
        TFMatrix << B2W, Eigen::Matrix3d::Zero(),
            skew(_est->getPcom()) * B2W, B2W;
        return TFMatrix;
    }
    else if (node->num == 2 || node->num == 5 || node->num == 8 || node->num == 11)
    {
        Eigen::Vector3d P;
        if (node->num == 2)
          P << _length_, -_weigh_, 0;
        else if (node->num == 5)
          P << _length_, _weigh_, 0;
        else if (node->num == 8)
          P << -_length_, -_weigh_, 0;
        else if (node->num == 11)
          P << -_length_, _weigh_, 0;

        TFMatrix << rotx(_lowstate->_motor_state[node->num - 2].q), Eigen::Matrix3d::Zero(),
            rotx(_lowstate->_motor_state[node->num - 2].q) * skew(P), rotx(_lowstate->_motor_state[node->num - 2].q);
        return TFMatrix;
    }
    else
    {
        Eigen::Vector3d P;
        if (node->num == 3)
          P << 0, -_labad_, 0;
        else if (node->num == 6)
          P << 0, _labad_, 0;
        else if (node->num == 9)
          P << 0, -_labad_, 0;
        else if (node->num == 12)
          P << 0, _labad_, 0;
        else
        {
          P << 0, 0, -_lhip_;
        }

        TFMatrix << roty(_lowstate->_motor_state[node->num - 2].q), Eigen::Matrix3d::Zero(),
            roty(_lowstate->_motor_state[node->num - 2].q) * skew(P), roty(_lowstate->_motor_state[node->num - 2].q);
        return TFMatrix;
    }
}


Eigen::MatrixXd wbc::Transform_P2C(std::shared_ptr<node> node){
    Eigen::MatrixXd TFMatrix(6, 6);
    Eigen::Matrix3d B2W;
    B2W = _imu->GetRotMat().cast<double>();
    if (node->num == 0)
    {
    return Eigen::MatrixXd::Identity(6, 6);
    }
    else if (node->num == 1)
    {
        TFMatrix << B2W.transpose(), Eigen::Matrix3d::Zero(),
            -B2W.transpose() * skew(_est->getPcom()), B2W.transpose();
        return TFMatrix;
    }
    else if (node->num == 2 || node->num == 5 || node->num == 8 || node->num == 11)
    {
        Eigen::Vector3d P;
        if (node->num == 2)
            P << _length_, -_weigh_, 0;
        else if (node->num == 5)
            P << _length_, _weigh_, 0;
        else if (node->num == 8)
            P << -_length_, -_weigh_, 0;
        else if (node->num == 11)
            P << -_length_, _weigh_, 0;
        TFMatrix << rotx(_lowstate->_motor_state[node->num - 2].q).transpose(), Eigen::Matrix3d::Zero(),
            -rotx(_lowstate->_motor_state[node->num - 2].q).transpose() * skew(P), rotx(_lowstate->_motor_state[node->num - 2].q).transpose();
        return TFMatrix;
    }
    else
    {
        Eigen::Vector3d P;
        if (node->num == 3)
          P << 0, -_labad_, 0;
        else if (node->num == 6)
          P << 0, _labad_, 0;
        else if (node->num == 9)
          P << 0, -_labad_, 0;
        else if (node->num == 12)
          P << 0, _labad_, 0;
        else
        {
          P << 0, 0, -_lhip_;
        }

        TFMatrix << roty(_lowstate->_motor_state[node->num - 2].q).transpose(), Eigen::Matrix3d::Zero(),
            -roty(_lowstate->_motor_state[node->num - 2].q).transpose() * skew(P), roty(_lowstate->_motor_state[node->num - 2].q).transpose();
        return TFMatrix;
    }
}


void wbc::CreatTree(){
    bind(node0, node1);
    bind(node1, node2);
    bind(node1, node5);
    bind(node1, node8);
    bind(node1, node11);
    bind(node2, node3);
    bind(node3, node4);
    bind(node5, node6);
    bind(node6, node7);
    bind(node8, node9);
    bind(node9, node10);
    bind(node11, node12);
    bind(node12, node13);
    Vnode.push_back(node0);
    Vnode.push_back(node1);
    Vnode.push_back(node2);
    Vnode.push_back(node3);
    Vnode.push_back(node4);
    Vnode.push_back(node5);
    Vnode.push_back(node6);
    Vnode.push_back(node7);
    Vnode.push_back(node8);
    Vnode.push_back(node9);
    Vnode.push_back(node10);
    Vnode.push_back(node11);
    Vnode.push_back(node12);
    Vnode.push_back(node13);
    is_tree_created = true; // 新增：树建立完毕，允许计算
}

void wbc::bind(std::shared_ptr<node> &parent, std::shared_ptr<node> &child){
    parent->add_child(child);
    child->parent = parent;
}