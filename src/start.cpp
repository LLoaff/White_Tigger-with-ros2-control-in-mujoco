#include <syslog.h>
#include "FSM/FSM.h"
#include <unistd.h>
#include <csignal>
#include <sched.h>

// void setProcessScheduler()
// {
//     pid_t pid = getpid();
//     sched_param param;
//     param.sched_priority = sched_get_priority_max(SCHED_FIFO);
//     if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1){
//         std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
//     }
// }

static void setMPCRealtimePriority(int cpu_core_id) {
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    int sched_ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (sched_ret != 0){
        std::cout << "[ERROR] MPC 设置实时优先级失败,请以root运行: "
                  << std::strerror(sched_ret) << std::endl;
    }
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);               // 清空CPU集合
    CPU_SET(cpu_core_id, &cpuset);   // 把目标核心加入集合

    // pthread_self() 获取当前线程自身的ID
    int ret = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    if (ret != 0) {
        std::cout << "[ERROR] MPC 绑定CPU核心失败, core=" << cpu_core_id << std::endl;
    }
}

int main() {

    // setMPCRealtimePriority(2);
    
    ControlComponent * ctrl = new ControlComponent();
    ctrl->dt = 0.002;    
    ctrl->_period = 0.5;
    ctrl->_stancePhaseRatio = 0.5;
    ctrl->waveGen = new WaveGenerator(ctrl->_period, ctrl->_stancePhaseRatio, Vec4(0, 0.5, 0.5, 0)); // Trot

    ctrl->Estimator_Init();

    FSM * fsm = new FSM(ctrl);


    while(1) 
    {
        fsm->run();
    }

    delete ctrl,fsm;

    return 0;
}
