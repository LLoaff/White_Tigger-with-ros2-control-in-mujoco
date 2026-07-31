#include <syslog.h>
#include "FSM/FSM.h"
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include "rclcpp/rclcpp.hpp"
// void setProcessScheduler()
// {
//     pid_t pid = getpid();
//     std::cout<<"线程PID: "<<pid<<std::endl;
//     sched_param param;
//     param.sched_priority = sched_get_priority_max(SCHED_FIFO);
//     if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1){
//         std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
//     }
// }
bool setControlThreadRealtime(int priority, int cpu_id) {
    pthread_t self = pthread_self();
    pid_t pid = getpid();
    std::cout<<"线程PID: "<<pid<<std::endl;
    // 设置SCHED_FIFO实时调度
    sched_param param;
    param.sched_priority = priority;
    int ret = pthread_setschedparam(self, SCHED_FIFO, &param);
    if (ret != 0) {
        std::cerr << "[ERROR] 设置实时优先级失败: " << strerror(ret) << std::endl;
        return false;
    }

    // 绑定到指定CPU核心（对应你内核隔离的核心，比如CPU3）
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);
    ret = pthread_setaffinity_np(self, sizeof(cpu_set_t), &cpuset);
    if (ret != 0) {
        std::cerr << "[ERROR] 设置CPU亲和性失败: " << strerror(ret) << std::endl;
        return false;
    }
    return true;
}

int main(int argc,char** argv) {

    // setProcessScheduler();
    rclcpp::init(argc,argv);
    
    ControlComponent * ctrl = new ControlComponent();
    

    ctrl->Estimator_Init();

    FSM * fsm = new FSM(ctrl);
    if (!setControlThreadRealtime(99, 3)) {
        std::cerr << "[WARNING] 实时配置未生效,检查limits.conf和用户组" << std::endl;
    }
    while(rclcpp::ok()) 
    {
        fsm->run();
    }

    delete ctrl,fsm;

    return 0;
}
