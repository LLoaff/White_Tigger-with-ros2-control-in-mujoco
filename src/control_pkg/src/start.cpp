#include <syslog.h>
#include "FSM/FSM.h"
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include "rclcpp/rclcpp.hpp"

void setProcessScheduler()
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1){
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

int main(int argc,char** argv) {

    setProcessScheduler();
    rclcpp::init(argc,argv);
    
    ControlComponent * ctrl = new ControlComponent();
    

    ctrl->Estimator_Init();

    FSM * fsm = new FSM(ctrl);


    while(rclcpp::ok()) 
    {
        fsm->run();
    }

    delete ctrl,fsm;

    return 0;
}
