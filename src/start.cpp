#include <syslog.h>
#include "FSM/FSM.h"
#include <unistd.h>
#include <csignal>
#include <sched.h>

void setProcessScheduler()
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1){
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

int main() {

    // setProcessScheduler();
    
    ControlComponent * ctrl = new ControlComponent();
    ctrl->dt = 0.003;    
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
