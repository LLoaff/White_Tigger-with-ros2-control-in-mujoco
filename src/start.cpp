#include "start.h"
#ifdef USE_SIM
start::start(mjModel *model, mjData *data):mjmodel(model),mjdata(data){
    // pthread_create(&pthread,NULL,start::lets_start,this);
    ctrl = new ControlComponent(mjmodel,mjdata);
    ctrl->dt = 0.002;    
    // ctrl->_period = 0.5;
    // ctrl->_stancePhaseRatio = 0.5;
    ctrl->_period = 0.5;
    ctrl->_stancePhaseRatio = 0.5;
    ctrl->waveGen = new WaveGenerator(ctrl->_period, ctrl->_stancePhaseRatio, 
                                        Vec4(0, 0.5, 0.5, 0),mjdata->time); // Trot
    ctrl->Estimator_Init();
    fsm = new FSM(ctrl);
}
#else
start::start(){
    // pthread_create(&pthread,NULL,start::lets_start,this);
    ctrl = new ControlComponent();
    ctrl->dt = 0.002;    
    // ctrl->_period = 0.5;
    // ctrl->_stancePhaseRatio = 0.5;
    ctrl->_period = 0.5;
    ctrl->_stancePhaseRatio = 0.5;
    ctrl->waveGen = new WaveGenerator(ctrl->_period, ctrl->_stancePhaseRatio, 
                                        Vec4(0, 0.5, 0.5, 0),0); // Trot
    ctrl->Estimator_Init();
    fsm = new FSM(ctrl);
}
#endif
start::~start(){
    // this->_isruning=false;
    delete fsm;
    delete ctrl;
    // pthread_join(pthread, NULL);
}
void start::run(){
    fsm->run();
}

void start::reset(){
    ctrl->setAllStance();
    ctrl->_ioros->SetZeroTau();
    ctrl->_ioros->SetZeroDq();
    ctrl->_ioros->SetZeroP();
    ctrl->_ioros->_state->_imu.Imu_Initial();
    ctrl->waveGen->reset(
        ctrl->_period,
        ctrl->_stancePhaseRatio,
        Vec4(0, 0.5, 0.5, 0),
        (double)getSystemTime()
    );
    fsm->initialize();
}

// void*start::lets_start(void * arg){
//     start* s = static_cast<start*>(arg);
//     s->_isruning=true;
//     s->ctrl = new ControlComponent(s->mjmodel,s->mjdata);
//     s->ctrl->dt = 0.002;    
//     s->ctrl->_period = 0.5;
//     s->ctrl->_stancePhaseRatio = 0.5;
//     s->ctrl->waveGen = new WaveGenerator(s->ctrl->_period, s->ctrl->_stancePhaseRatio, Vec4(0, 0.5, 0.5, 0)); // Trot
//     s->ctrl->Estimator_Init();
    
//     s->fsm = new FSM(s->ctrl);
//     while(s->_isruning){
//         s->fsm->run();
//     }
//     return NULL;
// }
