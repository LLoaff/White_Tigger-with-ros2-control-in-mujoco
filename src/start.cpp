#include "start.h"

start::start(mjModel *model, mjData *data):mjmodel(model),mjdata(data){
    pthread_create(&pthread,NULL,start::lets_start,this);
   
}
start::~start(){
    this->_isruning=false;
    delete ctrl;
    delete fsm;
    pthread_join(pthread, NULL);
}

void*start::lets_start(void * arg){
    start* s = static_cast<start*>(arg);
    s->_isruning=true;
    s->ctrl = new ControlComponent(s->mjmodel,s->mjdata);
    s->ctrl->dt = 0.002;    
    s->ctrl->_period = 0.5;
    s->ctrl->_stancePhaseRatio = 0.5;
    s->ctrl->waveGen = new WaveGenerator(s->ctrl->_period, s->ctrl->_stancePhaseRatio, Vec4(0, 0.5, 0.5, 0)); // Trot
    s->ctrl->Estimator_Init();
    
    s->fsm = new FSM(s->ctrl);
    while(s->_isruning){
        s->fsm->run();
    }
    return NULL;
}

