#ifndef START_H
#define START_H

#include <pthread.h>
#include "FSM/ControlComponent.h"
#include "FSM/FSM.h"
class start{
public:
    start(mjModel *model, mjData *data);
    ~start();
    void run();
    void reset();

    ControlComponent * ctrl;

private:
    pthread_t pthread;
    static void* lets_start(void * arg);
    bool _isruning;
    mjModel * mjmodel;
    mjData * mjdata;
    FSM * fsm;
};



#endif
