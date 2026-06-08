#ifndef TIMEMARKER_H
#define TIMEMARKER_H

#include <iostream>
#include <sys/time.h>
#include <unistd.h>
#include "IOPort/IORos.h"

//时间戳  微秒级， 需要#include <sys/time.h> 
inline long long getSystemTime(){
    struct timeval t;  
    gettimeofday(&t, NULL);
    return 1000000 * t.tv_sec + t.tv_usec;  
}
//时间戳  秒级， 需要getSystemTime()
inline double getTimeSecond(){
    double time = getSystemTime() * 0.000001;
    return time;
}
//等待函数，微秒级，从startTime开始等待waitTime微秒
inline void absoluteWait(IORos * ros,long long startTime, long long waitTime){
    long long currentTimeUs = static_cast<long long>(ros->get_simulation_time() * 1000000.0);
    if(currentTimeUs - startTime > waitTime){
        // std::cout << "[WARNING] The waitTime=" << waitTime << " of function absoluteWait is not enough!" << std::endl
        // << "The program has already cost " << currentTimeUs - startTime << "us." << std::endl;
    }
    while(static_cast<long long>(ros->get_simulation_time() * 1000000.0) - startTime < waitTime){
        usleep(50);
        // std::cout<<"now time: "<<static_cast<long long>(ros->get_simulation_time() * 1000000.0)<<" startTime: "<<startTime<<" waitTime: "<<waitTime<<std::endl;
    }
}

#endif //TIMEMARKER_H