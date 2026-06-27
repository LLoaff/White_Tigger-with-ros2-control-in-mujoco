
#include "low/LowState.h"

LowState::LowState():_motor_data{
        damiao::Motor(damiao::DM4310, 1, 0),  
        damiao::Motor(damiao::DM4310, 2, 0),  
        damiao::Motor(damiao::DM4310, 3, 0), 
        
        damiao::Motor(damiao::DM4310, 4, 0),  
        damiao::Motor(damiao::DM4310, 5, 0),  
        damiao::Motor(damiao::DM4310, 6, 0),  
        
        damiao::Motor(damiao::DM4310, 7, 0),  
        damiao::Motor(damiao::DM4310, 8, 0),  
        damiao::Motor(damiao::DM4310, 9, 0),  
        
        damiao::Motor(damiao::DM4310, 10, 0), 
        damiao::Motor(damiao::DM4310, 11, 0), 
        damiao::Motor(damiao::DM4310, 12, 0) 
}{

}

Eigen::Matrix<double,12,1> LowState::getJointpos(){
        Eigen::Matrix<double,12,1> jointpos;
        jointpos<< _motor_state[0].q,_motor_state[1].q,_motor_state[2].q,
                   _motor_state[3].q,_motor_state[4].q,_motor_state[5].q,
                   _motor_state[6].q,_motor_state[7].q,_motor_state[8].q,
                   _motor_state[9].q,_motor_state[10].q,_motor_state[11].q;
        return jointpos;
}

Eigen::Matrix<double,12,1> LowState::getJointvel(){
        Eigen::Matrix<double,12,1> jointvel;
        jointvel<< _motor_state[0].dq,_motor_state[1].dq,_motor_state[2].dq,
                   _motor_state[3].dq,_motor_state[4].dq,_motor_state[5].dq,
                   _motor_state[6].dq,_motor_state[7].dq,_motor_state[8].dq,
                   _motor_state[9].dq,_motor_state[10].dq,_motor_state[11].dq;
        return jointvel;
}
