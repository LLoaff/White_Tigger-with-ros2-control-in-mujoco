
#include "low/LowState.h"
#ifdef USE_SIM
LowState::LowState(mjModel *model, mjData *data):_model(model),_data(data),_imu(model,data){

}
#else
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
#endif


