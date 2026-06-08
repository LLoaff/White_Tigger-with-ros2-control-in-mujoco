
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


