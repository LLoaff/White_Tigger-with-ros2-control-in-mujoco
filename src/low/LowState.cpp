
#include "low/LowState.h"

LowState::LowState():_motor_data{
        damiao::Motor(damiao::DM4310, 1, 0x11),  
        damiao::Motor(damiao::DM4310, 2, 0x12),  
        damiao::Motor(damiao::DM4310, 3, 0x13), 
        
        damiao::Motor(damiao::DM4310, 4, 0x14),  
        damiao::Motor(damiao::DM4310, 5, 0x15),  
        damiao::Motor(damiao::DM4310, 6, 0x16),  
        
        damiao::Motor(damiao::DM4310, 7, 0x17),  
        damiao::Motor(damiao::DM4310, 8, 0x18),  
        damiao::Motor(damiao::DM4310, 9, 0x19),  
        
        damiao::Motor(damiao::DM4310, 10, 0x1a), 
        damiao::Motor(damiao::DM4310, 11, 0x1b), 
        damiao::Motor(damiao::DM4310, 12, 0x1c) 
}{

}


