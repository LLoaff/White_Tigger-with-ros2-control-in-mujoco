
#include "low/LowState.h"

LowState::LowState(mjModel *model, mjData *data):_model(model),_data(data),_imu(model,data){

}


