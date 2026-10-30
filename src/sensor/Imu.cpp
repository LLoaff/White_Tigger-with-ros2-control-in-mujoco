#include"sensor/Imu.h"

#ifdef USE_SIM
Imu::Imu(mjModel *model, mjData *data):_model(model),_data(data){
    _imu_acc_id = mj_name2id(_model,mjOBJ_SENSOR,"imu_acc");
    _imu_gyro_id = mj_name2id(_model,mjOBJ_SENSOR,"imu_gyro");
    _imu_quat_id = mj_name2id(_model,mjOBJ_SENSOR,"imu_quat");
    if(_imu_acc_id == -1){
        std::cout<<"没有 找到imu_acc "<<std::endl;
    }
    if(_imu_gyro_id == -1){
        std::cout<<"没有 找到imu_gyro"<<std::endl;
    }
    if(_imu_quat_id == -1){
        std::cout<<"没有 找到imu_quat"<<std::endl;
    }
}
void Imu::Imu_Update(){
    if (_imu_acc_id == -1 || _imu_gyro_id == -1 || _imu_quat_id == -1) {
        return;
    }

    accelerometer[0] = _data->sensordata[_model->sensor_adr[_imu_acc_id] + 0];
    accelerometer[1] = _data->sensordata[_model->sensor_adr[_imu_acc_id] + 1];
    accelerometer[2] = _data->sensordata[_model->sensor_adr[_imu_acc_id] + 2];

    gyroscope[0] = _data->sensordata[_model->sensor_adr[_imu_gyro_id] + 0];
    gyroscope[1] = _data->sensordata[_model->sensor_adr[_imu_gyro_id] + 1];
    gyroscope[2] = _data->sensordata[_model->sensor_adr[_imu_gyro_id] + 2];

    quaternion[0] = _data->sensordata[_model->sensor_adr[_imu_quat_id] + 0];
    quaternion[1] = _data->sensordata[_model->sensor_adr[_imu_quat_id] + 1];
    quaternion[2] = _data->sensordata[_model->sensor_adr[_imu_quat_id] + 2];
    quaternion[3] = _data->sensordata[_model->sensor_adr[_imu_quat_id] + 3];

    counts_acc++;
    counts_gyr++;
    counts_acc%=122;
    counts_gyr%=122;
    // std::cout<<"acc: \n"<< GetAcc() <<std::endl;
    // std::cout<<"gyro: \n"<< GetGyro() <<std::endl;
    // std::cout<<"ruler: \n"<< getRoll()<<" "<< getPitch() <<" "<< getYaw() <<std::endl;
    // std::cout<<"quat: \n"<< GetQuat() <<std::endl;
}
#else
Imu::Imu(){
    _serial.init("/dev/ttyimu",
        921600 ,
        itas109::ParityNone,
        itas109::DataBits8,
        itas109::StopOne);
        _serial.close();
        _serial.open();
        
        if(!_serial.isOpen()){
            printf("imu serial open fail--------- \n");
            _serial.close();
        }

        recv_buffer.resize(44);// 有个不认识的 0x53
}
void Imu::Imu_Update(){
    int FrameState = 0;
    int Bytenum = 0;
    int CheckSum = 0;
    uint8_t ACCData[11];
    uint8_t GYROData[11];
    uint8_t QUATData[11];
    int len = _serial.getReadBufferUsedLen();
    if(len >=44){
        _serial.readData(recv_buffer.data(),44);
        for(uint8_t i=0;i<44;i++){
            if(FrameState ==0){
                if(recv_buffer.data()[i] == 0x55 && Bytenum==0){
                    CheckSum = recv_buffer.data()[i];
                    Bytenum = 1;
                    continue;
                }
                else if(recv_buffer.data()[i] == 0x51 && Bytenum==1){
                    CheckSum += recv_buffer.data()[i];
                    FrameState = 1;
                    Bytenum = 2;
                    continue;
                }
                else if(recv_buffer.data()[i] == 0x52 && Bytenum==1){
                    CheckSum += recv_buffer.data()[i];
                    FrameState = 2;
                    Bytenum = 2;
                    continue;
                }
                
                else if(recv_buffer.data()[i] == 0x59 && Bytenum==1){
                    CheckSum += recv_buffer.data()[i];
                    FrameState = 3;
                    Bytenum = 2;
                    continue;
                }
                else if(recv_buffer.data()[i] == 0x53 && Bytenum == 1){
                    CheckSum += recv_buffer.data()[i];
                    FrameState = 4;
                    Bytenum = 2;
                }
               
            }
            // acc 
            else if(FrameState ==1){
                if(Bytenum < 10){
                    ACCData[Bytenum-2] = recv_buffer.data()[i];
                    CheckSum += recv_buffer.data()[i];
                    Bytenum += 1;
                }
                else{
                    if(recv_buffer.data()[i] == (CheckSum & 0xff)){
                        accelerometer[0] = (float)((int16_t)((uint16_t)ACCData[1] << 8 | ACCData[0])) / 32768.0 * 16.0*9.81 - bias_accx;
                        accelerometer[1] = (float)((int16_t)((uint16_t)ACCData[3] << 8 | ACCData[2])) / 32768.0 * 16.0*9.81 - bias_accy;
                        accelerometer[2] = (float)((int16_t)((uint16_t)ACCData[5] << 8 | ACCData[4])) / 32768.0 * 16.0*9.81 - bias_accz;
                        counts_acc++;
                        counts_acc%=122;
                        // std::cout<<"accelerometerx: "<< accelerometer[0] <<std::endl;
                        // std::cout<<"accelerometery: "<< accelerometer[1] <<std::endl;
                        // std::cout<<"accelerometerz: "<< accelerometer[2] <<std::endl;
                    }
                    CheckSum = 0;
                    Bytenum = 0;
                    FrameState = 0;
                    
                }
            }
            // gyro 
            else if(FrameState ==2){
                if(Bytenum < 10){
                    GYROData[Bytenum-2] = recv_buffer.data()[i];
                    CheckSum += recv_buffer.data()[i];
                    Bytenum += 1;
                }
                else{
                    if(recv_buffer.data()[i] == (CheckSum & 0xff)){
                        gyroscope[0] = (float)(((int16_t)((uint16_t)GYROData[1] << 8 | GYROData[0]))) / 32768.0 * 2000.0/180.0*M_PI;
                        gyroscope[1] = (float)(((int16_t)((uint16_t)GYROData[3] << 8 | GYROData[2]))) / 32768.0 * 2000.0/180.0*M_PI;
                        gyroscope[2] = (float)(((int16_t)((uint16_t)GYROData[5] << 8 | GYROData[4]))) / 32768.0 * 2000.0/180.0*M_PI;
                        // std::cout << "GYROData: "
                        // << std::hex << std::uppercase << std::setfill('0')
                        // << std::setw(2) << static_cast<int>(GYROData[0]) << " "
                        // << std::setw(2) << static_cast<int>(GYROData[1]) << "  "
                        // << std::setw(2) << static_cast<int>(GYROData[2]) << " "
                        // << std::setw(2) << static_cast<int>(GYROData[3]) << "  "
                        // << std::setw(2) << static_cast<int>(GYROData[4]) << " "
                        // << std::setw(2) << static_cast<int>(GYROData[5])
                        // << std::dec << std::setfill(' ')  // 切回十进制，避免影响后续输出
                        // << std::endl;

                        // std::cout<<"gyroscopex: "<< gyroscope[0] <<std::endl;
                        // std::cout<<"gyroscopey: "<< gyroscope[1] <<std::endl;
                        // std::cout<<"gyroscopez: "<< gyroscope[2] <<std::endl;
                        counts_gyr++;
                        counts_gyr%=122;
                    }

                    CheckSum = 0;
                    Bytenum = 0;
                    FrameState = 0;
                    

                }
            }
            // quat 
            else if(FrameState ==3){
                if(Bytenum < 10){
                    QUATData[Bytenum-2] = recv_buffer.data()[i];
                    CheckSum += recv_buffer.data()[i];
                    Bytenum += 1;
                }
                else{
                    if(recv_buffer.data()[i] == (CheckSum & 0xff)){
                        quaternion[0] = (float)((int16_t)((uint16_t)QUATData[1] << 8 | QUATData[0])) / 32768.0;
                        quaternion[1] = (float)((int16_t)((uint16_t)QUATData[3] << 8 | QUATData[2])) / 32768.0;
                        quaternion[2] = (float)((int16_t)((uint16_t)QUATData[5] << 8 | QUATData[4])) / 32768.0;
                        quaternion[3] = (float)((int16_t)((uint16_t)QUATData[7] << 8 | QUATData[6])) / 32768.0;
                    }
                    CheckSum = 0;
                    Bytenum = 0;
                    FrameState = 0;
                }
            } 
            else if(FrameState == 4){
                if(Bytenum < 10){
                    CheckSum += recv_buffer.data()[i];
                    Bytenum += 1;
                }
                else{
                    CheckSum = 0;
                    Bytenum = 0;
                    FrameState = 0;
                }
            }
        }
       
    }   

    // std::cout<<"acc: \n"<< GetAcc() <<std::endl;
    // std::cout<<"gyro: \n"<< GetGyro() <<std::endl;
    // std::cout<<"ruler: \n"<< getRoll()<<" "<< getPitch() <<" "<< getYaw() <<std::endl;
}
#endif
void Imu::Imu_GetBias(){
    int counts = 120;
    float acc_get[3],gyr_get[3];
    while(counts_acc<=counts && counts_gyr<=counts){
        usleep(20 *1000);
        Imu_Update();
        acc_get[0]+=accelerometer[0];
        acc_get[1]+=accelerometer[1];
        acc_get[2]+=accelerometer[2];

        gyr_get[0]+=gyroscope[0];
        gyr_get[1]+=gyroscope[1];
        gyr_get[2]+=gyroscope[2];
        std::cout<<"cunts: "<< counts_acc <<std::endl;
        

    }
    bias_accx = acc_get[0] / counts;
    bias_accy = acc_get[1] / counts;
    bias_accz = acc_get[2] / counts -9.81;

    bias_gyrx = gyr_get[0] / counts;
    bias_gyry = gyr_get[1] / counts;
    bias_gyrz = gyr_get[2] / counts;

    std::cout<<"acc_bias: "<< bias_accx <<" "<< bias_accy <<" "<< bias_accz <<" "<<"\n"
             <<"gyr_bias: "<< bias_gyrx <<" "<< bias_gyry <<" "<< bias_gyrz <<" "<<std::endl;
}

void Imu::Imu_Initial(){
    this->quaternion[0] = 1;
    this->quaternion[1] = 0;
    this->quaternion[2] = 0;
    this->quaternion[3] = 0;
}

Eigen::Matrix<float,3,3> Imu::GetRotMat(){
    Eigen::Matrix<float, 4, 1> quat;
    quat << quaternion[0],quaternion[1],quaternion[2],quaternion[3];
    return Quat2RotMat(quat);
}

Eigen::Matrix<float,3,1> Imu::GetAcc(){
    Eigen::Matrix<float,3,1> a;
    a<< accelerometer[0],accelerometer[1],accelerometer[2];
    return a;
}

Eigen::Matrix<float,3,1> Imu::GetGyro(){
    Eigen::Matrix<float,3,1> gryo;
    gryo<< gyroscope[0],gyroscope[1],gyroscope[2];
    return gryo;
}

Eigen::Matrix<float,4,1> Imu::GetQuat(){
    Eigen::Matrix<float,4,1> q;
    q<< quaternion[0],quaternion[1],quaternion[2],quaternion[3];
    return q;
}
Eigen::Matrix<float,3,1> Imu::getAccGlobal(){
        return GetRotMat() * GetAcc();
}

Eigen::Matrix<float,3,1> Imu::getGyroGlobal(){
    return GetRotMat() * GetGyro();
}

float Imu::getYaw(){
    return rotMatToRPY(GetRotMat())(2);
}

float Imu::getDYaw(){
    return getGyroGlobal()(2);
}

float Imu::getRoll(){
    float roll=0;
    float w = quaternion[0];
    float x = quaternion[1];
    float y = quaternion[2];
    float z = quaternion[3];

    roll = atan2(2*(w*x + y*z),1-2*(x*x+y*y));
    return roll;
}
float Imu::getPitch(){
    float pitch=0;
    float w = quaternion[0];
    float x = quaternion[1];
    float y = quaternion[2];
    float z = quaternion[3];

    pitch = asin(2*(w*y-z*x)) ;
    return pitch;
}
Imu::~Imu(){
    // _serial.close();
}
