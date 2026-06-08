#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "CSerialPort/SerialPort.h"
#include "eigen3/Eigen/Dense"

class imu : public rclcpp::Node
{
public:
    imu();
    void Imu_Update();
private:
    itas109::CSerialPort _serial;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    float quaternion[4];    // w, x, y, z
    float gyroscope[3];
    float accelerometer[3];
    uint8_t data_buff[42];

};

imu::imu():Node("imu_node"){
    _serial.init("/dev/ttyS4",
        1000000 ,
        itas109::ParityNone,
        itas109::DataBits8,
        itas109::StopOne);
        _serial.close();
        _serial.open();
        
        if(!_serial.isOpen()){
            RCLCPP_ERROR(this->get_logger(),"open fail--------- \n");
            _serial.close();
        }
    pub_ = this-> create_publisher<sensor_msgs::msg::Imu>("/imu_sensor_broadcaster/imu",1);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),  
        std::bind(&imu::Imu_Update, this)  
    );
    RCLCPP_INFO(this->get_logger(),"imu_node load successfully  !!!--------- \n");

}

void imu::Imu_Update(){

    uint8_t send_data=0x0a;
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->now();  
    msg.header.frame_id = "imu_link"; 
    // int counter=10;
    _serial.writeData(&send_data,1);
    // while(_serial.getReadBufferUsedLen()==0){
    //     counter--;
    //     if(counter == 0){
    //         break;
    //     }
    // }
    if(_serial.getReadBufferUsedLen()>0){
        _serial.readData(data_buff,42);
        if(data_buff[0]==0x0a && data_buff[41]==0x0b){
            memcpy(&quaternion[0], &data_buff[1], 4);
            memcpy(&quaternion[1], &data_buff[5], 4);
            memcpy(&quaternion[2], &data_buff[9], 4);
            memcpy(&quaternion[3], &data_buff[13], 4);

            memcpy(&accelerometer[0], &data_buff[17], 4);
            memcpy(&accelerometer[1], &data_buff[21], 4);
            memcpy(&accelerometer[2], &data_buff[25], 4);

            memcpy(&gyroscope[0], &data_buff[29], 4);
            memcpy(&gyroscope[1], &data_buff[33], 4);
            memcpy(&gyroscope[2], &data_buff[37], 4);

            msg.linear_acceleration.x = accelerometer[0];
            msg.linear_acceleration.y = accelerometer[1];
            msg.linear_acceleration.z = accelerometer[2];

            msg.angular_velocity.x = gyroscope[0];
            msg.angular_velocity.y= gyroscope[1];
            msg.angular_velocity.z= gyroscope[2];

            msg.orientation.w = quaternion[0];
            msg.orientation.x = quaternion[1];
            msg.orientation.y = quaternion[2];
            msg.orientation.z = quaternion[3];

            pub_->publish(msg);
        }
    }
}


int main(int argc,char** argv){

    rclcpp::init(argc,argv);

    auto node = std::make_shared<imu>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}