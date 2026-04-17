#ifndef ROBOT_HW_INTERFACE_HPP
#define ROBOT_HW_INTERFACE_HPP

#include <ros/ros.h>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <unistd.h> 
#include <iostream> 
#include <string.h>
#include <chrono>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ackermann_msgs/AckermannDrive.h>

#include "robot_control/VelocityData.h"
#include "robot_control/I2cData.h"
#include "robot_control/UARTData.h"


//hw params
#define HW_IF_UPDATE_FREQ 50 //100     // diferente do da work
#define HW_IF_TICK_PERIOD 1 / HW_IF_UPDATE_FREQ // diferente do da work
#define CMD_VEL_TIMEOUT_DEACELERATION_PERIOD 0.05
#define DECELERATION_RATE 0.5

class RobotHWInterface {
public:
    RobotHWInterface(ros::NodeHandle& nh); // Ajustado para receber NodeHandle por referência
    //void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void AckermannDriveCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg);
    void publishWheelSpeeds(); // Publicando velocidades do cmd_vel
    void encoderCallbackI2C(const robot_control::I2cData::ConstPtr& msg); // Callback para os dados do encoder
    void encoderCallbackUart(const robot_control::UARTData::ConstPtr& msg);
    void commandTimeoutCallback(const ros::TimerEvent&); // Callback para o timeout
    void updateWheelSpeedForDeceleration(); // Desaceleração
    void updateOdometry();

    int teste = 0;

private:
    ros::NodeHandle nh;

    // Hw Interface 
    ros::Publisher velocity_command_pub;
    //ros::Subscriber cmd_vel_sub;
    ros::Subscriber ack_drive_sub;
    ros::Subscriber imu_sub;

    // Odometria
    ros::Subscriber encoder_sub;
    ros::Publisher odom_pub;

    // Temporizador para o timeout de comandos
    ros::Timer command_timeout_; 

    // Variáveis membro para armazenar as velocidades lineares das rodas
    float left_wheel_speed = 0.0;
    float right_wheel_speed = 0.0;

    // Variáveis para armazenar as velocidades angulares das rodas
    float left_wheel_angular_speed= 0.0;
    float right_wheel_angular_speed = 0.0;

    // Parâmetros carregados do arquivo .yaml
    float wheel_separation_width;
    float wheel_separation_length;
    float wheel_radius;

    // Parâmetros para a odometria e intermediarios para calculo da velocidade
    double x;
    double y;
    double th;
    double timestamp;

    double x_old;
    double y_old;
    double th_old;
    double timestamp_old;

    bool delta;

    double vel_linear_x;
    double vel_linear_y;
    double vel_angular_z;

    double base_vel_linear;
    double base_vel_angular;

    double imu_yaw = 0.0;
    double imu_angular_vel_z = 0.0;
    double imu_initial_offset = 0.0;
    bool imu_initialized = false;

    // Angulo do servo
    double servo_angle;

    ros::Time current_time;

    tf::TransformBroadcaster odom_broadcaster;

};

#endif // ROBOT_HW_INTERFACE_HPP