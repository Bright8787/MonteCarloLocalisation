#include "ros/ros.h"
#include "wanderer.h"
#include <cstdlib>
#include <cmath>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/ResetEncoders.h"
#include "create_fundamentals/SensorPacket.h"
#include <boost/type_traits/alignment_of.hpp>
#include "sensor_msgs/LaserScan.h"

#include "package/PID_lib.hpp"

#include "pink_fundamentals/PID_drive.h"



typedef struct
{
    double x, y;
} Point2D;

create_fundamentals::DiffDrive srv;
create_fundamentals::ResetEncoders srv_encoder;

ros::ServiceClient diffDrive;
ros::ServiceClient resetEncoder;

double current_encoderLeft = 0;
double current_encoderRight = 0;

double previous_encoderLeft = 0;
double previous_encoderRight = 0;

bool encoder_received = false; // <- This is the key

/*Time parameter*/
double previous_time = 0;
double current_time = 0;
double delta_time = 0;

double motor_omega_right = 0;
double motor_omega_left = 0;
double error_angular = 0;
double error_angular_dt = 0;
/*Position and Orientation*/
double prev_error_angular = 0;
double current_pos_x = 0;
double current_pos_y = 0;
double previous_pos_x = 0;
double previous_pos_y = 0;
double current_theta = 0;
double previous_theta = 0;

double current_theta_m;
double previous_theta_m;

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr &msg)
{
    current_encoderLeft = msg->encoderLeft;
    current_encoderRight = msg->encoderRight;    
    encoder_received = true; // <-- Mark that we got data

    // ROS_INFO("left encoder: %f, right encoder: %f", current_encoderLeft, current_encoderRight);
}

void drive_motor(double speed_left, double speed_right)
{
    if (fabs(speed_left) > 100){
        if(speed_left < 0) {
            speed_left = -100;
        }
        else{
            speed_left = 100;
        }

    }
    
    if (fabs(speed_right) > 100){
        if(speed_right < 0) {
            speed_right = -100;
        }
        else{
            speed_right = 100; 
        }
    }
    
    srv.request.left = speed_left;
    srv.request.right = speed_right;
    diffDrive.call(srv);
}

double calculate_distance(double x1, double y1, double x2, double y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy);
}
void calculatPID(double set_point_x, double set_point_y,double base_speed)
{

    double d_left = (current_encoderLeft - previous_encoderLeft) * PID_lib::wheel_size * 0.5;
    double d_right = (current_encoderRight - previous_encoderRight) * PID_lib::wheel_size * 0.5;
    
    double d_center = 0.5 * (d_left + d_right);
    double delta_theta = (d_left - d_right) / PID_lib::wheel_space;

    current_time = ros::Time::now().toSec();

    delta_time = current_time - previous_time;
    
    current_theta = delta_theta + previous_theta ;
    current_pos_x =  previous_pos_x + d_center * cos(previous_theta + 0.5 * delta_theta) ;
    current_pos_y =  previous_pos_y + d_center * sin(previous_theta + 0.5 * delta_theta) ;

    ROS_INFO("Current Pos X: %f, Setpoint X: %f", current_pos_x, set_point_x);
    ROS_INFO("Current Pos Y: %f, Setpoint Y: %f", current_pos_y, set_point_y);
    ROS_INFO("Delta d: %f", calculate_distance(set_point_x, set_point_y, current_pos_x, current_pos_y));

    ROS_INFO("Current Theta : %f", current_theta);
    // Angular error calculation
    current_theta_m = std::atan2(set_point_y - current_pos_y, set_point_x - current_pos_x );

    error_angular =  current_theta_m - current_theta ;
    // Normalize angle error to [-π, π]
    error_angular = atan2(sin(error_angular), cos(error_angular));
    
    error_angular_dt =  (error_angular - prev_error_angular) / delta_time; 

    double omega_command = (8 * error_angular) + (0 * error_angular_dt);

    // Motor speeds
    motor_omega_left = base_speed + omega_command;
    motor_omega_right = base_speed - omega_command;


    // Save for next iteration
    previous_encoderLeft = current_encoderLeft;
    previous_encoderRight = current_encoderRight;
    prev_error_angular = error_angular;
    previous_pos_x = current_pos_x;
    previous_pos_y = current_pos_y;

    previous_theta = current_theta;
    previous_time = current_time;

}

void calculatPID_Orientation( double set_point_theta)
{

    current_time = ros::Time::now().toSec();
    double d_left = (current_encoderLeft - previous_encoderLeft) *  PID_lib::wheel_size * 0.5;
    double d_right = (current_encoderRight - previous_encoderRight) * PID_lib::wheel_size * 0.5;
    double delta_theta = (d_left - d_right ) / PID_lib::wheel_space;
    current_theta = delta_theta + previous_theta;
    error_angular =  (set_point_theta - current_theta) ;
    // error_angular = atan2(sin(error_angular), cos(error_angular));
    // Normalize angle error to [-π, π]    
    //  error_angular = atan2(sin(error_angular), cos(error_angular)) ;
    //  ROS_INFO("error_angular Normalize : %f", error_angular * 180 / M_PI);
    // Proportional control (add D or I terms if needed)
    // Normalize angle error to [-π, π]
    error_angular = atan2(sin(error_angular), cos(error_angular));
    ROS_INFO("error_angular  : %f", error_angular * 180 / M_PI);
    
    delta_time = current_time - previous_time;
    error_angular_dt =  (error_angular - prev_error_angular) / delta_time; 
    // error_angular_dt = 0;
    double omega_command = (8 * error_angular) + (0 * error_angular_dt);
    // ROS_INFO("d_left  : %f, d_right  : %f",d_left,d_right );
    
    ROS_INFO("Delta Theta : %f", delta_theta * 180 / M_PI);
    ROS_INFO("Current Theta : %f, Previous Theta : %f", 180 / M_PI * current_theta , 180 / M_PI * previous_theta );
    ROS_INFO("error_angular: %f, Setpoint Theta: %f", 180 / M_PI * error_angular , 180 / M_PI * set_point_theta);
    
    // Motor speeds
    motor_omega_left  =  omega_command;
    motor_omega_right =  -1*omega_command;
    //  ROS_INFO("omega_command: %f,", omega_command);

    // Save for next iteration
    previous_encoderLeft = current_encoderLeft;
    previous_encoderRight = current_encoderRight;
    prev_error_angular = error_angular;

    previous_theta = current_theta;
    previous_time = current_time;

}

double wrapTo2Pi(double angle_rad) {
    const double TWO_PI = 2.0 * M_PI;
    double wrapped = std::fmod(angle_rad, TWO_PI);
    if (wrapped < 0)
        wrapped += TWO_PI;
    return wrapped;
}
void drive_PID(double target_x, double target_y,  double degree, double base_speed)
{
    /*New Point New Start*/
    previous_time = 0;
    current_time = 0;
    delta_time = 0;

    current_pos_x = 0;
    current_pos_y = 0;
    current_theta = 0;
    current_theta_m = 0;

    previous_pos_x = 0;
    previous_pos_y = 0;
    previous_theta = 0;
    previous_theta_m = 0;    

    previous_encoderLeft = current_encoderLeft;
    previous_encoderRight = current_encoderRight;

    double set_point_x = target_x;
    double set_point_y = target_y;
    double set_point_theta =  degree;
    double set_point_theta_first = atan2(target_y,target_x) ;
    
    ROS_INFO("First Theta set point %f", set_point_theta_first * 180 / M_PI);
    
    ros::Rate wait_data(10);
    
    ROS_INFO("Drive PID is called");

    while (ros::ok())
    {
        ros::spinOnce();
                
        calculatPID_Orientation(set_point_theta_first);
                
        if (fabs(error_angular) <  0.05) {
            ROS_INFO("Threshhold Done");
            break;
        }
        // ROS_INFO("motor_speed_left: %f and motor_speed_left: %f\n",motor_speed_left,motor_speed_right);
        drive_motor(motor_omega_left, motor_omega_right);
        wait_data.sleep();
    }

    while (ros::ok())
    {
        ros::spinOnce();
        
        calculatPID(set_point_x,set_point_y,base_speed);


        double dist_err = calculate_distance(set_point_x, set_point_y, current_pos_x, current_pos_y);
        double theta_err = fabs(error_angular);  // already in radians

        if (dist_err < 0.05 ) {
            break;
        }
        // ROS_INFO("motor_speed_left: %f and motor_speed_left: %f\n",motor_speed_left,motor_speed_right);
        drive_motor(motor_omega_left, motor_omega_right);
        wait_data.sleep();
    }

        /*Correction to x = 0*/
    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //
    //     calculatPID_Orientation(set_point_theta * M_PI / 180);
    //
    //     if (fabs(error_angular) <  0.05) {
    //         ROS_INFO("Threshhold Done");
    //         break;
    //     }
    //     // ROS_INFO("motor_speed_left: %f and motor_speed_left: %f\n",motor_speed_left,motor_speed_right);
    //     drive_motor(motor_omega_left, motor_omega_right);
    //     wait_data.sleep();
    // }
    //
    ROS_INFO("Done");
    drive_motor(0, 0);
}

bool drive(pink_fundamentals::PID_drive::Request &req, pink_fundamentals::PID_drive::Response &res)
{
    drive_PID(req.x,req.y,req.degree,req.speed);
    res.success = true;

    return true ;
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "PID_Server");
        ros::NodeHandle n;
        ros::Subscriber sub_rad = n.subscribe("sensor_packet", 1, sensorCallback);
        diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
        resetEncoder = n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");
        ROS_INFO("Start Now");
        ROS_INFO("Data Received");
        ros::Rate rate(50); // 50 Hz control loop
        /*Service object                                            Service name*/
        ros::ServiceServer service_drive = n.advertiseService("PID_drive", drive);
        ROS_INFO("Waiting for encoder data...");

        while (!encoder_received && ros::ok()) {
                ros::spinOnce();
                ros::Duration(0.1).sleep();
         }
        ROS_INFO("Encoder data received. Starting PID.");

 
        
        

        ros::spin();

        return 0;
}


