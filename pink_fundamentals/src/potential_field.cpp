#include "ros/console.h"
#include "ros/init.h"
#include "ros/ros.h"

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <boost/type_traits/alignment_of.hpp>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <iterator>
#include <vector>
#include <optional>
#include "create_fundamentals/SensorPacket.h"
#include "package/PID_lib.hpp"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/ResetEncoders.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include "pink_fundamentals/PID_drive.h"

/*Struct data structure*/


typedef struct
{
    double x, y;
} Point2D;


struct Line
{
    double a, b, c; // Line in form: ax + by + c = 0
    std::vector<Point2D> inliers;
};

std_msgs::ColorRGBA red_color, green_color, blue_color;

/*Service and Client*/
create_fundamentals::ResetEncoders srv_encoder;
create_fundamentals::DiffDrive srv;
ros::ServiceClient pid_rotate;
ros::ServiceClient pid_drive;
ros::ServiceClient diffDrive;
ros::ServiceClient resetEncoder;
ros::Publisher field_pub ;
ros::Publisher  radius_pub;

/*Data*/
std::vector<Line> lines;
std::vector<Line> closest_intersect_line;
std::vector<float> laser_data;
std::vector<Line> locked_lines; 
std::vector<Point2D> intersect_points;
std::vector<Point2D> cartesian;

bool isTurning = false;
bool laserReceived = false;
double angle_min = 0;
double angle_increment = 0;
bool isIntersect = false;
bool should_intersect = false; // for intersecting
bool align_lock = false;
/*Potential Field*/
double total_repel_x;
double total_repel_y;
double fx_attract;
double fy_attract;
double attract_gain = 100;
double repel_gain = 0.05;  // η = 0.01
double influence_radius = 0.5; // 0.5 meter
double fx_total;
double fy_total;

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

double current_distance_to_goal = 0;
double prev_distance_to_goal = 999;
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

double distanceToPoint(const Point2D &point){
    return std::sqrt(point.x * point.x + point.y * point.y);
}
void publishInfluenceRadius(ros::Publisher& pub, double radius) {
    visualization_msgs::Marker circle;
    circle.header.frame_id = "laser";  // Or "odom", depending on your setup
    circle.header.stamp = ros::Time::now();
    circle.ns = "influence_radius";
    circle.id = 0;
    circle.type = visualization_msgs::Marker::LINE_STRIP;
    circle.action = visualization_msgs::Marker::ADD;

    circle.pose.orientation.w = 1.0;
    circle.scale.x = 0.02;  // Line thickness

    circle.color.r = 0.0;
    circle.color.g = 0.0;
    circle.color.b = 1.0;
    circle.color.a = 1.0;

    const int segments = 100;  // More segments = smoother circle
    for (int i = 0; i <= segments; ++i) {
        double angle = 2 * M_PI * i / segments;
        geometry_msgs::Point p;
        p.x =  radius * cos(angle);
        p.y =  radius * sin(angle);
        p.z = 0;
        circle.points.push_back(p);
    }

    pub.publish(circle);
}
void publishVectorArrow(double fx, double fy, std::string ns, int id, ros::Publisher& pub, std_msgs::ColorRGBA color) {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "laser";  // or "odom" or "map" depending on your tf tree
    arrow.header.stamp = ros::Time::now();
    arrow.ns = ns;
    arrow.id = id;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point start, end;
    start.x = 0;
    start.y = 0;
    start.z = 0;

    end.x = fx;
    end.y = fy;
    end.z = 0.0;

    arrow.points.push_back(start);
    arrow.points.push_back(end);

    arrow.scale.x = 0.02;  // shaft diameter
    arrow.scale.y = 0.04;  // head diameter
    arrow.scale.z = 0.0;   // ignored for ARROW

    arrow.color = color;
    arrow.lifetime = ros::Duration(0.1);  // update fast enough
    pub.publish(arrow);
}
void calculatPID(double set_point_x, double set_point_y,double base_speed)
{

    double d_left = (current_encoderLeft - previous_encoderLeft) * PID_lib::wheel_size * 0.5;
    double d_right = (current_encoderRight - previous_encoderRight) * PID_lib::wheel_size * 0.5;
    
    double d_center = 0.5 * (d_left + d_right);
    double delta_theta = (d_left -  d_right) / PID_lib::wheel_space;

    current_time = ros::Time::now().toSec();

    delta_time = current_time - previous_time;
    
    current_theta = delta_theta + previous_theta ;
    current_pos_x =  previous_pos_x + d_center * cos(previous_theta + 0.5 * delta_theta) ;
    current_pos_y =  (previous_pos_y - d_center * sin(previous_theta + 0.5 * delta_theta)) ;

    ROS_INFO("Current Pos X: %f, Setpoint X: %f", current_pos_x, set_point_x);
    ROS_INFO("Current Pos Y: %f, Setpoint Y: %f", current_pos_y, set_point_y);
    // ROS_INFO("Delta d: %f", calculate_distance(set_point_x, set_point_y, current_pos_x, current_pos_y));
    // ROS_INFO("Current Theta : %f", current_theta);

    /*Potential Field*/ 
    double distance = 0;
    /*Repulsive force calculate*/
     for(int i = 0; i < cartesian.size(); i++){    
        distance = distanceToPoint(cartesian[i]);
        if (distance < influence_radius && distance > 0.005  ) {  // ignore near-zero noise
            double scale = (repel_gain * (1.0 / distance - 1.0 / influence_radius)) / (distance * distance);

            // Repel force is in the opposite direction of the obstacle
            double fx = scale * (-cartesian[i].x / distance);
            double fy = scale * (-cartesian[i].y / distance);

            total_repel_x += fx;
            total_repel_y += fy;
        }
    }

    /*Attractive force calculate*/
    fx_attract = attract_gain * (set_point_x - current_pos_x);
    fy_attract = attract_gain * (set_point_y - current_pos_y);
    /*For repulsive force only*/
    // fx_attract = 0;
    // fy_attract = 0;
    /*Total force calculate*/
    fx_total = fx_attract + total_repel_x;
    fy_total = fy_attract + total_repel_y;
    // ROS_INFO("fx_total: %f",fx_total);
    // ROS_INFO("fy_total: %f",fy_total);
    current_distance_to_goal = calculate_distance(set_point_x, set_point_y, current_pos_x, current_pos_y);
    /*Decrease repel gain when it is close to the target*/


    
    if(current_distance_to_goal < 0.3){
        repel_gain = 0.01;
        ROS_INFO("Repel Gain: ", repel_gain );
        influence_radius = 0.2;
    }
    ROS_INFO("prev_distance_to_goal: %f",prev_distance_to_goal);
    ROS_INFO("current_distance_to_goal: %f",current_distance_to_goal);

    publishInfluenceRadius(radius_pub,influence_radius);
    publishVectorArrow(fx_attract, fy_attract, "Attract", 1, field_pub, red_color);
    publishVectorArrow(total_repel_x, total_repel_y, "Repel", 1, field_pub, green_color);
    publishVectorArrow(fx_total, fy_total, "Total", 1, field_pub, blue_color);

    /*Now Calculate Error*/
    double heading = atan2(fy_total, fx_total); // Direction to move (based on goal + obstacles)
    // error_angular = heading + current_theta  ;
    error_angular = heading ;
    
    // error_angular = heading - current_theta   ;
    ROS_INFO("heading %f",heading * 180 / M_PI);
    ROS_INFO("current_theta %f",current_theta * 180 / M_PI);
    ROS_INFO("Error angular: %f",error_angular * 180 / M_PI);
    // Normalize angle error to [-π, π]
    error_angular = atan2(sin(error_angular), cos(error_angular));
    // ROS_INFO("Error norm angular: %f",error_angular * 180 / M_PI );
    
    // error_angular_dt =  (error_angular - prev_error_angular) / delta_time; 

    double omega_command = (1 * error_angular) + (0 * error_angular_dt);

    // Motor speeds
    motor_omega_left = base_speed - omega_command;
    motor_omega_right = base_speed + omega_command;

    // ROS_INFO("Omega Left: %f", motor_omega_left);
    // ROS_INFO("Omega Right: %f", motor_omega_right);
    

    // Save for next iteration
    previous_encoderLeft = current_encoderLeft;
    previous_encoderRight = current_encoderRight;
    prev_error_angular = error_angular;
    previous_pos_x = current_pos_x;
    previous_pos_y = current_pos_y;
   
    previous_theta = current_theta;
    previous_time = current_time;
    total_repel_x = 0;
    total_repel_y = 0;

}


void drive_PID(double target_x, double target_y,  double degree, double base_speed)
{
    /*New Point New Start*/
    resetEncoder.call(srv_encoder);

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

    ros::Rate wait_data(10);

    ROS_INFO("Drive PID is called");
    while (ros::ok())
    {
        ros::spinOnce();

        calculatPID(set_point_x,set_point_y,base_speed);

        /*Break Condition*/
        if ( current_distance_to_goal < 0.1 ) {
            ROS_INFO("Break of small distance");
            break;
        }
        // if (current_distance_to_goal > prev_distance_to_goal) {
        //     if(current_distance_to_goal - prev_distance_to_goal > 0.001){
        //         ROS_INFO("Break of error");
        //         break;

        //     }


        // }
        prev_distance_to_goal = current_distance_to_goal;
        // ROS_INFO("motor_speed_left: %f and motor_speed_left: %f\n",motor_speed_left,motor_speed_right);
        drive_motor(motor_omega_left, motor_omega_right);
        wait_data.sleep();
    }
    repel_gain = 10;
    prev_distance_to_goal = 999;
    ROS_INFO("Done");
    drive_motor(0, 0);
}

bool drive(pink_fundamentals::PID_drive::Request &req, pink_fundamentals::PID_drive::Response &res)
{
    drive_PID(req.x,req.y,req.degree,req.speed);
    res.success = true;

    return true ;
}


std::vector<float> filterLaserReadings(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    std::vector<float> readings;

    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
        float r = msg->ranges[i];

        if (!std::isnan(r) && !std::isinf(r) && r >= msg->range_min && r <= msg->range_max)
        {
            readings.push_back(r);
        }
        else
        {
              // Keep data point
           readings.push_back(-999);
        }
    }
    return readings;
}


std::vector<Point2D> polarToCartesian(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser_data = filterLaserReadings(msg);
    std::vector<Point2D> points;
    double start_angle = angle_min;
    for (size_t i = 0; i < laser_data.size(); ++i)
    {
        start_angle = start_angle + angle_increment; // Convert degrees to radians
        //Filter out data points
        if(laser_data[i] != -999){
          points.push_back({laser_data[i] * cos(start_angle), laser_data[i] * sin(start_angle)});
        }
    }
    return points;
}


void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr &msg)
{
    current_encoderLeft = msg->encoderLeft;
    current_encoderRight = msg->encoderRight;    
    encoder_received = true; // <-- Mark that we got data

    // ROS_INFO("left encoder: %f, right encoder: %f", current_encoderLeft, current_encoderRight);
}

/*Call back funciton*/
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    /*Set Angle increment global parameter*/
    angle_increment = msg->angle_increment;
    angle_min = msg->angle_min;
    /*Filter out nan and out of range values*/
    laserReceived = true;
    cartesian = polarToCartesian(msg);
}


int main(int argc, char **argv)
{
        ros::init(argc, argv, "PID_Server");
        ros::NodeHandle n;
        ros::Subscriber sub_rad = n.subscribe("sensor_packet", 1, sensorCallback);
        /*Service object                                            Service name*/
        ros::Subscriber sub_laser = n.subscribe("scan_filtered", 1, laserCallback);
        diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
        resetEncoder = n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoder");
        field_pub = n.advertise<visualization_msgs::Marker>("potential_field", 10);
        radius_pub = n.advertise<visualization_msgs::Marker>("influence_radius_marker", 1);
        ros::ServiceServer service_drive = n.advertiseService("Potential_drive_1", drive);

        // Call this every loop with current position

        /*Define Marker colors*/
        red_color.r = 1.0; red_color.a = 1.0;
        green_color.g = 1.0; green_color.a = 1.0;
        blue_color.b = 1.0; blue_color.a = 1.0;

        ROS_INFO("Start Now");
        ROS_INFO("Data Received");
        ros::Rate rate(50); // 50 Hz control loop
        ROS_INFO("Waiting for encoder data...");

        while (!encoder_received && ros::ok() & !laserReceived) {
                ros::spinOnce();
                ros::Duration(0.1).sleep();
         }
        ROS_INFO("Encoder data and Laser data received. Starting Potential drive.");

        ros::spin();

        return 0;
}






