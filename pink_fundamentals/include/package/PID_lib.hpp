#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "ros/ros.h"
#include <cstdlib>
#include <cmath>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"


namespace PID_lib{

enum class Direction {
    ROTATE_RIGHT = 0,
    ROTATE_LEFT  = 1
};

constexpr double wheel_space = 0.265; // in m // 265
constexpr double wheel_size = 0.0620;   // in m
constexpr double encoder_tick = 6.283;
constexpr double Kp = 1.5;  //    0.35   Mahmoud 1.5
constexpr double Ki = 0.5; // 0.013 Mahmoud 0.5
constexpr double Kd = 0.00;  // Mahmoud 0.07

constexpr double Kp_left = Kp;
constexpr double Kp_right = Kp; // 0.65
constexpr double Kd_left = Kd;
constexpr double Kd_right = Kd;
constexpr double Ki_left = Ki;
constexpr double Ki_right = Ki;
constexpr double threshold = 0.5; // 0.5
constexpr double rotations_threshold = 0.1;
constexpr double  max_integral = 2;


}

#endif // ROBOT_CONTROLLER_H
