
#ifndef WANDERER_H
#define WANDERER_H

#include "ros/init.h"
#include "ros/ros.h"
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iterator>
#include <vector>
#include <numeric>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"

extern std::vector<float> laser_data;
extern bool isTurning;
extern double wheel_space;
extern double wheel_size;
extern bool laserReceived;
extern ros::ServiceClient diffDrive;
extern create_fundamentals::DiffDrive srv;
extern float LASER_FOV_DEGREES;

typedef enum {
    ROTATE_RIGHT,
    ROTATE_LEFT
} Direction;

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg);
std::vector<float> filterLaserReadings(const sensor_msgs::LaserScan::ConstPtr& msg);
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
double calculate_rotate_time(double speed1, double radian);
double calculate_drive_time(double speed1, double distance);
void rotate_radian(float radian, double speed, Direction dir);
void drive_distance(float distance, double speed);
bool wandererThreshold(float thresh);
void avoidObstacle(float thresh);

#endif // WANDERER_H
