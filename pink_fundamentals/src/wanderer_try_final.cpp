#include "ros/init.h"
#include "ros/ros.h"
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iterator>
#include <vector>
#include "create_fundamentals/DiffDrive.h"
#include "sensor_msgs/LaserScan.h"
#include <numeric>  // for std::accumulate
#include "pink_fundamentals/Wanderer.h"
// float laser_data = 0.0;
std::vector<float>laser_data;
bool isTurning = false;
//std::vector<float> laser_data;
double wheel_space = 265.00 ;// in mm // 250
double wheel_size = 62.00 ;// in mm
bool laserReceived = false;
bool isWandering = false;
bool hasStopped = false; 
ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive srv;
float LASER_FOV_DEGREES = 180.0; // eig 120 war gut
typedef enum
{
    ROTATE_RIGHT,
    ROTATE_LEFT
} Direction;



std::vector<float>filterLaserReadings(const sensor_msgs::LaserScan::ConstPtr& msg){
std::vector<float> readings;

  for (size_t i = 0 ; i <msg->ranges.size() ; i++) {
    float r = msg->ranges[i];

    if (!std::isnan(r) && !std::isinf(r) && r >= msg->range_min && r<= msg->range_max) {
      readings.push_back(r);
    }
    //ignore readings
    else
    {
        readings.push_back(9.99);
    }

  }
  return readings;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
     laser_data= filterLaserReadings(msg);
     laserReceived =true;
    //  ROS_INFO("Valid readings count: %zu", laser_data.size());
    
    // for (size_t i = 0; i < laser_data.size(); ++i) {
    //     ROS_INFO("laser_data[%zu] = %f", i, laser_data[i]);
    // }

}

double calculate_rotate_time(double speed1, double radian){
    double omega = (speed1*(wheel_size/2) )/(wheel_space/2);
    if (omega == 0) {
        ROS_WARN("Omega is zero — cannot compute rotation time.");
        return 0;
    }
    float time = radian/omega;
    return time;

}

double calculate_drive_time(double speed1, double distance){
    float time = distance / (speed1* wheel_size/2);
    return time;

}


void rotate_radian(float radian, double speed, Direction dir){
       // float time = 0;
    float time = calculate_rotate_time(speed, radian);
        if (!std::isfinite(time) || time <= 0.0 || time > 10.0) {
            ROS_ERROR("Invalid rotation time: %f — skipping sleep to prevent crash", time);
            return;
        }

        if(dir == ROTATE_RIGHT){

          ROS_INFO("Turn right");
          srv.request.left = speed;
          srv.request.right = -1 * speed;
          diffDrive.call(srv);
          
          time = calculate_rotate_time(speed,radian);
          ROS_INFO("time: %f", time) ;
          ros::Duration(time).sleep();
        }
        else{

          ROS_INFO("Turn Left");
          srv.request.left = -1 *  speed;
          srv.request.right = speed;
          diffDrive.call(srv);
          
          time = calculate_rotate_time(speed,radian);
          ROS_INFO("time: %f", time) ;
          ros::Duration(time).sleep();

        }
}

void drive_distance(float distance, double speed){
        float time = 0;
        ROS_INFO("Driving");
        srv.request.left = speed;
        srv.request.right = speed;
        diffDrive.call(srv);

        time = calculate_drive_time(speed, distance);
        ROS_INFO("time: %f", time) ;
        ros::Duration(time).sleep();
}
/*
bool isPathClearLeft(const sensor_msgs::LaserScan::ConstPtr& scan, float safe_distance = 0.4, int window_size = 20)
{
    int left_start = scan->ranges.size() / 2;
    int left_end = scan->ranges.size();

    for (int i = left_start; i < left_end - window_size; ++i)
    {
        bool window_clear = true;
        for (int j = 0; j < window_size; ++j)
        {
            if (scan->ranges[i + j] < safe_distance)
            {
                window_clear = false;
                break;
            }
        }

        if (window_clear)
        {
            return true;  // Found a safe path
        }
    }

    return false;  // No clear path found
}
*/

bool wandererThreshold(float thresh){
    ros::spinOnce();
    if (!laserReceived) {
        ROS_WARN("No laser data received yet...");
        return false;
    }

    if (laser_data.empty()) return false;

    float min_range = *std::min_element(laser_data.begin(), laser_data.end());
    ROS_INFO("Min range: %f", min_range);
    return min_range <= thresh;
} 

void avoidObstacle(float thresh){
    ros::spinOnce();
    ros::Duration(0.05).sleep();  
    while(!isTurning && wandererThreshold(thresh)){
        isTurning = true;
        srv.request.left = 0;
        srv.request.right = 0;
        diffDrive.call(srv);
        size_t mid = laser_data.size() / 2;

        double sum1 = std::accumulate(laser_data.begin(), laser_data.begin() + mid, 0.0); //left half
        double avg1 = sum1 / mid;

        double sum2 = std::accumulate(laser_data.begin() + mid, laser_data.end(), 0.0);//right half
        double avg2 = sum2 / (laser_data.size() - mid);

        if (sum1 >= sum2){
            rotate_radian(3.14 /3, 5, ROTATE_RIGHT);
        }
        else{
            rotate_radian(3.14 / 3, 5, ROTATE_LEFT);
        }
        isTurning = false;
    }
}

bool wandererCall(pink_fundamentals::Wanderer::Request &req, pink_fundamentals::Wanderer::Response &res)
{

    isWandering = req.isWander;
    res.success = true;

    return true ;
}

void timerWandererCallback(const ros::TimerEvent&)
{   
    if (isWandering)
    {
        ros::spinOnce();
        hasStopped = false;
        if (!laserReceived) {
        ROS_WARN("No laser data received yet...");
        }
    
        ROS_INFO("Laser data size: %ld", laser_data.size());
        if (!laser_data.empty())
        {
            if (!wandererThreshold(0.20)) {
                ROS_INFO("Clear path, moving forward...");
                srv.request.left  = 8;
                srv.request.right = 8 ;
                diffDrive.call(srv);
            }
            else {
            avoidObstacle(0.20);
            }  
        }
        else
        {
            ROS_WARN("No valid laser data!");
        }
    }
    else
    {
        if (!hasStopped) {
            srv.request.left = 0;
            srv.request.right = 0;
            diffDrive.call(srv);  // Send stop command only once
            hasStopped = true;
            ROS_INFO("Wanderer stopped.");
        }
    }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "wanderer");
  ros::NodeHandle n;
  ros::Subscriber sub_laser = n.subscribe("scan_filtered", 1, laserCallback);
  diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

  ros::ServiceServer service_wanderer = n.advertiseService("Wanderer", wandererCall);
  ros::Timer timer = n.createTimer(ros::Duration(0.1), timerWandererCallback); // Routine


  ros::spin();
  return 0;
}


// float min_range = *std::min_element(laser_data.begin(), laser_data.end());
        //
        // ROS_INFO("Minimum detected range: %f", min_range);
        // size_t mid = laser_data.size() / 2;
        //
        // // If too close, look for alternative direction
        // if (min_range < 0.2 && !isTurning)
        // {
        //   isTurning = true;
        //     ROS_INFO("Driving stops to think");
        //     srv.request.left = 0;
        //     srv.request.right = 0;
        //     diffDrive.call(srv);
        //
        //   // Search for next available open window in laser_data
        //   const float SAFE_DISTANCE = 0.18; // Why was it 0.4?
        //   const int WINDOW_SIZE = 20;
        //   bool found = false;
        //
        //   for (size_t i = 0; i < laser_data.size() - WINDOW_SIZE; i += WINDOW_SIZE)
        //   {
        //       if (laser_data[i] > 0.18)
        //       {
        //           float angle_per_index = (LASER_FOV_DEGREES * M_PI / 180.0) / laser_data.size();
        //           float angle = (i - mid) * angle_per_index;
        //           if (std::fabs(angle) > M_PI) {
        //               ROS_WARN("Computed angle too large: %f rad — skipping", angle);
        //               continue;
        //           }
        //           ROS_INFO("Turning to index %zu, angle: %f rad", i, angle);
        //           if (std::abs(angle) < 0.01) {
        //               ROS_INFO("Angle too small, skippp rotation");
        //           }
        //             else {
        //               if (angle < 0) {
        //                   rotate_radian(std::fabs(angle), 5, ROTATE_LEFT);
        //                found = true;
        //                 break;
        //                }
        //               else {
        //                   rotate_radian(angle, 5, ROTATE_RIGHT);
        //
        //               found = true;
        //               break;
        //                 }
        //             }
        //       }
        //   }
        //
        //   // If no path found, rotate in place as fallback
        //   if (!found)
        //   {
        //       ROS_WARN("No clear path found! Rotating right as fallback.");
        //       rotate_radian(3.14 / 6, 5, ROTATE_RIGHT);
        //   }
        //
        //   isTurning = false;
        // }
        // else
        // {
        //     ROS_INFO("Clear path, moving forward...");
        //     srv.request.left = 10;
        //     srv.request.right = 10;
        //     diffDrive.call(srv);
        // }