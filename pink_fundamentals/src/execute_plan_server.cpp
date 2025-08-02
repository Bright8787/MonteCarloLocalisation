#include "ros/console.h"
#include "ros/init.h"
#include "ros/ros.h"
#include "wanderer.h"

#include <boost/type_traits/alignment_of.hpp>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iterator>
#include <vector>
#include <optional>
#include <std_msgs/Empty.h>
#include "package/PID_lib.hpp"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "create_fundamentals/PlaySong.h"
#include "create_fundamentals/ResetEncoders.h"

#include "pink_fundamentals/align_call.h"
#include "pink_fundamentals/PID_drive.h"
#include "pink_fundamentals/ExecutePlan.h"
#include "pink_fundamentals/Pose.h"
#include "pink_fundamentals/ExactPose.h"
#include <pink_fundamentals/Grid.h>




/*Struct data structure*/
typedef struct
{
    double x, y;
} Point2D;

struct TargetVector {
    double dx;
    double dy;
    double distance;
    double target_x;
    double target_y;
};


/*Data*/
double x_best = 0.0;
double y_best = 0.0;
double theta_best = 0.0;
double theta_from_quaternion = 0.0;
int cell_row = 0;
int cell_col = 0;
int cell_dir = 0; 
bool safe_shutdown = false;
bool align_lock = false;
bool plan_status = true;
bool use_PID;
bool use_AVF;
bool aligned_after_plam = true;
bool laserReceived = false;
bool cell_pos_received = false;
bool exact_pos_received = false;

create_fundamentals::DiffDrive srv;
create_fundamentals::PlaySong  srv_song;
create_fundamentals::ResetEncoders srv_encoder;
pink_fundamentals::align_call srv_align;
pink_fundamentals::PID_drive srv_pid_drive;
pink_fundamentals::PID_drive apf_srv;


ros::ServiceClient pid_drive;
ros::ServiceClient align_server;
ros::ServiceClient diffDrive;
ros::ServiceClient resetEncoder;
ros::ServiceClient play_song;
std::vector<float> laser_data;

ros::Publisher line_array_pub;
ros::Publisher intersect_point_pub;
ros::Publisher perpendicular_line_array_pub;
ros::Publisher exact_pose_pub;

// Eigen::MatrixXf particles;

double calculate_drive_time(double speed1, double distance)
{
    float time = 1000 *distance / (speed1 * 62 / 2);
    return fabs(time);
}

double calculate_rotate_time(double speed1, double radian){
    double omega = (speed1*(PID_lib::wheel_size/2) )/(PID_lib::wheel_space/2);
    double time = radian/omega;
    return fabs(time);
}

void rotate_degree(double degree, double speed, PID_lib::Direction dir)
{
    double radian = M_PI * degree / 180.00;
    double time = 0;
    if (dir == PID_lib::Direction::ROTATE_RIGHT)
    {

        ROS_INFO("Turn right");
        srv.request.left = speed;
        srv.request.right = -1 * speed;
        diffDrive.call(srv);

        time = calculate_rotate_time(speed, radian);
        ROS_INFO("time: %f", time);
        ros::Duration(time).sleep();
    }
    else
    {

        ROS_INFO("Turn Left");
        srv.request.left = -1 * speed;
        srv.request.right = speed;
        diffDrive.call(srv);

        time = calculate_rotate_time(speed, radian);
        ROS_INFO("time: %f", time);
        ros::Duration(time).sleep();
    }
     srv.request.left = 0;
    srv.request.right = 0;
    diffDrive.call(srv);
}

/* Callbacks */

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laserReceived = true;
    laser_data = filterLaserReadings(msg);
}

// // well see if its necessary...
// void publishBestPosCallback(const std_msgs::Empty::ConstPtr& msg) {
//     ROS_INFO("Calling publishExactPose with: x = %.3f, y = %.3f, theta = %.3f°", x_best, y_best, theta_best * 180.0 / M_PI);
//     pink_fundamentals::ExactPose pose_msg;
//     pose_msg.x = x_best;
//     pose_msg.y = y_best;
//     pose_msg.theta = theta_best;

//     exact_pose_pub.publish(pose_msg);
// }
// to use the best weighted particles position as the robots position to calculate
void exactPoseCallback(const pink_fundamentals::ExactPose::ConstPtr& msg) {
    exact_pos_received = true;
    x_best = msg->x;
    y_best = msg->y;
    theta_best = msg->theta;
    double theta = 2 * atan2(theta_best, x_best);
    // theta_from_quaternion = tf::getYaw(msg.orientation);
    ROS_INFO("Exact pose received: x=%f y=%f theta=%f", x_best, y_best, theta);
}
//  also dk if this is necessary
// void publishGridCellCallback(const std_msgs::Empty::ConstPtr& msg) {
//         ROS_INFO("Publishing robot cell: row = %d, col = %d, orientation = %d (%.1f°)", row, col, dir, angle_in_2PI);
// }
// but this is:
void getCellPos(const pink_fundamentals::Pose::ConstPtr& msg) {
    cell_pos_received = true;
    cell_dir = msg->orientation;
    cell_row = msg->row;
    cell_col = msg->column;
    // ROS_INFO("Cell Pose received: dir: %d, cell(%d, %d)", cell_dir, cell_row, cell_col);
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
       //     readings.push_back(-999);
        }
    }
    return readings;
}
// void checkObstacle(){
//     double total_time = calculate_drive_time(speed, distance);
//     ros::Time start = ros::Time::now();
//     ros::Rate  r(20);             // 20 Hz loop
  
//     srv.request.left  = speed;
//     srv.request.right = speed;
//     diffDrive.call(srv);
  
//     // loop until time is up or threshold trips
//     while (ros::ok() && (ros::Time::now() - start).toSec() < total_time) {
//       ros::spinOnce();                   // get fresh laser data
//       if (wandererThreshold(0.16)) {
//         ROS_ERROR("drive_distance ABORT: obstacle within 0.15m");
//         srv.request.left  = 0;
//         srv.request.right = 0;
//         diffDrive.call(srv);
        
        
//         srv_song.request.number = 1;
//         play_song.call(srv_song);
//         plan_status = false;
//         break;
//       }
//       r.sleep();
//     }


// }
void drive(double x, double y, double degree, double speed){

    srv_pid_drive.request.x = x;
    srv_pid_drive.request.y = y;
    srv_pid_drive.request.degree = degree;
    srv_pid_drive.request.speed = speed;
    
    pid_drive.call(srv_pid_drive);

}

void avf(double x, double y, double degree, double speed){

    apf_srv.request.x      = x;
    apf_srv.request.y      = y;
    apf_srv.request.degree = degree;
    apf_srv.request.speed  = speed;
    pid_drive.call(apf_srv);

}


double wrapTo2Pi(double angle_rad) {
    const double TWO_PI = 2.0 * M_PI;
    double wrapped = std::fmod(angle_rad, TWO_PI);
    if (wrapped < 0)
        wrapped += TWO_PI;
    return wrapped;
}
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double directionToAngle(int dir) {
    switch (dir) {
        case 0: return 0.0;              // R 0
        case 1: return -M_PI / 2.0;       // Up -90
        case 2: return M_PI;             // L 180
        case 3: return M_PI / 2.0;      // Down 90
        default: return 0.0;             // no
    }
}

double rotate_to_dir(int dir, double cur_theta) {
    ROS_INFO("dir: %d", dir);
    double target_theta = directionToAngle(dir);
    double rotation_needed = normalizeAngle(target_theta - cur_theta);
    ROS_INFO("offset für theta: %f, targettheta: %f , curTheta: %f", rotation_needed, target_theta, cur_theta);

    return rotation_needed;

}



/* Geometry calculating positions */

TargetVector getVectorToNextCell(double x, double y, int direction, double cell_size = 0.8) {
    // check if the robot starts from the center of the cell 
    double multiple_check = 0.4;
    auto isMultiple = [multiple_check](double val) {
        return std::fmod(val, multiple_check) < 1e-4 || std::fmod(val, multiple_check) > multiple_check - 1e-4;
    };
    if (!isMultiple(x) || !isMultiple(y)) {
        double corrected_x = std::round(x / cell_size) * cell_size;
        double corrected_y = std::round(y / cell_size) * cell_size;

        double dx = corrected_x - x;
        double dy = corrected_y - y;
        double dist = std::sqrt(dx * dx + dy * dy);

        ROS_WARN("Robot not aligned to cell grid, correcting to (%.3f, %.3f)", corrected_x, corrected_y);

        return {.dx=dx, .dy=dy, .distance=dist, .target_x=corrected_x, .target_y=corrected_y};
    }
    
    double x_target = x; // initialize as the current position
    double y_target = y;
    // Move one cell in the given direction
    switch (direction) {
        case 0: {
            x_target += 0.8;
            break; // RIGHT (x+1)
        }
        case 1: {
            y_target -= 0.8;
            break; // UP (y-1)
        }
        case 2: {
            x_target -= 0.8;
            break; // LEFT (x-1)
        }
        case 3: {
            y_target += 0.8;
            break; // DOWN (y+1)
        }
    }

    // Vector from robot to cell center
    double dx = x_target - x;
    double dy = y_target - y;
    double dist = std::sqrt(dx * dx + dy * dy);

    // check if robot is middle of cell -> I try recursive...
    ROS_INFO("dist: %f, cell(%d, %d)", dist, cell_row, cell_col);
    return {.dx=dx, .dy=dy, .distance=dist, .target_x=x_target, .target_y=y_target}; // only dx and dy necessary?
}

float getAverageMiddleThird(const std::vector<float>& laser_data) {

    size_t total_size = laser_data.size();
    size_t start_index = total_size / 3;
    size_t end_index = 2 * total_size / 3;

    float sum = 0.0;
    int count = 0;

    for (size_t i = start_index; i < end_index; ++i) {
        float range = laser_data[i];
        // Skip invalid readings (NaN or Inf)
            sum += range;
            ++count;
    
    }

    if (count == 0) return std::numeric_limits<float>::infinity();  // or some default
    return sum / count;
}

bool wandererThreshold1(float thresh){

    if (laser_data.empty()) return false;
    // ros::spinOnce();  
    // float min_range = *std::min_element(laser_data.begin()*1/3, laser_data.end()*2/3);
    // float min_range = getAverageMiddleThird(laser_data);
    // float min_range = *std::min_element(laser_data.begin(), laser_data.end());
    float min_range = laser_data.size()/2;
    // float min_range =laser_data[laser_data.size()/2];
    // ROS_INFO("Min range: %f", min_range);
    return min_range <= thresh;
} 

bool drive_distance(double distance, double speed) {

    double total_time = calculate_drive_time(speed, distance);
    ros::Time start = ros::Time::now();
    ros::Rate  r(10);             // 10 Hz loop
    
    // if (wandererThreshold1(0.15 )) {
        
    //     ROS_ERROR("drive_distance ABORT: obstacle within 0.15m");
    //     srv.request.left  = 0;
    //     srv.request.right = 0;
    //     diffDrive.call(srv);
        
        
    //     srv_song.request.number = 1;
    //     play_song.call(srv_song);
    //     plan_status = false;
    //     return false;
    // }
    srv.request.left  = speed;
    srv.request.right = speed;
    diffDrive.call(srv);

    while (ros::ok() && (ros::Time::now() - start).toSec() < total_time) {
        ros::spinOnce();                   // get fresh laser data
        if (wandererThreshold1(0.2)) {// vorher wars 0.2
          ROS_ERROR("drive_distance ABORT: obstacle within 0.15m");
          srv.request.left  = 0;
          srv.request.right = 0;
          diffDrive.call(srv);
          
          srv_song.request.number = 1;
          play_song.call(srv_song);
          plan_status = false;
          break;
        }
        r.sleep();
    }   

    ROS_INFO("Finish driving");
    //   // loop until time is up or threshold trips
    //   while (ros::ok() && (ros::Time::now() - start).toSec() < total_time) {
    //     ros::spinOnce();                   // get fresh laser data
    //     if (wandererThreshold1(0.15 )) {
    //       ROS_ERROR("drive_distance ABORT: obstacle within 0.15m");
    //       srv.request.left  = 0;
    //       srv.request.right = 0;
    //       diffDrive.call(srv);
        
        
    //       srv_song.request.number = 1;
    //       play_song.call(srv_song);
    //       plan_status = false;
    //       break;
    //     }
    //     r.sleep();
    //   }
    ROS_INFO("Stop");
    srv.request.left  = 0;
    srv.request.right = 0;
    diffDrive.call(srv);

  // return false if we aborted early
  return true;
}

/* Aligning */

bool doAlignment() {

    if(!align_server.call(srv_align)) return false;
    return true;
}


bool executePlan(std::vector<int> plan) {

    bool use_PID = false;
    use_AVF = true;
    ros::param::get("/use_PID", use_PID);

    /*Do the Alignment first*/
    // double wx = x_best;
    // double wy = y_best;
    int cur_dir = cell_dir; // from cellPosCallback
    /* To use without monte carlo */
    // int cur_dir = 3; // UP
    // theta_best = 155;
    // cell_col = 1;
    // cell_row = 2;
    // x_best = 1.9;
    // y_best = 1;
    
    ROS_INFO("use_PID: %d", use_PID);
    ROS_INFO("use_AVF: %d", use_AVF);

    if (use_AVF) {
        ROS_INFO("In afv execution");
        for (size_t i = 0; i < plan.size(); i++) {
            int next_dir = plan[i];
            ROS_INFO("next_dir: %d", next_dir);
            TargetVector vec = getVectorToNextCell(x_best, y_best, next_dir);
            int forward_speed = 6;
            int diff = next_dir - cur_dir;
            // cur_dir = next_dir; // Packe ich ans ende der schleife
            double winkel = theta_best;
            theta_best = atan2(sin(winkel),cos(winkel));
            double dist_x_local = cos(theta_best) * vec.dx + sin(theta_best) * vec.dy;
            double dist_y_local = -sin(theta_best) * vec.dx + cos(theta_best) * vec.dy;
            double dist_local = std::sqrt(dist_x_local * dist_x_local + dist_y_local * dist_y_local);
            ROS_INFO("Theta best: %f", theta_best);
            double angleMinusOffset = (rotate_to_dir(next_dir, theta_best))*180/M_PI; // umgerechnet von radian in degree
            // double angle_correction = angleMinusOffset;
            ROS_INFO("angle offset: %f", angleMinusOffset);

            ROS_INFO("diff: %d", diff);
            if (diff == -2) { // von unten nach oben
                // rotate_degree(180, 4, PID_lib::Direction::ROTATE_RIGHT);
                // drive_distance(dist_local, forward_speed); // 0.8
                avf(-0.8, 0, angleMinusOffset, forward_speed);
            }
            else if (diff == 2){
                avf(0.8, 0, angleMinusOffset, forward_speed);
            }
            else if (diff == -1 || diff == 3 ) { // von unten -> rechts, rechts -> unten
                    ROS_INFO("whats angleoffset now: %f", angleMinusOffset);
                    avf(0, 0.8, angleMinusOffset, forward_speed); // 90
            }
            else if (diff == 1 || diff == -3) { // oben -> links, rechts -> oben, links -> unten
                    avf(0, -0.8, angleMinusOffset, forward_speed); // -90
                    ROS_INFO("anlgeMinusOFfset: %f", angleMinusOffset);
            }
            else { // diff = 0 (also gleiche richtung bleibt erhalten)
                avf(0.8, 0, angleMinusOffset, forward_speed);
            }
             if (!plan_status) {
               return false;
            }
            cur_dir = next_dir;
            ROS_INFO("cur_dir: %d", cur_dir);
            // aktualisiere theta wert
            ros::Duration(0.2).sleep();
            ros::spinOnce();
 
        }
    }
    else if (use_PID) {
        for (size_t i = 0; i < plan.size(); i++) {
            int next_dir = plan[i];
            ROS_INFO("next_dir: %d", next_dir);
            TargetVector vec = getVectorToNextCell(x_best, y_best, next_dir);
            int forward_speed = 6;
            int diff = next_dir - cur_dir;
            // cur_dir = next_dir; // Packe ich ans ende der schleife
            double winkel = theta_best;
            theta_best = atan2(sin(winkel),cos(winkel));
            double dist_x_local = cos(theta_best) * vec.dx + sin(theta_best) * vec.dy;
            double dist_y_local = -sin(theta_best) * vec.dx + cos(theta_best) * vec.dy;
            double dist_local = std::sqrt(dist_x_local * dist_x_local + dist_y_local * dist_y_local);
            ROS_INFO("Theta best: %f", theta_best);
            double angleMinusOffset = (rotate_to_dir(next_dir, theta_best))*180/M_PI; // umgerechnet von radian in degree
            // double angle_correction = angleMinusOffset;
            ROS_INFO("angle offset: %f", angleMinusOffset);

            ROS_INFO("diff: %d", diff);
            if (diff == -2 || diff == 2) {
                rotate_degree(180, 4, PID_lib::Direction::ROTATE_RIGHT);
                drive_distance(dist_local, forward_speed); // 0.8
 
            }
            else if (diff == -1 || diff == 3 ) { // von unten -> rechts, rechts -> unten
                    ROS_INFO("whats angleoffset now: %f", angleMinusOffset);
                    drive(0, 0.8, angleMinusOffset, forward_speed); // 90
            }
            else if (diff == 1 || diff == -3) { // oben -> links, rechts -> oben, links -> unten
                    drive(0, -0.8, angleMinusOffset, forward_speed); // -90
                    ROS_INFO("anlgeMinusOFfset: %f", angleMinusOffset);
            }
            else { // diff = 0 (also gleiche richtung bleibt erhalten)
                drive(0.8, 0, angleMinusOffset, forward_speed);
            }
            if (!plan_status) {
                ROS_WARN("AVF returned false, but continuing with next step...");
                return false;
            }
            // aktualisiere theta wert
            ros::Duration(0.2).sleep();
            ros::spinOnce();
            cur_dir = next_dir;
            ROS_INFO("cur_dir: %d", cur_dir);
        }

    }
    
    // else {

    //     /*Use No AVF ODOMETRY LOGIC*/
    //     // drive distance x is vorderes, 982 in monte carlo
    //     for (size_t i = 0; i < plan.size(); i++) {
    //         int next_dir = plan[i];
    //         TargetVector vec = getVectorToNextCell(x_best, y_best, next_dir);
    //         int forward_speed = 6;
    //         int diff = next_dir - cur_dir;
    //         cur_dir = next_dir;

    //         double winkel = theta_best;
    //         /* Worls to local converting for drive distance */
    //         const double CELL_METERS = 0.8;
    //         double x_center = cell_col * CELL_METERS + 0.5 * CELL_METERS + 0.05;
    //         double y_center = cell_row * CELL_METERS + 0.5 * CELL_METERS + 0.05;
    //         theta_best = atan2(sin(winkel),cos(winkel));
    //         // double x_local = (cos(theta_best) * (x_center - x_best) + sin(theta_best) * (y_center - y_best ));
    //         // double y_local = (-sin(theta_best) * (x_center - x_best) + cos(theta_best) * (y_center - y_best ));
    //         // double set_point_theta_first = atan2(x_local, y_local);
    //         // double x_tar_local = (cos(theta_best) * (x_center - vec.target_x) + sin(theta_best) * (y_center - vec.target_y ));
    //         // double y_tar_local = (-sin(theta_best) * (x_center - vec.target_x) + cos(theta_best) * (y_center - vec.target_y ));
    //         // double coorected_x = x_tar_local - x_local;
    //         // double corrected_y = y_tar_local - y_local;
    //         double dist_x = vec.target_x - x_best;
    //         double dist_y = vec.target_y - y_best;
    //         double dist_x_local = (cos(theta_best) * (x_center - dist_x) + sin(theta_best) * (y_center - dist_y));
    //         double dist_y_local = (-sin(theta_best) * (x_center - dist_x) + cos(theta_best) * (y_center - dist_y));
    //         double dist_local = std::sqrt(dist_x_local * dist_x_local + dist_y_local * dist_y_local);
    //         double angleMinusOffset = (rotate_to_dir(next_dir, theta_best))*180/M_PI; // umgerechnet von radian in degree
    //         double dist_from_vec = vec.distance;

    //         // ROS_INFO_STREAM(
    //         //     "\n=== Coordinate Debug Info ===\n"
    //         //     << "World to Local Conversion:\n"
    //         //     << "  Cell indices: (row = " << cell_row << ", col = " << cell_col << ")\n"
    //         //     << "  Cell center (world): (x = " << x_center << ", y = " << y_center << ")\n"
    //         //     << "  Robot pose: (x = " << x_best << ", y = " << y_best << ", theta = " << theta_best << " rad)\n"
    //         //     << "  Local position of cell center: (x_local = " << x_local << ", y_local = " << y_local << ")\n"
    //         //     << "  Local target position: (x_tar_local = " << x_tar_local << ", y_tar_local = " << y_tar_local << ")\n"
    //         //     << "  Local correction vector: (dx = " << coorected_x << ", dy = " << corrected_y << ", dist = " << dist_local << ")\n"
    //         //     << "  Angle to target (set_point_theta_first) = " << set_point_theta_first << " rad"
    //         // );

    //         ROS_INFO("diff: %d", diff);
    //         if (diff == -2 || diff == 2) {
    //             rotate_degree(180-angleMinusOffset, 5, PID_lib::Direction::ROTATE_RIGHT);
    //             drive_distance(dist_from_vec, forward_speed);
    //         }
    //         else if (diff == -1 || diff == 3) { // 3: R to D, -1: L to U, D to L
    //             rotate_degree(90-angleMinusOffset, 5, PID_lib::Direction::ROTATE_RIGHT);
    //             drive_distance(dist_from_vec, forward_speed);

    //         }
    //         else if (diff == 1 || diff == -3) { // -3: D to R, 1: U to L, D to R, L to D
    //             rotate_degree(90-angleMinusOffset, 5, PID_lib::Direction::ROTATE_LEFT);
    //             drive_distance(dist_from_vec, forward_speed);

    //         }
    //         else {
    //             drive_distance(dist_from_vec, forward_speed);

    //         }
    //         if (!plan_status) {
    //            return false;
    //         }

    //     }
    //     ROS_INFO("PLAN DONE!");
    //     srv.request.left  = 0;
    //     srv.request.right = 0;
    //     diffDrive.call(srv);
    // }
    return true;
}
        /*Use AVF*/
        // if (use_PID) {
        //   double step = 0.8;
        // int grid_x = 0, grid_y = 0;
        // std::vector<Point2D> path;
        // // Build the absolute coordinates of each step
        // for (int dir : plan) {
        //     switch (dir) {
        //         case 0: grid_y -= 1; break; // RIGHT
        //         case 1: grid_x += 1; break; // UP
        //         case 2: grid_y += 1; break; // LEFT
        //         case 3: grid_x -= 1; break; // DOWN
        //     }
        //     path.push_back({ grid_x * step, grid_y * step });
        //     grid_x = 0;
        //     grid_y = 0;
        // }

        // // Send one APF goal at a time, waiting for each to finish
        // for (size_t i = 0; i < path.size(); ++i) {
        //     pink_fundamentals::PID_drive apf_srv;
        //     apf_srv.request.x      = path[i].x;
        //     apf_srv.request.y      = path[i].y;
        //     apf_srv.request.degree = 0.0;
        //     apf_srv.request.speed  = 5.0;
        //     ROS_INFO("x: %f, y: %f", path[i].x, path[i].y);

        //     ROS_INFO("APF segment %zu → (%.2f, %.2f)", i, path[i].x, path[i].y);
        //     if (!ros::service::call("/Potential_drive", apf_srv) || !apf_srv.response.success) {
        //         ROS_ERROR("APF segment %zu failed", i);
        //         return false;
        //     }
        //     }
        // }



// for execute plan works without alignment
bool executePlanCallback(pink_fundamentals::ExecutePlan::Request &req,pink_fundamentals::ExecutePlan::Response &res ) {

    ROS_INFO("Execute plan");


    /*Execute plan with avf or odometry*/
    bool use_PID = false;
    ros::param::get("/use_PID", use_PID);
    std::vector<int> plan = req.plan; // nimm plan vom request
    bool make_it_plan = true;
    while (ros::ok()) {
        ros::spinOnce();
        if (executePlan(plan)) {
            ROS_INFO("Plan executed successfully");
            res.success = true;
            safe_shutdown = true;
            return true;
        }

        safe_shutdown = true;
        res.success = false;
        return false;
    }
    // if ros::ok() ever false:
    res.success = false;
    return true;
}

/**** subscribe the best particle publisher ******/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "execute_plan_server");
    ros::NodeHandle n;
    
    /*Services*/
    resetEncoder = n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoder");
    pid_drive = n.serviceClient<pink_fundamentals::PID_drive>("PID_drive");
    align_server = n.serviceClient<pink_fundamentals::align_call>("align");
    diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    
    /*Advertises*/
    ros::ServiceServer service_executePlan = n.advertiseService("execute_plan", executePlanCallback);
    // pose_inCell_pub = n.advertise<geometry_msgs::PoseStamped>("plan_pose", 1);

    /*Topics Subscriber*/
    ros::Subscriber sub_laser = n.subscribe("scan_filtered", 1, laserCallback);
    // ros::Subscriber sub_exactPos = n.subscribe("scan_filtered", 1, publishBestPosCallback);
    // ros::Subscriber cell_sub = n.subscribe("/scan_filtered", 1, publishGridCellCallback);
    ros::Subscriber cellPos_sub = n.subscribe("/pose", 1, getCellPos);
    ros::Subscriber exactPos_sub = n.subscribe("/exact_pose", 1, exactPoseCallback);

    /*Publisher*/
    ros::Publisher cell_center_pub = n.advertise<geometry_msgs::PointStamped>("cell_center", 1);

    /*Client name                                              Service name*/
    //line_array_pub = n.advertise<visualization_msgs::MarkerArray>("ransac_lines", 1);
    //perpendicular_line_array_pub = n.advertise<visualization_msgs::MarkerArray>("perpenducular_line", 1);
    //intersect_point_pub = n.advertise<visualization_msgs::Marker>("intersect_line", 1);

    ROS_INFO("Start Now");
    while (!laserReceived) {
        ros::spinOnce();
        // ROS_WARN("No laser data received yet...");
    }
     ROS_WARN("laser data received ...");

    // ROS_WARN("Waiting for cell position...");
    while (!cell_pos_received && ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        ROS_INFO("Waiting for cell position...");
    }
    ROS_INFO("MAIN LOOP: cell_row: %d, cell_col: %d, dir: %d", cell_row, cell_col, cell_dir);
    while (!exact_pos_received && ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        // ROS_INFO("Waiting for exact_pose ...");
    }
    ROS_INFO("MAIN LOOP: x: %f, y: %f", x_best, y_best);
    ROS_INFO("Cell position received completely!");
    // if (!doAlignment()) {
    //     ROS_ERROR("Aligned failed");
    //    return false;
    // }
    ros::Rate r(10); // 10 Hz
    while (ros::ok()) {
        ros::spinOnce();
        if (safe_shutdown) {
            ROS_INFO("Shutting down cleanly...");
            ros::shutdown();  //  Shutdown happens AFTER callback ends
        }

        r.sleep();
    }

    return 0;
}
