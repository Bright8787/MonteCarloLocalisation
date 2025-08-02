#include "ros/init.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <random>
#include "create_fundamentals/DiffDrive.h"
#include "pink_fundamentals/PID_drive.h"
#include "pink_fundamentals/Wanderer.h"
#include "create_fundamentals/ResetEncoders.h"
#include "pink_fundamentals/align_call.h"
#include "pink_fundamentals/Pose.h"
#include "eigen-3.4.0/Eigen/Dense"
#include "package/PID_lib.hpp"
/*Publisher*/
ros::Publisher pose_pub;
ros::Publisher marker_pub;
ros::Publisher particle_pub;
ros::Publisher particle_pub_resampled;
ros::Publisher part_pub_zero;  // publish for debugging
visualization_msgs::Marker rays;
ros::Publisher pose_map_pub;

/*Subscriber*/
create_fundamentals::DiffDrive srv;
pink_fundamentals::Wanderer srv_wanderer;
pink_fundamentals::PID_drive srv_pid_drive;
create_fundamentals::ResetEncoders srv_encoder;
pink_fundamentals::align_call srv_align;

ros::ServiceClient resetEncoder;
ros::ServiceClient diffDrive;
ros::ServiceClient wanderer_client;
ros::ServiceClient pid_drive;
ros::ServiceClient align_server;


sensor_msgs::LaserScan latest_scan;
bool scan_received = false;
boost::shared_ptr<nav_msgs::OccupancyGrid const> map_msg;
bool map_ready = false;
bool encoder_received = false;
bool localisation_state = false;
/*Sensor Parameter*/
double min_range;
double max_range;
double angle_min;
double angle_max;
double angle_increment;
Eigen::MatrixXf particles;
Eigen::MatrixXf particles_prev;
struct EncoderData{
    /*Odometry Parameter*/
    double current_encoderLeft;
    double current_encoderRight;
    double previous_encoderLeft;
    double previous_encoderRight;
};

struct Particle {
    geometry_msgs::Pose pose;
    double theta;
    double weight;
};

struct FilteredSensorData {
    double radius;
    double angle; // radian
};

struct RobotPosition{
    double x;
    double y;
    double theta;
};

struct OdometryModel{
    double rot_1;
    double trans;
    double rot_2;
};

struct NoiseParameter{
    double alpha_1; // First rotation noise
    double alpha_2; // Noise in the first rotation caused by translation error
    double alpha_3; // Noise related to translational error in the straight motion 
    double alpha_4; // Noise in the translation caused by rotation error
};

struct AdaptiveInjection{
    double weight_slow = 0;
    double weight_fast = 0;
};

typedef enum{
    RIGHT,
    UP,
    LEFT,
    DOWN
} Dir;
// std::vector<float> laser_data;
/*Sensor Model*/
double w_hit = 0.8;
double w_rand = 0.2;
double w_max = 0.1;
double confident_level = 0;
/*Encoder data for motion model*/
EncoderData encoderData = {.current_encoderLeft = 0, .current_encoderRight = 0, .previous_encoderLeft = 0, .previous_encoderRight = 0};
/*Real Robot Position*/
RobotPosition previous_position = {.x = 0, .y = 0, .theta = 0};
RobotPosition current_position  = {.x = 0, .y = 0, .theta = 0};
/*sample_motion_model*/
OdometryModel motionModel = {.rot_1 = 0, .trans = 0, .rot_2 = 0};
NoiseParameter noiseParam;
AdaptiveInjection adaptiveInjection = {.weight_slow = 0, .weight_fast = 0};

bool isInsideMap(double x, double y) ;
double computeWeight(Eigen::MatrixXf &particles, const sensor_msgs::LaserScan &real_scan);
void publishZeroWeightParticles(const Eigen::MatrixXf & particles, ros::Publisher& pub) ;
void publishRay(ros::Publisher &publisher);
void printParticle(const Particle& p) {
    const geometry_msgs::Point& pos = p.pose.position;
    const geometry_msgs::Quaternion& q = p.pose.orientation;

    double yaw_from_quaternion = tf::getYaw(q);

    ROS_INFO("Particle:");
    ROS_INFO("  Position: x = %.3f, y = %.3f, z = %.3f", pos.x, pos.y, pos.z);
    ROS_INFO("  Theta (stored): %.3f", p.theta * 180 / M_PI);
    ROS_INFO("  Weight: %.5f", p.weight);
}

void drive(double x, double y, double degree, double speed){

    srv_pid_drive.request.x = x;
    srv_pid_drive.request.y = y;
    srv_pid_drive.request.degree = degree;
    srv_pid_drive.request.speed = speed;
    
    pid_drive.call(srv_pid_drive);

}

void startWanderer(bool start){
    srv_wanderer.request.isWander = start;
    wanderer_client.call(srv_wanderer);
}

bool doAlignment() {
    if(!align_server.call(srv_align)) return false;
    return true;
}
// void drive_motor(double speed_left, double speed_right)
// {
//     if (fabs(speed_left) > 100){
//         if(speed_left < 0) {
//             speed_left = -100;
//         }
//         else{
//             speed_left = 100;
//         }

//     }
    
//     if (fabs(speed_right) > 100){
//         if(speed_right < 0) {
//             speed_right = -100;
//         }
//         else{
//             speed_right = 100; 
//         }
//     }
    
//     srv.request.left = speed_left;
//     srv.request.right = speed_right;
//     diffDrive.call(srv);

// }

// void drive_forwards(double speed1, double distance)
// {

//     double time = 1000 *distance / (speed1 * 62 / 2);
//     drive_motor(speed1,speed1);
//     ros::Duration(time).sleep();
//     drive_motor(0,0);

// }

/********************************* Laser Data **********************************/
std::vector<FilteredSensorData>filterLaserReadings(const sensor_msgs::LaserScan& msg){

  std::vector<FilteredSensorData>readings;
  double min_angle = msg.angle_min;
  double max_angle = msg.angle_max;
  double angle_increment = msg.angle_increment;

  for (size_t i = 0 ; i < msg.ranges.size() ; i++) {

  double  r = msg.ranges[i];

  if (!std::isnan(r) && !std::isinf(r) && r >= msg.range_min && r<= msg.range_max) {
    /*Not sure if max_angle - (i * angle_increment) or min_angle + (i * angle_increment)*/
    FilteredSensorData filtered_sensor_data =  {.radius = r, .angle = min_angle + (i * angle_increment)};
    readings.push_back(filtered_sensor_data);
  }
  
}
return readings;
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {

    latest_scan = *msg;
    min_range = msg->range_min;
    max_range = msg->range_max;
    angle_increment = msg->angle_increment;
    // ROS_INFO("Receiving Laser Data");
    scan_received = true;
}

/********************************* Ray Casting **********************************/
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_msg = msg;
    if(map_msg) map_ready = true;
    // ROS_INFO("Received maze map.");
}

// Convert world to map grid coordinates
bool worldToMap(double wx, double wy, int &mx, int &my) {
    if (map_msg){
        double origin_x = map_msg->info.origin.position.x;
        double origin_y = map_msg->info.origin.position.y;
        double resolution = map_msg->info.resolution;
        
        mx = static_cast<int>((wx - origin_x) / resolution);
        my = static_cast<int>((wy - origin_y) / resolution);
        
        return mx >= 0 && my >= 0 &&
        mx < (int)map_msg->info.width &&
        my < (int)map_msg->info.height;
    } else{
        ROS_WARN("NO MAP RECEVIED");
        return false;
    }
}
// Get occupancy value at a map index
int getCell(int mx, int my) {

    return map_msg->data[my * map_msg->info.width + mx];
}
bool isOccupied(double wx, double wy) {
    
    int x_idx, y_idx = 0;
    if ( !worldToMap(wx,wy,x_idx, y_idx)) {
       ROS_WARN("ABORT CANT CONVERT MAP POS TO OG INDEX in isOccupied");
       return false;
    }
    return (getCell(x_idx ,y_idx) > 50) ;  
}


bool isValidPos(double wx, double wy){

    double offset = 0.05;
    if (!isInsideMap(wx, wy)) return false;

    std::vector<std::pair<double, double>> offsets = {
        {0, 0}, {offset, 0}, {0, offset}, {-offset, 0}, {0, -offset},
        {offset, offset}, {offset, -offset}, {-offset, offset}, {-offset, -offset}
    };

    for (auto& off : offsets) {
        if (isOccupied(wx + off.first, wy + off.second)) {
            return false; // Ungültige Position, weil eine Nachbarzelle besetzt ist
        }
    }

    return true; // Alles frei

}

bool isPathValid(double x_prev, double y_prev , double x_current, double y_current){
    int x_prev_idx, x_current_idx, y_prev_idx, y_current_idx = 0;

    if (!isInsideMap(x_prev,y_prev) ||  !isInsideMap(x_current,y_current)){
        return false;
    }
   
    if ( !worldToMap(x_prev,y_prev, x_prev_idx, y_prev_idx)) {
       ROS_WARN("ABORT CANT CONVERT MAP POS TO OG INDEX in isPathValid prev");
    }
    if ( !worldToMap(x_current,y_current, x_current_idx, y_current_idx)) {
        ROS_WARN("ABORT CANT CONVERT MAP POS TO OG INDEX in isPathValid next");
     }
     int dx = abs(x_current_idx - x_prev_idx);
     int dy = abs(y_current_idx - y_prev_idx);
     int sx = x_prev_idx < x_current_idx ? 1 : -1;
     int sy = y_prev_idx < y_current_idx ? 1 : -1;
     int err = dx - dy;
 
     while (true) {
         if (isOccupied(x_prev_idx, y_prev_idx))
             return false;  // Intersected with occupied cell
 
         if (x_prev_idx == x_current_idx && y_prev_idx == y_current_idx)
             break;
 
         int e2 = 2 * err;
         if (e2 > -dy) { err -= dy; x_prev_idx += sx; }
         if (e2 < dx)  { err += dx; y_prev_idx += sy; }
     }
 
    return true;  // No obstacles along the path
}

// Raycast to find distance from pose at a relative angle
double raycast(const geometry_msgs::Pose& pose, double angle_offset_deg, geometry_msgs::Point& hit_point, double max_range) {
    double angle = tf::getYaw(pose.orientation) + angle_offset_deg * M_PI / 180.0;
    double x = pose.position.x;
    double y = pose.position.y;
    // ROS_INFO("Max range: %f", max_range); 
    double step = 0.05;
    for (double r = 0.0; r < max_range; r += step) {
        double rx = x + r * cos(angle);
        double ry = y + r * sin(angle);
        int mx, my;
        if (!worldToMap(rx, ry, mx, my)) {
            break;
        }
        if (getCell(mx, my) > 50) {
            hit_point.x = rx;
            hit_point.y = ry;
            hit_point.z = 0.0;
            return r;
        }
    }
    hit_point.x = x + max_range * cos(angle);
    hit_point.y = y + max_range * sin(angle);
    hit_point.z = 0.0;
    return max_range;
}

geometry_msgs::Point to_cell(int row, int col, double x_move = 0.0, double y_move = 0.0) {
    geometry_msgs::Point p;
    const double CELL_METERS = 0.8;

    // Center of the cell
    double base_x = col * CELL_METERS + 0.5 * CELL_METERS;
    double base_y = row * CELL_METERS + 0.5 * CELL_METERS;

    // Add relative movement from the center
    p.x = base_x + x_move;
    p.y = base_y + y_move;
    p.z = 0.0;
    return p;
}

/********************************* Sampling Particles on the map **********************************/
double sample(double variance) {
    static std::default_random_engine generator(std::random_device{}());
    std::normal_distribution<double> distribution(0.0, std::sqrt(variance));
    return distribution(generator);
}
Eigen::MatrixXf sampleParticles(int num_particles) {

    Eigen::MatrixXf particles(4, num_particles); // x,y,theta,weight
    for (int i = 0; i < 4; ++i) {
        particles.row(i).setConstant(0);  
    }

    int cell_size = 8;
    int maze_coordiante_width = map_msg->info.width / cell_size;
    int maze_coordiante_height = map_msg->info.height / cell_size;
    double total_weight = 0.0;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> cell_x(0, maze_coordiante_height -1 ); // random available cell_x
    std::uniform_int_distribution<> cell_y(0, maze_coordiante_width  -1 ); // random available cell_y
    std::uniform_real_distribution<> offset(-0.3, 0.3);  // within 0.8m cell
    std::uniform_real_distribution<> yaw_dist(-M_PI, M_PI);

    for (int i = 0; i < num_particles; ++i) {
        /*Generate Random Particle*/
        double orientation = yaw_dist(gen) ;
        // double orientation = yaw_dist(gen); // in radian
        geometry_msgs::PoseStamped pose_msg;
        /*Might be a bug from euler to quaternion*/
        pose_msg.pose.position = to_cell(cell_x(gen), cell_y(gen), offset(gen), offset(gen));
        // pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(orientation);  // In quaternion
        particles(0, i) = pose_msg.pose.position.x + 0.05;    // x + global offset because of map
        particles(1, i) = pose_msg.pose.position.y + 0.05;   // y + global offset because of map
        particles(2, i) = orientation;               // theta
        particles(3, i) = 1.0;                      // weight
        // ROS_INFO("Cell x: %f, Cell y: %f, Orientation to x Axis: %f", particles(0, i), particles(1, i), particles(2, i) * 180 / M_PI);
    }

    return particles;
}
double uniformJitter(double min_angle, double max_angle) {
    static std::default_random_engine generator(std::random_device{}());
    std::uniform_real_distribution<double> distribution(min_angle, max_angle);
    return distribution(generator);
}

Eigen::MatrixXf resampleParticles(Eigen::MatrixXf &particles_local) {

    Eigen::MatrixXf new_particles(4, particles_local.cols());
    ROS_INFO("Resampling..");
    int N = particles_local.cols();
    double total_weight = computeWeight(particles_local, latest_scan);  // Compute weight of all particles in one go. Return totalweight
    double weight_avg = total_weight/N;
    publishRay(marker_pub);
    /*Adaptive Injection*/
    adaptiveInjection.weight_slow =  adaptiveInjection.weight_slow + 0.05 * (weight_avg - adaptiveInjection.weight_slow); // alpha 0.01 slow
    adaptiveInjection.weight_fast =  adaptiveInjection.weight_fast + 0.1 * (weight_avg - adaptiveInjection.weight_fast); // alpha 0.8 fast

    double p_inject = std::max(0.0, 1.0 - (adaptiveInjection.weight_fast / adaptiveInjection.weight_slow));

    int injected_particles_num = 0;
    std::vector<double> cdf(N, 0.0);
    particles_local(3, 0) = particles_local(3, 0)/total_weight;
    cdf[0] = particles_local(3, 0);
    // cumulative distribution function, cdf from particle 1 with weight 0.1 = 0.1,
    // cdf from particle 2 with weight 0.3 = (0.1 +0.3) = 0.4
    for (int i = 1; i < N; ++i) {
        
        particles_local(3, i) = particles_local(3, i)/total_weight; // Gewicht normalisieren
        cdf[i] = cdf[i - 1] + (particles_local(3, i));    
    }

    // zufallsgenerator
    publishZeroWeightParticles(particles_local,part_pub_zero);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    for (int i = 0; i < N; ++i) {
        double r = dis(gen);
        /*With dynamic Injection*/
        /*If current Particles dont perform good, we add new one*/
        if(r < p_inject ){ 
            ROS_INFO("Confident Level low");
            Eigen::MatrixXf new_particle = sampleParticles(1);
            new_particles(0,i) = new_particle(0,0); // x
            new_particles(1,i) = new_particle(1,0); // y
            new_particles(2,i) = new_particle(2,0); // theta
            new_particles(3,i) = 1.0 / N; // new weight uniformly
            injected_particles_num++;
        }
        else{

            auto it = std::lower_bound(cdf.begin(), cdf.end(), r); // Find position in the CDF
            int idx = std::distance(cdf.begin(), it); // Get the index of the selected particle
            // ROS_INFO("No Jitter -> x: %f,y: %f, theta: %f, weight: %f", particles(0, idx), particles(1,idx), particles(2, idx) * 180.0 / M_PI, particles(3, idx));
            new_particles(0,i) = particles_local(0,idx) + uniformJitter(-0.05,0.05); // x + noise ("Jitter") 0-0.05 m noise 0.0001
            new_particles(1,i) = particles_local(1,idx) + uniformJitter(-0.05,0.05); // y + noise ("Jitter") 0-0.05 m noise 0.0001
            //double jitter_theta = particles(2, idx);
            double jitter_theta = particles(2, idx) + uniformJitter(-M_PI/6, M_PI/6);
            // ROS_INFO("Jitter Theta is %f", jitter_theta);
            new_particles(2, i) = atan2(sin(jitter_theta), cos(jitter_theta));

            new_particles(3,i) = 1.0 / N; // new weight uniformly
            // ROS_INFO("With Jitter -> x: %f,y: %f, theta: %f, weight: %f", new_particles(0, i), new_particles(1, i), new_particles(2, i) * 180.0 / M_PI, new_particles(3, i));
        }
     
    }
    // for (int j = 0; j < particles.cols(); ++j) {
        
    //     ROS_INFO("after Resampling x: %f, y: %f, theta: %f, Gewicht: %f",new_particles(0, j),new_particles(1, j),new_particles(2, j),new_particles(3, j) );

    // }
    ROS_INFO("New Injected Particles: %d", injected_particles_num );
    return new_particles;
}

void publishParticles(const Eigen::MatrixXf &particles, ros::Publisher &publisher) {
    ROS_INFO("Publishing");
    geometry_msgs::PoseArray particle_array;
    particle_array.header.frame_id = "map";
    particle_array.header.stamp = ros::Time::now();

    for (int i = 0; i < particles.cols(); i++) {  // Assume particles is a vector of your particle data
        geometry_msgs::Pose pose;
        pose.position.x = particles(0,i);
        pose.position.y = particles(1,i);
        pose.orientation = tf::createQuaternionMsgFromYaw(particles(2,i));  // In quaternion

        particle_array.poses.push_back(pose);
    }

    publisher.publish(particle_array);

}
void publishZeroWeightParticles(const Eigen::MatrixXf & particles, ros::Publisher& pub) {
    geometry_msgs::PoseArray pose_array;
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = "map";  // or "odom", depending on your TF

    const double epsilon = 0.00;  // Threshold for "near zero"

    for (int i = 0; i < particles.cols(); ++i) {
        double weight = particles(3, i);
        if (std::abs(weight) == epsilon) {
            geometry_msgs::Pose pose;
            pose.position.x = particles(0, i);  // x
            pose.position.y = particles(1, i);  // y
            pose.position.z = 0;

            double theta = particles(2, i);     // theta
            tf::Quaternion q;
            q.setRPY(0, 0, theta);
            pose.orientation.x = q.x(); 
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();

            pose_array.poses.push_back(pose);
        }
    }

    pub.publish(pose_array);
}
// void publishParticles(const Eigen::MatrixXf &particles, ros::Publisher &publisher) {
//     ROS_INFO("Publishing");
//     geometry_msgs::PoseArray particle_array;
//     particle_array.header.frame_id = "map";
//     particle_array.header.stamp = ros::Time::now();
//
//     for (int i = 0; i < particles.cols(); i++) {  // Assume particles is a vector of your particle data
//         if (particles(3,i) >= 0.001) { // added if statement to only publish the particles with more bup than zero
//             geometry_msgs::Pose pose;
//             pose.position.x = particles(0,i);
//             pose.position.y = particles(1,i);
//             pose.orientation = tf::createQuaternionMsgFromYaw(particles(2,i));  // In quaternion
//
//             particle_array.poses.push_back(pose);
//             ROS_INFO("Particle %d != 0: -> x: %.2f, y: %.2f, theta: %.2f, weight: %.5f", i, particles(0, i), particles(1, i), particles(2, i) * 180.0 / M_PI, particles(3, i));
//         }
//     }
//
//     publisher.publish(particle_array);
//
// }

// // to debugging
// void publishParticles_pub_zero(const Eigen::MatrixXf &particles, ros::Publisher &publisher) {
//     geometry_msgs::PoseArray particle_array;
//     particle_array.header.frame_id = "map";
//     particle_array.header.stamp = ros::Time::now();
//
//     for (int i = 0; i < particles.cols(); i++) {
//         if (particles(3, i) < 0.001) {  // Only publish pub around 0
//             geometry_msgs::Pose pose;
//             pose.position.x = particles(0,i);
//             pose.position.y = particles(1,i);
//             pose.orientation = tf::createQuaternionMsgFromYaw(particles(2,i));
//
//             particle_array.poses.push_back(pose);
//             ROS_INFO("Particle %d == 0: -> x: %.2f, y: %.2f, theta: %.2f, weight: %.5f", i, particles(0, i), particles(1, i), particles(2, i) * 180.0 / M_PI, particles(3, i));
//         }
//
//     }
//
//     //
//     // if (!particle_array.poses.empty()) {
//     //     ROS_INFO("Publishing %lu zero-weight particles", particle_array.poses.size());
//     //     publisher.publish(particle_array);
//     // } else {
//     //     ROS_INFO("No zero-weight particles to publish.");
//     // }
//     publisher.publish(particle_array);
//
//
// }
/*********************************    Evaluate Sensor data  Sensor Model    **********************************/
std::vector<FilteredSensorData> filterAngles(const std::vector<FilteredSensorData>& laser_data, double lower, double upper) {
    std::vector<FilteredSensorData> filtered;

    for (int i = 0; i < laser_data.size(); i++) { // skip every 50 beams for speed
        if(laser_data[i].angle * 180.0 / M_PI  > lower && laser_data[i].angle * 180.0 / M_PI  < upper ){
            filtered.push_back(laser_data[i]);
        }
    }

    return filtered;
}

// ComputeWeight weill return the totalWeight of the particles matrix
double computeWeight(Eigen::MatrixXf &particles_local, const sensor_msgs::LaserScan &real_scan)
{
    double sigma = 0.1;           // sensor noise
    double max_laser_range = 1.0; // 1 meter
    double prob = 0;
    double totalWeight = 0;
    geometry_msgs::Point end;
    geometry_msgs::Pose pos;
    int N = particles_local.cols();
    std::vector<FilteredSensorData> laser_data = filterAngles(filterLaserReadings(real_scan), -90.00, 90.00);
    for (int j = 0; j < N; ++j) {
    /* 1 Beam is 0.352 degree
       Angle min : -135 degree
       Angle max :  120 degree */
            prob = 0;
            for (int i = 0; i < laser_data.size(); i += 3) // skip every 20 beams for speed
            {   
                // For RayCast   
                pos.position.x = particles_local(0, j) ;
                pos.position.y = particles_local(1, j) ;
                pos.orientation = tf::createQuaternionMsgFromYaw(particles_local(2,j));  // In quaternion
                
                double angle = (  laser_data[i].angle) * 180.0 / M_PI; // convert to degree
                
                double expected_distance = raycast(pos, angle, end, max_laser_range) - 0.15; // Laser Offset
                double observed_distance = laser_data[i].radius;
                ROS_INFO("Expected_distance: %f, Observed_distance: %f, Degree: %f", expected_distance, observed_distance, angle);
                /*Visualize the ray*/
                rays.points.push_back(pos.position);
                rays.points.push_back(end);
                double diff = observed_distance - expected_distance;
                // 1. Normal distribution
                prob += w_hit * std::exp(-(diff * diff) / (2 * sigma * sigma) ) / sigma * std::sqrt(2.0f * M_PI);
                // 2. Uniform distribution - There will always be random noise because we filtered all of the invalid sensor data(In case it is invalud the uniform random will be zero)
                // prob += w_max * 1 / max_laser_range; // 1/Max_range where Max_range is 1 meter
                // 3. Random distribution 
                prob += w_rand * ((std::abs(observed_distance - max_laser_range) < 0.01) ? 1.0 : 0.0);        
            }   
            if(!isValidPos(particles_local(0, j),particles_local(1, j))){
                prob = 0;
                // ROS_INFO("Setting Particle Weight to 0 in computeWeight");  
            }            
            // ROS_INFO("x: %f,y: %f, theta: %f, weight: %f", particles_local(0, j), particles_local(1, j), particles_local(2, j) * 180.0 / M_PI, particles_local(3, j));
            particles_local(3, j) = prob ; // Laser Offset
            totalWeight += particles_local(3, j);
    }
    // The prob will be later normalized for resampling
    return totalWeight;
}
/*********************************    Evaluate Odometry data Odometry model     **********************************/
/*Check if robot position is outside the map or too close to the wall*/
bool isInsideMap(double x, double y) {
    double map_min_x = map_msg->info.origin.position.x;
    double map_min_y = map_msg->info.origin.position.y;
    double map_max_x = map_min_x + map_msg->info.width * map_msg->info.resolution;
    double map_max_y = map_min_y + map_msg->info.height * map_msg->info.resolution;

    return (x >= map_min_x && x < map_max_x) && (y >= map_min_y && y < map_max_y);
}


void sampleMotionModelOdometry(RobotPosition &current_position, RobotPosition &previous_position){
    /*Could be real Robot Movement --- Might have to be debugged*/
    // ROS_INFO("Before Noise Motion model -> Prev Pos : x = %f, y = %f, Curr Pos : x = %f, y = %f, Prev Theta : %f , Curr Theta : %f ", previous_position.x, previous_position.y, current_position.x, current_position.y,previous_position.theta * 180 / M_PI,current_position.theta * 180 / M_PI );
    motionModel.rot_1 = atan2(current_position.y - previous_position.y, current_position.x - previous_position.x) - previous_position.theta;
    motionModel.trans = std::sqrt( (current_position.y - previous_position.y)*(current_position.y - previous_position.y) + 
                                   (current_position.x - previous_position.x)*(current_position.x - previous_position.x) );
    motionModel.rot_2 = current_position.theta - previous_position.theta - motionModel.rot_1;
    // ROS_INFO("Before Noise Motion model -> Rot_1 : %f, Trans : %f, Rot_2 : %f ", motionModel.rot_1 , motionModel.trans, motionModel.rot_2 );

    /*Real Movement + Encoder uncertainty(sampled noise) --- Might have to be debugged*/
    double rot_1_temp = motionModel.rot_1 + sample(noiseParam.alpha_1 * fabs(motionModel.rot_1) + noiseParam.alpha_2* motionModel.trans);
    double translation_noise = sample(noiseParam.alpha_3  * motionModel.trans + noiseParam.alpha_4*(fabs(motionModel.rot_1)  + fabs(motionModel.rot_2) ));
    double trans_temp = motionModel.trans + translation_noise ;
    // ROS_INFO("Translation noise: %f", translation_noise );
    double rot_2_temp = motionModel.rot_2 + sample(noiseParam.alpha_1  * fabs(motionModel.rot_2) + noiseParam.alpha_2*motionModel.trans);
    
    motionModel.rot_1 = rot_1_temp;
    motionModel.trans = trans_temp;
    motionModel.rot_2 = rot_2_temp;

    // ROS_INFO("After Noise Motion model -> Rot_1 : %f, Trans : %f, Rot_2 : %f ", motionModel.rot_1 , motionModel.trans, motionModel.rot_2  );
}

void diffDriveModel(EncoderData &encoderData, RobotPosition &current_position, RobotPosition &previous_position){


    double d_left = (encoderData.current_encoderLeft - encoderData.previous_encoderLeft) * PID_lib::wheel_size * 0.5;
    double d_right = (encoderData.current_encoderRight - encoderData.previous_encoderRight) * PID_lib::wheel_size * 0.5;
    double d_center = 0.5 * (d_left + d_right);
    double delta_theta = (d_left - d_right) / PID_lib::wheel_space;
    /*New Robot Position*/
    double current_theta = delta_theta + previous_position.theta ;
    double current_pos_x =  previous_position.x + d_center * cos(previous_position.theta + 0.5 * delta_theta) ;
    double current_pos_y =  previous_position.y + d_center * sin(previous_position.theta + 0.5 * delta_theta) ;
    
    current_position = {.x = current_pos_x, .y = current_pos_y, .theta = atan2(sin(current_theta), cos(current_theta)) };
    sampleMotionModelOdometry(current_position,previous_position);
    /*Update Robot previous Position*/
    previous_position = current_position;
    /*Update Encoder Data*/
    encoderData.previous_encoderLeft = encoderData.current_encoderLeft;
    encoderData.previous_encoderRight = encoderData.current_encoderRight;

}
void updateParticlePos(Eigen::MatrixXf &particles_local ){

    // Vector of current heading (θ)
    Eigen::ArrayXf theta = particles_local.row(2).transpose().array();

    // Apply motion model: rotation + translation + rotation
    Eigen::ArrayXf moved_heading = theta + motionModel.rot_1 ;
    Eigen::ArrayXf dx = motionModel.trans * moved_heading.cos();
    Eigen::ArrayXf dy = motionModel.trans * moved_heading.sin();

    // Update particles_local
    particles_local.row(0) += dx.matrix().transpose();              // x
    particles_local.row(1) += dy.matrix().transpose();              // y
    particles_local.row(2) += Eigen::ArrayXf::Constant(particles_local.cols(), motionModel.rot_1 + motionModel.rot_2).matrix().transpose(); // theta

}
void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr &msg)
{
    encoder_received = true; // <-- Mark that we got data

    encoderData.current_encoderLeft = msg->encoderLeft;
    encoderData.current_encoderRight = msg->encoderRight; 
    // ROS_INFO("Receiving Encoder data");
    // diffDriveModel(encoderData,current_position,previous_position);
    //
    // ROS_INFO("left encoder: %f, right encoder: %f",  encoderData.current_encoderLeft, encoderData.current_encoderRight);
}
/****************************Execute Localisation***********************************************/
void publishRay(ros::Publisher &publisher){
    rays.header.frame_id = "map"; // or "odom" depending on your TF
    rays.header.stamp = ros::Time::now();
    rays.ns = "raycast";
    rays.id = 0;
    rays.type = visualization_msgs::Marker::LINE_LIST;
    rays.action = visualization_msgs::Marker::ADD;
    rays.scale.x = 0.02; // Line width
    rays.color.r = 1.0;
    rays.color.g = 0.0;
    rays.color.b = 0.0;
    rays.color.a = 1.0;
    rays.pose.orientation.w = 1.0;
    
    publisher.publish(rays);

}
bool isLocalizationLost_densitiy_cluster(Eigen::MatrixXf& particles, float cluster_distance = 0.3, float cluster_ratio_threshold = 0.8) {
    int N = particles.cols(); // total number of particles
    std::vector<int> cluster_sizes(N, 0); // to save the similiar (clustered) particles
    for (int i = 0; i < N; i++){ // iterate over particles
        int count = 0; // saves how many other particles are close
        float xi = particles(0, i); // x coordinate of particle
        float yi = particles(1, i); // y coordinate of particle

        for (int j = 0; j < N; ++j) { // loop to compare again with the particle i
            if (i == j) continue; // no comparing to itself
            float xj = particles(0, j);
            float yj = particles(1, j);

            float dist_sq = std::pow(xi - xj, 2) + std::pow(yi - yj, 2); // distance between particle i and j
            if (dist_sq <= cluster_distance * cluster_distance) { // within threshold should be counted, its close
                ++count;
            }
        }
        cluster_sizes[i] = count; // amount of close particles of particle i
    }
    int max_cluster_size = *std::max_element(cluster_sizes.begin(), cluster_sizes.end()); // find the biggest number which means the most close particles of i
    float cluster_ratio = static_cast<float>(max_cluster_size) / static_cast<float>(N); // to get values between 0 and 1, N should be the same as "cluster_sizes.size()"
    double total_weight = computeWeight(particles, latest_scan);  // now compare it to laser data
    double average_weight = total_weight / N;
  //  float mean_x = particles.row(0).mean();
  //  float mean_y = particles.row(1).mean();
  //  bool pose_inside = isInsideMap(mean_x, mean_y);
    return (cluster_ratio < cluster_ratio_threshold || average_weight < 0.8); // || !pose_inside
   // return cluster_ratio < cluster_ratio_threshold; // if left is smaler than right, its smaler than the threshold we want, the particles are too spread out, could be due to uncertainty
}

double wrapTo2Pi(double angle_rad) {
    const double TWO_PI = 2.0 * M_PI;
    double wrapped = std::fmod(angle_rad, TWO_PI);
    if (wrapped < 0)
        wrapped += TWO_PI;
    return wrapped;
}
void publishPosMsg(double wx, double wy, double angle){
    // Orientation in reference with x-Axis (our horizontal axis)
    pink_fundamentals::Pose pos;    
    Dir dir;
    const double CELL_METERS = 0.8;

    // World Pos to Map Pos
    double col_wx = (wx - 0.5 * CELL_METERS) / CELL_METERS;
    double row_wx = (wy - 0.5 * CELL_METERS) / CELL_METERS;

    int col = static_cast<int>(std::floor(col_wx + 0.5));
    int row = static_cast<int>(std::floor(row_wx + 0.5));
    double angle_in_2PI=  wrapTo2Pi(angle) * 180.0 / M_PI;
    ROS_INFO("Testing angle_in_2PI: %f",angle_in_2PI );
    if (angle_in_2PI >= 45 && angle_in_2PI < 135)
        dir = DOWN;
    else if (angle_in_2PI >= 135 && angle_in_2PI < 225)
        dir = LEFT;
    else if (angle_in_2PI >= 225 && angle_in_2PI < 315)
        dir = UP;
    else
        dir = RIGHT;

    pos.row = row;
    pos.column = col;
    pos.orientation =  dir;

    pose_map_pub.publish(pos);
}
double localisationConfinent(Eigen::MatrixXf& particles, double threshold){

    int N = particles.cols();

    // 1. Find max weight and index
    int max_idx;
    particles.row(3).maxCoeff(&max_idx);
    
    // 2. Get max-weight particle's pose
    double x_max = particles(0, max_idx);
    double y_max = particles(1, max_idx);
    double theta_max = particles(2, max_idx);
    double weight_max = particles(3, max_idx);
    // 3. Count nearby particles (thresholds can be tuned)
    int count = 0;
    double pos_thresh = 0.4;     // meters
    double theta_thresh = 10.0 * M_PI / 180.0; // 10 degrees in radians
    // Check the distance of all particle to the particle_max
    for (int j = 0; j < N; ++j) {
        double dx = particles(0, j) - x_max;
        double dy = particles(1, j) - y_max;
        double d_theta = fabs(theta_max  - particles(2, j));
        // d_theta < theta_thresh
        if (std::sqrt(dx*dx + dy*dy) < pos_thresh ){
            count++;
            ROS_INFO("x: %f, y: %f, theta: %f",particles(0, j),particles(1, j),particles(2, j));

        }
    }
    // 4. Check if cluster formed
    double ratio = static_cast<double>(count) / N;
    if (ratio > threshold) {

        ROS_INFO("Cluster formed around best particle (%.2f%% of particles nearby)", ratio * 100.0);
        ROS_INFO("Pos is x: %.3f, y = %.3f, Theta = %.3f, Weight = %.3f", x_max,y_max,theta_max *180/M_PI,weight_max);

    } else {

        ROS_INFO("No strong cluster yet (%.2f%% nearby)", ratio * 100.0);
    }
    return ratio ;
    

}
void executeParticleFilter(const ros::TimerEvent&){

    // if( confident_level < 0.90) {
        ROS_INFO("Calling Particle Filter");
        // Differential drive model to get new x,y,theta + Motion Model
        diffDriveModel(encoderData,current_position,previous_position);
        // Update Position of the particles
        updateParticlePos(particles);
        // Use Sensor Model to evaluate new laser data and resample
        
        particles = resampleParticles(particles); // Sensor model is in resampleParitlces -> computeWeight
        publishParticles(particles, particle_pub_resampled);
        confident_level = localisationConfinent(particles,0.90);
    // }
    // else{
    //     ROS_INFO("Confident");
    //     startWanderer(false); 
    //     doAlignment();
    // }
   

}
/****************************************************************************************/
int main(int argc, char** argv) {
    ros::init(argc, argv, "monte_carlo_matrix");
    ros::NodeHandle nh;
    /*Topics*/
    ros::Subscriber scan_sub = nh.subscribe("/scan_filtered", 1, laser_callback); // mapCallback???
    ros::Subscriber map_sub = nh.subscribe("/maze_map", 1, mapCallback);
    ros::Subscriber sub_rad = nh.subscribe("sensor_packet", 1, sensorCallback);
    /*Services*/
    diffDrive = nh.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    pid_drive = nh.serviceClient<pink_fundamentals::PID_drive>("PID_drive");
    wanderer_client = nh.serviceClient<pink_fundamentals::Wanderer>("Wanderer");
    resetEncoder = nh.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");
    align_server = nh.serviceClient<pink_fundamentals::align_call>("align");

    /*Advertises*/
    particle_pub = nh.advertise<geometry_msgs::PoseArray>("sampled_particles", 1);
    particle_pub_resampled = nh.advertise<geometry_msgs::PoseArray>("resampled_particles", 1);
    part_pub_zero = nh.advertise<geometry_msgs::PoseArray>("zero_weight_particles", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("test_pose", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("raycast", 1);
    pose_map_pub = nh.advertise<pink_fundamentals::Pose>("pose", 10);

    if(resetEncoder.call(srv_encoder)){
        ROS_INFO("Encoder Reset");
    }
    else{
        ROS_INFO("Encoder not Reset");
    }

    ros::Rate loop_rate(10);

    while (ros::ok() && !encoder_received) {
        ros::spinOnce();
        ROS_INFO("Waiting for encoder");
        loop_rate.sleep();  
    }
    while (ros::ok() && !map_msg) {
        ros::spinOnce();
        ROS_INFO("Waiting for /maze_map");
        loop_rate.sleep();
    }
    ROS_INFO("/maze_map... and encoder Received");  
    noiseParam = {.alpha_1 = 0.0001, .alpha_2 = 0.0001, .alpha_3 = 0.0001, .alpha_4 = 0.0001};
    // Start Localisation
    localisation_state = true;
    /*Test Particle Distribution*/
    ROS_INFO("First Sample finished");
    particles = sampleParticles(1);
    // publishParticles(particles, particle_pub_resampled);

    particles.row(0).setConstant(0.45);  
    particles.row(1).setConstant(0.45);  
    particles.row(2).setConstant(-M_PI/2);  
    particles.row(3).setConstant(0);  
    // particles = resampleParticles(particles);
    double total_weight = computeWeight(particles, latest_scan);  // Compute weight of all particles in one go. Return totalweight
    publishRay(marker_pub);
    publishParticles(particles, particle_pub_resampled);

    // startWanderer(true);
    // ros::Timer timer = nh.createTimer(ros::Duration(0.8), executeParticleFilter);

    ros::spin();
    return 0;
}

    // particles.row(0).setConstant(0.45);  
    // particles.row(1).setConstant(0.45);  
    // particles.row(2).setConstant(M_PI/2);  
    // particles.row(3).setConstant(0);  

    // ROS_INFO("Publishing %lu particles", particles.cols());
    // while(ros::ok){
    //     ros::spinOnce();
    //     Eigen::MatrixXf particles1 = resampleParticles(particles);
    //     // if(!isValidPos(particles(0,0),particles(1,0))){
    //     // ROS_INFO("Not Valid Pos");
            
    //     // }
    //     if(isLocalized(particles1,0.035)){
    //         ROS_INFO("It is localised");    
    //     }
    //     publishParticles(particles1, particle_pub_resampled);
    // }

    // for (int j = 0; j < particles.cols(); ++j) {
        
    //     ROS_INFO("First Batch x: %f, y: %f, theta: %f",particles(0, j),particles(1, j),particles(2, j));

    // }