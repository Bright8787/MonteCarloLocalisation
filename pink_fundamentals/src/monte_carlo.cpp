#include "ros/init.h"
#include <ros/ros.h>
#include <cstdlib>
#include <std_msgs/Empty.h>
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
#include "create_fundamentals/SensorPacket.h"
#include "create_fundamentals/PlaySong.h"
#include "create_fundamentals/StoreSong.h"
#include "create_fundamentals/ResetEncoders.h"
#include <utility>         // for std::pair
#include "pink_fundamentals/PID_drive.h"
#include "pink_fundamentals/Wanderer.h"
#include "pink_fundamentals/align_call.h"
#include "pink_fundamentals/Pose.h"
#include "pink_fundamentals/ExactPose.h"
#include <unordered_map>
#include "package/songs_lib.hpp"

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
ros::Publisher exact_pose_map_pub;
ros::Publisher pos_target_pub;

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
ros::ServiceClient storeSongClient;
ros::ServiceClient playSongClient;
/*Timer*/
ros::Timer timer;
sensor_msgs::LaserScan latest_scan;
bool scan_received = false;
boost::shared_ptr<nav_msgs::OccupancyGrid const> map_msg;
bool map_ready = false;
bool encoder_received = false;

bool localisation_state = false;
bool is_localized= false;

/*Sensor Parameter*/
double min_range;
double max_range;
double angle_min;
double angle_max;
double angle_increment;

double x_best;
double y_best;
double theta_best;
double w_best;

double cluster_threshold = 0.1;
double confident_threshold = 0.39;
double confident_level_threshhold = 0.80;
bool resample_confident = true;
Eigen::MatrixXf particles;
Eigen::MatrixXf particles_prev;
int Particle_number = 1500;
bool was_lost_before = false;

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

/********************************* Gauss LUT  **********************************/

class GaussianLookup {
    public:
        GaussianLookup(double sigma, double resolution = 0.0001f)
            : sigma(sigma), min_diff(0.0f), max_diff(1.1f), resolution(resolution) {
            
            size = static_cast<int>((max_diff - min_diff) / resolution) + 1;
            table.resize(size);
    
            double denom = sigma * std::sqrt(2.0f * M_PI);
            for (int i = 0; i < size; ++i) {
                double diff = min_diff + i * resolution;
                table[i] = std::exp(-(diff * diff) / (2 * sigma * sigma)) / denom;
            }
        }
    
        double get(double diff) const {
            if (diff < min_diff || diff > max_diff)
                return 0.0f;
    
            double index_f = (diff - min_diff) / resolution;
            int index = static_cast<int>(index_f);
            
            // Linear interpolation 
            if (index + 1 < size) {
                double weight = index_f - index;
                return (1.0f - weight) * table[index] + weight * table[index + 1];
            } else {
                return table[index];
            }
        }
    
    private:
        double sigma;
        double min_diff, max_diff;
        double resolution;
        int size;
        std::vector<double> table;
    };
GaussianLookup exp_gauss(0.1);  // default resolution = 0.001 sigma = 0.1
/********************************* Global Parameter **********************************/
/*Sensor Model*/
double w_hit = 0.8;
double w_rand = 0.2;
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
std::unordered_map<int, std::pair<double, double>> ray_direction_lookup;
bool isInsideMap(double x, double y) ;
double computeWeight(Eigen::MatrixXf &particles, const sensor_msgs::LaserScan &real_scan);
void publishZeroWeightParticles(const Eigen::MatrixXf & particles, ros::Publisher& pub) ;
void publishRay(ros::Publisher &publisher);

void publishBestParticle(double x_best, double y_best, double theta_best, ros::Publisher& pub) {
    geometry_msgs::PoseArray pose_array;

    // ROS_INFO("Publishing Best Particle");
    pose_array.header.frame_id = "map";
    pose_array.header.stamp = ros::Time::now();

    geometry_msgs::Pose pose;
    pose.position.x = x_best;
    pose.position.y = y_best;
    pose.orientation = tf::createQuaternionMsgFromYaw(theta_best);  // In quaternion

    pose_array.poses.push_back(pose);

    pub.publish(pose_array);
}

void publishTargetPos(double x_best, double y_best, double theta_best, ros::Publisher& pub) {
    geometry_msgs::PoseArray pose_array;

    // ROS_INFO("Publishing Middle of Cell");
    pose_array.header.frame_id = "map";
    pose_array.header.stamp = ros::Time::now();

    geometry_msgs::Pose pose;
    pose.position.x = x_best;
    pose.position.y = y_best;
    pose.orientation = tf::createQuaternionMsgFromYaw(theta_best);  // In quaternion
    pose_array.poses.push_back(pose);



    pub.publish(pose_array);
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
    while( !wanderer_client.call(srv_wanderer));
}

void shutdownCallback(const std_msgs::Empty::ConstPtr&) {
    ROS_WARN("Shutdown requested!");
    ros::shutdown();
}

/********************************* Laser Data **********************************/
std::vector<FilteredSensorData>filterLaserReadings(const sensor_msgs::LaserScan& msg){

  std::vector<FilteredSensorData>readings;
  double min_angle = msg.angle_min;
  double max_angle = msg.angle_max;
  double angle_increment = msg.angle_increment;

  for (size_t i = 0 ; i < msg.ranges.size() ; i++) {

  double  r = msg.ranges[i];
  if(std::isnan(r) || std::isinf(r)){
    FilteredSensorData filtered_sensor_data =  {.radius = 1.05, .angle = min_angle + (i * angle_increment)};
    readings.push_back(filtered_sensor_data);
  }
  else{
    if ( r >= msg.range_min && r<= msg.range_max) {
        FilteredSensorData filtered_sensor_data =  {.radius = r, .angle = min_angle + (i * angle_increment)};
        readings.push_back(filtered_sensor_data);
    }
    /*Not sure if max_angle - (i * angle_increment) or min_angle + (i * angle_increment)*/
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
    //    ROS_WARN("ABORT CANT CONVERT MAP POS TO OG INDEX in isOccupied");
       return false;
    }
    return (getCell(x_idx ,y_idx) > 50) ;  
}


bool isValidPos(double wx, double wy){

    double offset = 0.1;
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
double raycast(const geometry_msgs::Pose& pose, double angle_offset_deg, geometry_msgs::Point& hit_point, double max_range) {
    double robot_yaw = tf::getYaw(pose.orientation);
    double absolute_angle_deg = robot_yaw * 180.0 / M_PI + angle_offset_deg;

    // Optional: round or discretize angle to match lookup table keys
    int angle_key = static_cast<int>(std::round(absolute_angle_deg));

    // Lookup cos and sin
    auto it = ray_direction_lookup.find(angle_key);
    if (it == ray_direction_lookup.end()) {
        // fallback if key not found
        double angle_rad = robot_yaw + angle_offset_deg * M_PI / 180.0;
        it = ray_direction_lookup.emplace(angle_key, std::make_pair(cos(angle_rad), sin(angle_rad))).first;
    }
    // Lookup Table for sin and cos
    double dx = it->second.first;
    double dy = it->second.second;

    double x = pose.position.x;
    double y = pose.position.y;
    double step = 0.1;

    for (double r = 0.0; r < max_range; r += step) {
        double rx = x + r * dx;
        double ry = y + r * dy;
        int mx, my;
        if (!worldToMap(rx, ry, mx, my)) break;
        if (getCell(mx, my) > 50) {
            hit_point.x = rx;
            hit_point.y = ry;
            hit_point.z = 0.0;
            return r;
        }
    }

    // No hit
    hit_point.x = x + max_range * dx;
    hit_point.y = y + max_range * dy;
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
    std::uniform_real_distribution<> offset(-0.2, 0.2);  // within 0.8m cell
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
double uniformJitter(double min_value, double max_value) {
    static std::default_random_engine generator(std::random_device{}());
    std::uniform_real_distribution<double> distribution(min_value, max_value);
    return distribution(generator);
}

Eigen::MatrixXf resampleParticles(Eigen::MatrixXf &particles_local, bool jitterState) {

    auto start = std::chrono::high_resolution_clock::now();

    Eigen::MatrixXf new_particles(4, particles_local.cols());
    double max_injection ;
    double aplha_slow ;
    double aplha_fast ;

    // ROS_INFO("Resampling..");
    int N = particles_local.cols();
    double total_weight = computeWeight(particles_local, latest_scan);  // Compute weight of all particles in one go. Return totalweight
    double weight_avg = total_weight/N;
    // ROS_INFO("weight_avg is : %f", weight_avg);


    if(jitterState){
        max_injection = 200;
        aplha_slow = 0.05;
        aplha_fast = 0.5;
    }
    else{
        max_injection = 50;
        aplha_slow = 0.02;
        aplha_fast = 2;
    }

    
    // publishRay(marker_pub);
    /*Adaptive Injection*/
    adaptiveInjection.weight_slow =  adaptiveInjection.weight_slow + aplha_slow * (weight_avg - adaptiveInjection.weight_slow); // alpha 0.01 slow
    adaptiveInjection.weight_fast =  adaptiveInjection.weight_fast + aplha_fast * (weight_avg - adaptiveInjection.weight_fast); // alpha 0.8 fast
    // ROS_INFO("slow is : %f", adaptiveInjection.weight_slow);
    // ROS_INFO("fast is : %f", adaptiveInjection.weight_fast);

    double p_inject = std::max(0.0, 1.0 - (adaptiveInjection.weight_fast / adaptiveInjection.weight_slow));
    // ROS_INFO("P_inject is : %f", p_inject);

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
   
        if(r < p_inject && injected_particles_num < max_injection){ 

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
            // Not Confident
            double jitter_theta,jitter_x,jitter_y;
            if(jitterState){

                jitter_x = uniformJitter(-0.05,0.05);
                jitter_y = uniformJitter(-0.05,0.05);
                jitter_theta = particles(2, idx) + uniformJitter(-M_PI/12, M_PI/12);
                
            // Confident no jitter
            }else{           

                jitter_x = uniformJitter(-0.01,0.01);
                jitter_y = uniformJitter(-0.01,0.01);
                jitter_theta = particles(2, idx); 
            }
            new_particles(0,i) = particles_local(0,idx) + jitter_x; // x + noise ("Jitter") 0-0.05 m noise 0.0001
            new_particles(1,i) = particles_local(1,idx) + jitter_y; // y + noise ("Jitter") 0-0.05 m noise 0.0001
            new_particles(2, i) = atan2(sin(jitter_theta), cos(jitter_theta));
            new_particles(3,i) = 1.0 / N; // new weight uniformly
            // ROS_INFO("With Jitter -> x: %f,y: %f, theta: %f, weight: %f", new_particles(0, i), new_particles(1, i), new_particles(2, i) * 180.0 / M_PI, new_particles(3, i));
        }
     
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;  // milliseconds
    // ROS_INFO("resample took %f ms", elapsed.count());
    ROS_INFO("New Injected Particles: %d", injected_particles_num );
    return new_particles;
}

void publishParticles(const Eigen::MatrixXf &particles, ros::Publisher &publisher) {
    // ROS_INFO("Publishing");
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
    auto start_time = std::chrono::high_resolution_clock::now();

    double sigma = 0.1;           // sensor noise
    double max_laser_range = 1.0; // 1 meter
    double prob = 0;
    double totalWeight = 0;
    double laser_offset = 0.1;
    geometry_msgs::Point end;
    geometry_msgs::Pose pos;
    int N = particles_local.cols();
    std::vector<FilteredSensorData> laser_data = filterAngles(filterLaserReadings(real_scan), -120.00, 120.00);
    for (int j = 0; j < N; ++j) {
        
    /* 1 Beam is 0.352 degree
       Angle min : -135 degree  
       Angle max :  120 degree */
            prob = 0;


            pos.position.x = particles_local(0, j) + laser_offset * cos((particles_local(2,j)));
            pos.position.y = particles_local(1, j) + laser_offset * sin((particles_local(2,j)));
            pos.orientation = tf::createQuaternionMsgFromYaw(particles_local(2,j));  // In quaternion

            if(isValidPos(particles_local(0, j),particles_local(1, j))){
                // ROS_INFO("Setting Particle Weight to 0 in computeWeight");  
                for (int i = 0; i < laser_data.size(); i += 20) // skip every 20 beams for speed
                {   
                    // For RayCast   
                    double angle = -(laser_data[i].angle) * 180.0 / M_PI; // convert to degree
                    
                    double expected_distance = raycast(pos, angle, end, max_laser_range) ; // Laser Offset
                    
                    double observed_distance = laser_data[i].radius;
                    // ROS_INFO("Expected_distance: %f, Observed_distance: %f, Degree: %f", expected_distance, observed_distance, angle);
                    /*Visualize the ray*/
                    // rays.points.push_back(pos.position);
                    // rays.points.push_back(end);
                    double diff = fabs(observed_distance - expected_distance);
                    // 1. Normal distribution
                    // double gauss = std::exp(-(diff * diff) / (2 * sigma * sigma) ) / (sigma * std::sqrt(2.0f * M_PI));
                    prob += w_hit * exp_gauss.get(diff);
                    // 2. Uniform distribution - There will always be random noise because we filtered all of the invalid sensor data(In case it is invalud the uniform random will be zero)
                    // prob += w_max * 1 / max_laser_range; // 1/Max_range where Max_range is 1 meter
                    // 3. Random distribution 
                    prob += w_rand * ((std::abs(observed_distance - max_laser_range) < 0.01) ? 1.0 : 0.0);        
                }   
            }     
             
            particles_local(3, j) = prob ; // Laser Offset
            // ROS_INFO("x: %f,y: %f, theta: %f, weight: %f", particles_local(0, j), particles_local(1, j), particles_local(2, j) * 180.0 / M_PI, particles_local(3, j));
            totalWeight += particles_local(3, j);
    }
    // The prob will be later normalized for resampling
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end_time - start_time;  // milliseconds
    ROS_INFO("computeWeight took %f ms", elapsed.count());
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
RobotPosition estimateWeightedPose(const Eigen::MatrixXf& particles) {
    Eigen::VectorXf weights = particles.row(3);
    float weight_sum = weights.sum();
    Eigen::VectorXf w = weights / weight_sum;

    Eigen::VectorXf x_vals = particles.row(0);
    Eigen::VectorXf y_vals = particles.row(1);
    Eigen::VectorXf theta_vals = particles.row(2);

    float x_mean = (w.array() * x_vals.array()).sum();
    float y_mean = (w.array() * y_vals.array()).sum();

    float sin_sum = (w.array() * theta_vals.array().sin()).sum();
    float cos_sum = (w.array() * theta_vals.array().cos()).sum();
    float theta_mean = std::atan2(sin_sum, cos_sum);
    RobotPosition pos = {.x = x_mean, .y = y_mean , .theta = theta_mean};

    return pos;
}

void kMeansClustering(const Eigen::MatrixXf &particles, int K, int max_iters, std::vector<int> &assignments,std::vector<std::pair<float, float>> &centers)
{
    int N = particles.cols();
    assignments.resize(N);

    // Seed random for centroid initialization
    std::srand(static_cast<unsigned>(std::time(nullptr)));

    // Initialize cluster centers randomly from particles
    centers.clear();
    for (int k = 0; k < K; ++k) {
        int idx = std::rand() % N;
        centers.push_back({particles(0, idx), particles(1, idx)});
    }

    for (int iter = 0; iter < max_iters; ++iter) {
        bool assignments_changed = false;

        // Assignment step: assign each particle to closest centroid
        for (int i = 0; i < N; ++i) {
            float x = particles(0, i);
            float y = particles(1, i);
            float min_dist = std::numeric_limits<float>::max();
            int best_cluster = -1;
            for (int k = 0; k < K; ++k) {
                float dx = x - centers[k].first;
                float dy = y - centers[k].second;
                float dist = dx*dx + dy*dy;  // squared distance
                if (dist < min_dist) {
                    min_dist = dist;
                    best_cluster = k;
                }
            }
            if (assignments[i] != best_cluster) {
                assignments[i] = best_cluster;
                assignments_changed = true;
            }
        }

        if (!assignments_changed) {
            // No change in assignments, converged
            break;
        }

        // Update step: recompute centers
        std::vector<int> counts(K, 0);
        std::vector<std::pair<float, float>> new_centers(K, {0.0f, 0.0f});

        for (int i = 0; i < N; ++i) {
            int c = assignments[i];
            new_centers[c].first += particles(0, i);
            new_centers[c].second += particles(1, i);
            counts[c]++;
        }
        for (int k = 0; k < K; ++k) {
            if (counts[k] > 0) {
                new_centers[k].first /= counts[k];
                new_centers[k].second /= counts[k];
            } else {
                // If cluster lost all points, reinitialize randomly
                int idx = std::rand() % N;
                new_centers[k] = {particles(0, idx), particles(1, idx)};
            }
        }
        centers = new_centers;
    }
}
int countParticlesNearCluster(const Eigen::MatrixXf& particles, float x_cluster, float y_cluster, float radius) {
    int N = particles.cols();
    int count = 0;
    float radius_sq = radius * radius;

    for (int i = 0; i < N; ++i) {
        float dx = particles(0, i) - x_cluster;
        float dy = particles(1, i) - y_cluster;
        float dist_sq = dx * dx + dy * dy;

        if (dist_sq <= radius_sq) {
            ++count;
        }
    }
    return count;
}

double isLocalizationLost_densitiy_cluster(Eigen::MatrixXf& particles, double cluster_distance , double cluster_ratio_threshold ) {

    auto start_time = std::chrono::high_resolution_clock::now();
    int K = 3;              // Number of clusters you want
    int max_iters = 20;     // Max iterations for k-means to converge
    std::vector<int> assignments;
    std::vector<std::pair<float,float>> centers;

    kMeansClustering(particles, K, max_iters, assignments, centers);

    // Now you have:
    // - `assignments[i]`: cluster index of particle i
    // - `centers[k]`: centroid of cluster k

    // You can compute cluster weights by summing particle weights per cluster:
    std::vector<double> cluster_weights(K, 0.0);
    for (int i = 0; i < particles.cols(); ++i) {
        int c = assignments[i];
        cluster_weights[c] += particles(3, i); // assuming weight stored at row 3
    }

    // Pick the cluster with max weight as best estimate:
    int best_cluster = 0;
    double max_weight = cluster_weights[0];
    for (int k = 1; k < K; ++k) {
        if (cluster_weights[k] > max_weight) {
            max_weight = cluster_weights[k];
            best_cluster = k;
        }
    }
    
    // The best pose estimate is the centroid of best cluster:
     double x_best_local = centers[best_cluster].first;
     double y_best_local = centers[best_cluster].second;

    // Optionally, compute average orientation of particles in best cluster:
    double sin_sum = 0.0, cos_sum = 0.0;
    for (int i = 0; i < particles.cols(); ++i) {
        if (assignments[i] == best_cluster) {
            double theta = particles(2, i);
            sin_sum += std::sin(theta);
            cos_sum += std::cos(theta);
        }
    }
    double theta_best_local = std::atan2(sin_sum, cos_sum);
    double ratio = static_cast<double>(countParticlesNearCluster(particles, x_best_local, y_best_local, 0.4)) / particles.cols();
    if(ratio > cluster_ratio_threshold){
        x_best = x_best_local;
        y_best = y_best_local;
        theta_best = theta_best_local;
    }
    else{
        x_best = -1;
        y_best = -1;
        theta_best = -1;
    }
    // Now double x_best_local, double y_best_local, theta_best is your cluster-based pose estimate
    auto end_time = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> elapsed = end_time - start_time;  // milliseconds
    ROS_WARN("Find Cluster took %f ms", elapsed);  
    return ratio ; 

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

    if(wx < 0 || wy < 0 ){
        pos.row = -1;
        pos.column = -1;
        pos.orientation = -1;
    pose_map_pub.publish(pos);

        return;
    }
    // World Pos to Map Pos
    double col_wx = (wx - 0.5 * CELL_METERS) / CELL_METERS;
    double row_wx = (wy - 0.5 * CELL_METERS) / CELL_METERS;

    int col = static_cast<int>(std::floor(col_wx + 0.5));
    int row = static_cast<int>(std::floor(row_wx + 0.5));
    double angle_in_2PI = wrapTo2Pi(angle) * 180.0 / M_PI;
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
    pos.orientation = dir;

    pose_map_pub.publish(pos);
}
void publishExactPose(double x, double y, double theta, ros::Publisher& pub) {
    
    pink_fundamentals::ExactPose exact_pose_msg;
    // if(fabs(theta) < 5) theta = 0;
    exact_pose_msg.x = x;
    exact_pose_msg.y = y;
    exact_pose_msg.theta = theta;  // In quaternion sollte radiants sein
    // exact_pose_msg.orientation = theta;

    pub.publish(exact_pose_msg);

    ROS_INFO("Published exact pose: x = %f, y = %f, theta = %f",
             x, y, theta * 180.00 / M_PI);
}



double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}
void precomputeRayDirections(double min_angle_deg, double max_angle_deg, double step_deg) {
    for (double a = min_angle_deg; a <= max_angle_deg; a += step_deg) {
        int angle_key = static_cast<int>(a * 100);  // Key with 0.01 degree precision
        double rad = a * M_PI / 180.0;
        ray_direction_lookup[angle_key] = std::make_pair(cos(rad), sin(rad));
    }
}

void executeParticleFilter(const ros::TimerEvent&){
    auto start_time = std::chrono::high_resolution_clock::now();

    // ROS_INFO("Calling Particle Filter");

    if(confident_level < confident_level_threshhold ) {
        ROS_INFO("Not Confident");
        // start -> lost
        if(!was_lost_before){
            // "When the robot is confident and has cluster_threshold of 0.1 -> The probably has a small swarm of particles 
            // that are super close to each other"
            // Confident threshold has to be decreased to compensate the decreased thresh_hold 
            // We have to be sure that it really converges
            cluster_threshold = 0.1;
            confident_level_threshhold = 0.8; 
            startWanderer(true); 
            playSong(1,playSongClient);  // Dein 'Lost'-Song
            // particles = sampleParticles(Particle_number);
            // particles = resampleParticles(particles); // Sensor model is in resampleParitlces -> computeWeight
            ROS_WARN("Robot LOST Playing 'Lost' sound");
            was_lost_before = true;
        }
    }
    else if (confident_level > confident_level_threshhold ){
        ROS_INFO("Confident");
        cluster_threshold = 0.4;
        // When the robot is confident and has cluster_threshold of 0.4 -> The probably has a big swarm that is not close to each other"
        // Confident threshold has to be increased to compensate the increased thresh_hold
        confident_level_threshhold = 0.5; 
        // lost -> Confident
        if(was_lost_before){
            startWanderer(false); 
            // confident_level = isLocalizationLost_densitiy_cluster(particles,cluster_threshold,confident_threshold);   // best pos is updated in here particles,distance_threshhold,confident to print best pose

            playSong(2,playSongClient);  // Dein 'Juhu'-Song
            ROS_WARN("Robot RECOVERED Playing celebration sound\n Current Confident %f", confident_level);  

            publishBestParticle(x_best,y_best,theta_best,particle_pub);
            publishExactPose(x_best,y_best,theta_best,exact_pose_map_pub);
            publishPosMsg(x_best,y_best,theta_best);
            was_lost_before = false;
            // publishBestParticle(x_best,y_best,theta_best,particle_pub);
            // ros::Duration(4.0).sleep();
            // timer.stop();
        }
        else{
            // Confident -> Confindent       
            if(isValidPos(x_best,y_best)){
                ROS_WARN("Confident to Confident");  
                startWanderer(false); 
                publishBestParticle(x_best,y_best,theta_best,particle_pub);
                publishExactPose(x_best,y_best,theta_best,exact_pose_map_pub);
                publishPosMsg(x_best,y_best,theta_best);

            }
        }
    }
    publishExactPose(x_best,y_best,theta_best,exact_pose_map_pub);
    publishPosMsg(x_best,y_best,theta_best);
    diffDriveModel(encoderData,current_position,previous_position);
    // Update Position of the particles
    updateParticlePos(particles);
    // Use Sensor Model to evaluate new laser data and resample
    // resample when not confident
    particles = resampleParticles(particles,was_lost_before); // Sensor model is in resampleParitlces -> computeWeight
    confident_level = isLocalizationLost_densitiy_cluster(particles,cluster_threshold,confident_level_threshhold);   // best pos is updated in here particles,distance_threshhold,confident to print best pose
    ROS_WARN("Current Confident %f", confident_level);  
    publishParticles(particles, particle_pub_resampled);
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end_time - start_time;  // milliseconds
    ROS_WARN("Particle Filter took %f ms", elapsed);  

    
}
// Generate Random particle to test Convergence criteria
Eigen::MatrixXf generateParticleCluster(int random_particle){

    Eigen::MatrixXf new_particles(4, Particle_number);
    // ROS_INFO("Resampling..");
    int N = Particle_number;
    int injected_particles_num = 0;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    std::uniform_real_distribution<> x_dist(0, 4.4);
    std::uniform_real_distribution<> y_dist(0, 4.4);
    std::uniform_real_distribution<> theta_dist(-3.14, 3.14);

    Eigen::MatrixXf cluster_particle(4, 1);

    cluster_particle(0,0) = 1.2; // x
    cluster_particle(1,0) = 4.4; // y
    cluster_particle(2,0) = -M_PI/4; // theta
    cluster_particle(3,0) = 1.0 / N; // new weight uniformly

    for (int i = 0; i < Particle_number; ++i) {
        double r = dis(gen);
        /*With dynamic Injection*/
        /*If current Particles dont perform good, we add new one*/
        if(i < random_particle){ 
            new_particles(0,i) = x_dist(gen); // x
            new_particles(1,i) = y_dist(gen); // y
            new_particles(2,i) = theta_dist(gen); // theta
            new_particles(3,i) = 1.0 / N; // new weight uniformly
            // ROS_INFO("Random No -> x: %f,y: %f, theta: %f, weight: %f", new_particles(0, i), new_particles(1,i), new_particles(2, i) * 180.0 / M_PI, new_particles(3, i));

            injected_particles_num++;
        }
        else{        

            new_particles(0,i) = cluster_particle(0,0)   + uniformJitter(-0.005,0.005); // x + noise ("Jitter") 0-0.02 m noise 0.0001
            new_particles(1,i) = cluster_particle(1,0)   + uniformJitter(-0.005,0.005); // y + noise ("Jitter") 0-0.05 m noise 0.0001
            double jitter_theta = cluster_particle(2, 0) + uniformJitter(-M_PI/12, M_PI/12);
            new_particles(2,i) = atan2(sin(jitter_theta), cos(jitter_theta));
            // new_particles(2,i) = atan2(sin(cluster_particle(2, 0)), cos(cluster_particle(2, 0)));

            new_particles(3,i) = 1.0 / N; // new weight uniformly
            // ROS_INFO("No -> x: %f,y: %f, theta: %f, weight: %f", new_particles(0, i), new_particles(1,i), new_particles(2, i) * 180.0 / M_PI, new_particles(3, i));
        }
     
    }
    ROS_INFO("New Injected Particles: %d", injected_particles_num );
    // ROS_INFO("Real -> x: %f,y: %f, theta: %f, weight: %f", cluster_particle(0, 0), cluster_particle(1,0), cluster_particle(2, 0) * 180.0 / M_PI, cluster_particle(3, 0));

    return new_particles;
}

/****************************************************************************************/
int main(int argc, char** argv) {
    ros::init(argc, argv, "monte_carlo_matrix");
    ros::NodeHandle nh;
    /*Topics*/
    ros::Subscriber scan_sub = nh.subscribe("/scan_filtered", 1, laser_callback); // mapCallback???
    ros::Subscriber map_sub = nh.subscribe("/maze_map", 1, mapCallback);
    ros::Subscriber sub_rad = nh.subscribe("sensor_packet", 1, sensorCallback);
    ros::Subscriber sub = nh.subscribe("/shutdown_driver", 1, shutdownCallback);
    /*Services*/
    diffDrive = nh.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    pid_drive = nh.serviceClient<pink_fundamentals::PID_drive>("PID_drive");
    wanderer_client = nh.serviceClient<pink_fundamentals::Wanderer>("Wanderer");
    resetEncoder = nh.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");
    align_server = nh.serviceClient<pink_fundamentals::align_call>("align");
    storeSongClient = nh.serviceClient<create_fundamentals::StoreSong>("store_song");
    playSongClient = nh.serviceClient<create_fundamentals::PlaySong>("play_song");
    /*Advertises*/
    particle_pub = nh.advertise<geometry_msgs::PoseArray>("localised_pos", 1);
    particle_pub_resampled = nh.advertise<geometry_msgs::PoseArray>("resampled_particles", 1);
    part_pub_zero = nh.advertise<geometry_msgs::PoseArray>("zero_weight_particles", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseArray>("test_pose", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("raycast", 1);
    pose_map_pub = nh.advertise<pink_fundamentals::Pose>("pose", 1);
    exact_pose_map_pub = nh.advertise<pink_fundamentals::ExactPose>("exact_pose", 1);
    pos_target_pub = nh.advertise<geometry_msgs::PoseArray>("target_pose", 1);


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
    noiseParam = {.alpha_1 = 0.001, .alpha_2 = 0.001, .alpha_3 = 0.0001, .alpha_4 = 0.0001};
    precomputeRayDirections(-120.0, 120.0, 0.1);  // Every 0.1 degree

    /*Start Localisation*/ 
    localisation_state = true;
    /*Test Particle Distribution*/
    ROS_INFO("First Sample finished");
    particles = sampleParticles(Particle_number);
    particles = resampleParticles(particles,resample_confident); // Sensor model is in resampleParitlces -> computeWeight
    // publishRay(marker_pub);
    publishParticles(particles, particle_pub_resampled);
    startWanderer(true);
    timer = nh.createTimer(ros::Duration(0.1), executeParticleFilter);
    
    ros::spin();
    return 0;
}



