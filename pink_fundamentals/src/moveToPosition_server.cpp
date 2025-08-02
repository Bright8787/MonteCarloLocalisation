#include <ros/ros.h>
#include <vector>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <algorithm>
#include <queue>
#include "pink_fundamentals/MoveToPosition.h"
#include "pink_fundamentals/DistanceToNode.h"
#include "pink_fundamentals/Grid.h"
#include "pink_fundamentals/Row.h"
#include "pink_fundamentals/Cell.h"
#include "pink_fundamentals/Pose.h"
#include "sensor_msgs/LaserScan.h" 
#include "create_fundamentals/DiffDrive.h"
#include "pink_fundamentals/ExactPose.h"
#include "pink_fundamentals/PID_drive.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "package/songs_lib.hpp"
#include "create_fundamentals/PlaySong.h"
#include "create_fundamentals/StoreSong.h"
pink_fundamentals::PID_drive apf_srv;
pink_fundamentals::PID_drive srv_pid_drive;
ros::ServiceClient srv_apf_drive;
ros::ServiceClient pid_drive;
ros::Publisher marker_pub;
ros::ServiceClient storeSongClient;
ros::ServiceClient playSongClient;

const std::vector<std::pair<int, std::pair<int, int>>> all_directions = {
    {0, {0, 1}},   // RIGHT
    {1, {-1, 0}},  // TOP
    {2, {0, -1}},  // LEFT
    {3, {1, 0}}    // BOTTOM
};
struct Position{
    double x;
    double y;
    double theta;
};
/*Struct data structure*/
typedef struct
{
    double x, y;
} Point2D;
/*Potential Field*/
double total_repel_x;
double attract_mag;
double total_repel_y;
double fx_attract;
double fy_attract;
double attract_gain = 50; // was 90, I try 900
double repel_gain = 0.00;  // η = 0.1
double influence_radius = 0.6; // 0.8 meter
double fx_total;
double fy_total;

bool executePlan(std::vector<std::pair<int, int>> &plan );
/*service*/
ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive srv;

/*Wall*/
std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> graph;
std::vector<std::
vector<std::vector<int>>> walls;

/*Data*/
std::vector<float> laser_data;
std::vector<Point2D> cartesian;
// Data receive flag
bool map_info = false;
bool isPoseReady = false;
bool isExactReady = false;
bool laserReceived = false;
// Wheel Speed
double  motor_omega_left = 0;
double  motor_omega_right = 0;
// pose topic 
int localised_cell_x = 0;  
int localised_cell_y = 0;  
int localised_cell_orientation = 0;

Position localised = {.x = 0, .y = 0, .theta = 0};
double localised_x = 0;  
double localised_y = 0;  
double localised_orientation = 0;
//Laser Callback
double angle_min = 0;
double angle_increment = 0;

void publishHeadingErrorArrow(const Position& world_pos, double error_theta, ros::Publisher& marker_pub) {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "map"; // or "odom" depending on your frame
    arrow.header.stamp = ros::Time::now();
    arrow.ns = "heading_error";
    arrow.id = 0;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    // Start point = robot position
    geometry_msgs::Point p_start, p_end;
    p_start.x = world_pos.x;
    p_start.y = world_pos.y;
    p_start.z = 0;

    // End point = in direction of heading error (rotate from robot orientation)
    double arrow_length = 0.5;
    double display_theta = world_pos.theta + error_theta;
    p_end.x = p_start.x + arrow_length * cos(display_theta);
    p_end.y = p_start.y + arrow_length * sin(display_theta);
    p_end.z = 0;

    arrow.points.push_back(p_start);
    arrow.points.push_back(p_end);

    // Style
    arrow.scale.x = 0.05; // shaft diameter
    arrow.scale.y = 0.1;  // head diameter
    arrow.scale.z = 0.1;

    arrow.color.r = 1.0;
    arrow.color.g = 0.0;
    arrow.color.b = 0.0;
    arrow.color.a = 1.0;

    marker_pub.publish(arrow);
}
double calculate_distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}
double distanceToPoint(const Point2D &point){
    return std::sqrt(point.x * point.x + point.y * point.y);
}
void printWalls() {
    for (size_t i = 0; i < walls.size(); ++i) {
        ROS_INFO("Row %lu:", i);
        for (size_t j = 0; j < walls[i].size(); ++j) {
            std::ostringstream oss;
            oss << "  Cell " << j << ": [";

            for (size_t k = 0; k < walls[i][j].size(); ++k) {
                oss << walls[i][j][k];
                if (k < walls[i][j].size() - 1) oss << ", ";
            }
            oss << "]";
            ROS_INFO("%s", oss.str().c_str());
        }
    }


}
void printGraph(const std::map<std::pair<int, int>, std::vector<std::pair<int, int>>>& graph) {
    for (const auto& entry : graph) {
        const auto& key = entry.first;
        const auto& neighbors = entry.second;

        std::string line = "Node (" + std::to_string(key.first) + ", " + std::to_string(key.second) + "): ";
        for (const auto& n : neighbors) {
            line += "(" + std::to_string(n.first) + ", " + std::to_string(n.second) + ") ";
        }

        ROS_INFO("%s", line.c_str());
    }
}
// aus publish map genommen
void createWall(const pink_fundamentals::Grid::ConstPtr& msg) {
    walls.clear();
    for (const auto& row : msg->rows) {
        std::vector<std::vector<int>> row_cells;
        for (const auto& cell : row.cells) {
            std::vector<int> cell_walls(cell.walls.begin(), cell.walls.end());
            row_cells.push_back(cell_walls);
        }
        walls.push_back(row_cells);
    }
    // printWalls();
    // ROS_INFO("Grid data converted to wall format (%ld rows)", walls.size());
}

std::vector<std::pair<int, int>> bfs(
    const std::map<std::pair<int, int>, std::vector<std::pair<int, int>>>& graph,
    const std::pair<int, int>& start,
    const std::pair<int, int>& goal)
{
    std::queue<std::pair<int, int>> q;
    std::map<std::pair<int, int>, std::pair<int, int>> parent;
    std::set<std::pair<int, int>> visited;

    q.push(start);
    visited.insert(start);

    while (!q.empty()) {
        auto current = q.front();
        q.pop();

        if (current == goal) {
            // Reconstruct path
            std::vector<std::pair<int, int>> path;
            for (auto node = goal; node != start; node = parent[node]) {
                // swap x and y because if the map
                path.emplace_back(node.first, node.second);  // (y, x) instead of (x, y)

                // path.emplace_back(node.second, node.first);  // (y, x) instead of (x, y)
            }
            // path.emplace_back(start.second, start.first);  // (y, x) instead of (x, y)
            path.emplace_back(start.first, start.second);  // (y, x) instead of (x, y)

            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const auto& neighbor : graph.at(current)) {
            if (visited.count(neighbor) == 0) {
                visited.insert(neighbor);
                parent[neighbor] = current;
                q.push(neighbor);
            }
        }
    }

    // No path found
    return {};
}
void printNeighbors(const  std::vector<std::pair<int, int>> & neighbors) {
    std::ostringstream oss;
    oss << "Neighbors: ";
    for (const auto& p : neighbors) {
        oss << "(" << p.first << ", " << p.second << ") ";
    }
    ROS_INFO_STREAM(oss.str());
}
void buildGraphFromWallArray(const std::vector<std::vector<std::vector<int>>>& wall_data) {
    graph.clear();
    int rows = wall_data.size();
    int cols = rows > 0 ? wall_data[0].size() : 0;

    std::vector<std::pair<int, int>> neighbors;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            std::set<int> wall_set(wall_data[i][j].begin(), wall_data[i][j].end());
            std::pair<int, int> current = {i, j};
            // Code is the direction
            for (const auto& [code, offset] : all_directions) {
                if (wall_set.count(code) == 0) { // if direction is not in the wall list
                    int ni = i + offset.first;
                    int nj = j + offset.second;
                    if (ni >= 0 && ni < rows && nj >= 0 && nj < cols) {
                        neighbors.emplace_back(ni, nj);
                    }
                }
            }
            graph[current] = neighbors;
            neighbors.clear();
        }
    }
    // printNeighbors(neighbors);
    // printGraph(graph);
    // ROS_INFO("Graph built from wall array (%d x %d)", rows, cols);
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

std::vector<std::pair<int, int>> simplifyPath(
    const std::vector<std::pair<int, int>>& path,
    const std::map<std::pair<int, int>, std::vector<std::pair<int, int>>>& graph)
{
    if (path.size() <= 2) return path; // cant simplify

    std::vector<std::pair<int, int>> simplified;
    simplified.push_back(path[0]);

    size_t anchor = 0;
    for (size_t i = 2; i < path.size(); ++i) {
        const auto& from = path[anchor];
        const auto& to = path[i];
        const auto& neighbors = graph.at(from);

        if (std::find(neighbors.begin(), neighbors.end(), to) == neighbors.end()) {
            simplified.push_back(path[i - 1]);
            anchor = i - 1;
        }
    }

    // Always include the goal
    simplified.push_back(path.back());

    return simplified;
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
void drive_motor(double speed_left, double speed_right)
{
    srv.request.left = speed_left;
    srv.request.right = speed_right;
    diffDrive.call(srv);
}
/***************************************Callback function ************************************************/

bool distanceToNodeCallback(pink_fundamentals::DistanceToNode::Request &req, pink_fundamentals::DistanceToNode::Response &res) {

    auto start = std::make_pair(req.row_start, req.column_start);
    auto goal = std::make_pair(req.row_end, req.column_end);
    auto path = bfs(graph,start,goal);
    ROS_INFO("#distanceToNodeCallback# Service is called");
    res.success = true;
    res.distance = path.size();
    ROS_INFO("#distanceToNodeCallback# Distance is %d", res.distance);

    return true;
}

bool moveToPositionCallback(pink_fundamentals::MoveToPosition::Request &req,
    pink_fundamentals::MoveToPosition::Response &res) {
    // swap row and column
    if(isPoseReady){

        auto start = std::make_pair(localised_cell_x, localised_cell_y);
        // ROS_INFO("row: %d, column: %d",localised_cell_x,localised_cell_y);
        auto goal = std::make_pair(req.row, req.column);
        // ROS_INFO("row: %d, column: %d",req.row,req.column);
        auto path = bfs(graph,start,goal);
        // Under Test
        // path = simplifyPath(path,graph);
        // printGraph(graph);

        // Get the Path
        for (auto& p : path) {
            if(path.back() != p ){
            ROS_INFO("(%d,%d) -> ", p.first, p.second );
            }
            else{
            ROS_INFO("(%d,%d)", p.first, p.second );
            }
        }  
        bool isSuccess = executePlan(path);      

        if(isSuccess){
            res.success = true;
            return true;
        }  
    }
    res.success = false;
    return true;;

}
void mapCallback(const pink_fundamentals::Grid::ConstPtr& msg) {
    createWall(msg);
    map_info = true;
    buildGraphFromWallArray(walls);
}
void poseCallback(const pink_fundamentals::Pose::ConstPtr& msg) {
    // Confition if it is localised or not
    if(msg->row < 0 || msg->column < 0){
        isPoseReady = false;
    }
    else{

        isPoseReady = true; 
        localised_cell_x = msg->row;
        localised_cell_y = msg->column;
        localised_cell_orientation = msg->orientation;     

    }
}
void exactPoseCallback(const pink_fundamentals::ExactPose::ConstPtr& msg) {
    // Confition if it is localised or not
    localised.x = msg->x;
    localised.y = msg->y;
    localised.theta = msg->theta;    
    if(msg->x < 0 || msg->y < 0){
        isExactReady = false;
        ROS_WARN("In valid Pose Received Pos: (%f, %f)", localised.x,localised.y);
    }
    else{
        isExactReady = true; 
        ROS_INFO("Received Pos: (%f, %f)", localised.x,localised.y) ;
    }
}
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    /*Set Angle increment global parameter*/
    angle_increment = msg->angle_increment;
    angle_min = msg->angle_min;
    /*Filter out nan and out of range values*/
    laserReceived = true;
    cartesian = polarToCartesian(msg);
}

void avf(double x, double y, double degree, double speed){

    apf_srv.request.x      = x;
    apf_srv.request.y      = y;
    apf_srv.request.degree = degree;
    apf_srv.request.speed  = speed;
    srv_apf_drive.call(apf_srv);

}
void drive_pid(double x, double y, double degree, double speed){

    srv_pid_drive.request.x = x;
    srv_pid_drive.request.y = y;
    srv_pid_drive.request.degree = degree;
    srv_pid_drive.request.speed = speed;
    
    pid_drive.call(srv_pid_drive);

}

/*************************************** APF Global ************************************************/
void calculateAPF(Position world_pos, Position target_pos, double base_speed)
{   
    // Compute distance to goal

    double dx = target_pos.x - world_pos.x;
    double dy = target_pos.y - world_pos.y;
    double theta_goal_global = atan2(dy, dx);  // angle to goal in global frame
    ROS_INFO("Theta global is %f",theta_goal_global * 180 / M_PI);

    double local_x =  cos(world_pos.theta) * dx + sin(world_pos.theta) * dy;
    double local_y = -sin(world_pos.theta) * dx + cos(world_pos.theta) * dy;
    ROS_INFO("local_x: %f, local_f: %f",local_x,local_y);
    
    double current_distance_to_goal = calculate_distance(world_pos.x, world_pos.y, target_pos.x, target_pos.y);

    // Fade factor for repulsion: from 1 (far) to 0 (at goal)
    double fade_exponent = 1.0;
    double fade_factor   = pow(current_distance_to_goal / influence_radius, fade_exponent);
    fade_factor = std::min(1.0, std::max(0.0, fade_factor));
    // Potential field: repulsion
    total_repel_x = total_repel_y = 0.0;
    for (int i = 0; i < cartesian.size(); i++) {
        double dist = distanceToPoint(cartesian[i]);
        if (dist < influence_radius && dist > 0.005) {
            // Apply fade to repulsive gain
            double eff_repel = repel_gain * fade_factor;

            // Repulsion magnitude term
            double scale = (eff_repel * (1.0/dist - 1.0/influence_radius)) / (dist * dist) * 0.5; //*0.5 hinugefügt

            double ux = cartesian[i].x / dist; 
            double uy = cartesian[i].y / dist;

            total_repel_x += scale * ux;
            total_repel_y += scale * uy;
        }
    }
    // Attractive force 
    fx_attract = attract_gain * (local_x); // TODO Transform
    fy_attract = attract_gain * (local_y); // TODO Transform
    attract_mag = std::sqrt(fx_attract * fx_attract + fy_attract * fy_attract);
    // ROS_INFO("attract mag is x: %f y: %f",fx_attract, fy_attract);
    // ROS_INFO("attract mag is %f",attract_mag);
    // Total field
    fx_total = fx_attract + total_repel_x;
    fy_total = fy_attract + total_repel_y;
    // ROS_INFO("Repel mag is x: %f y: %f",total_repel_x, total_repel_y);
    // ROS_INFO("Total  mag is x: %f y: %f",fx_total, fy_total);
    // ROS_INFO("Distance to goal: %f", current_distance_to_goal);
    // ROS_INFO("Fade factor: %f", fade_factor);
    
    // Visualize
    // publishInfluenceRadius(radius_pub, influence_radius);
    // publishVectorArrow(fx_attract, fy_attract, "Attract", 1, field_pub, red_color);
    // publishVectorArrow(total_repel_x, total_repel_y, "Repel", 1, field_pub, blue_color);
    // publishVectorArrow(fx_total, fy_total, "Total", 1, field_pub, green_color);

    // Compute angular error
    double heading = atan2(fy_total, fx_total);
    // ROS_INFO("Heading Error: %f deg", heading * 180.0 / M_PI);
    
    double raw_error = heading;
    // double raw_error = heading;   
    // Normalize to [-pi,pi]
    raw_error = atan2(sin(raw_error), cos(raw_error));
    ROS_INFO("Heading Error: %f deg", raw_error * 180.0 / M_PI);
    double error_angular = atan2(sin(raw_error), cos(raw_error));

    // ROS_INFO("Heading: %f deg", heading * 180.0 / M_PI);
    // ROS_INFO("Current theta: %f deg", world_pos.theta * 180.0 / M_PI);
    // ROS_INFO("Error angular: %f deg", error_angular * 180.0 / M_PI);

    // Simple P-control on heading
    double omega_command = 4 * error_angular;  // Gain of 1 was the Problem. AVF has too little influence

    // Motor commands
    motor_omega_left  = base_speed + omega_command;
    motor_omega_right = base_speed - omega_command;


}
void goToTargetGlobal(Position world_pos, Position target_pos, double base_speed) {

    double dx = target_pos.x - world_pos.x;
    double dy = target_pos.y - world_pos.y;
    double distance = sqrt(dx*dx + dy*dy);

    double heading_to_goal = atan2(dy, dx);
    double error_theta = heading_to_goal - world_pos.theta;
    error_theta = atan2(sin(error_theta), cos(error_theta)); // Normalize
    ROS_INFO("Error Theta : %f", error_theta * 180/M_PI);
    publishHeadingErrorArrow(world_pos, error_theta, marker_pub);

    double omega = 7.0 * error_theta; // Adjust gain
    ROS_INFO("Omega : %f", omega);

    double linear = base_speed * std::max(0.0, cos(error_theta));

    // Stop if close to goal
    if (distance < 0.1) {
        linear = 0;
        omega = 0;
    }

    motor_omega_left  = linear + omega;
    motor_omega_right = linear - omega;
}

bool isValidPos(double x, double y){

    if(x < 0 || y < 0){
        return false;
    }
 
    return true;
}
bool executePlan(std::vector<std::pair<int, int>> &plan ){
    const double CELL_METERS = 0.8;
    const double offset = 0.00; // offset is in raycast now
    const double base_speed = 10.0;
    Position target_world; 
    ros::Rate wait_data(10);
    // Center of the cell
    for (auto& p : plan) {
        // In our map x and y are swapped!
        ros::spinOnce(); // Update localised_pos
        target_world.x = p.second * CELL_METERS + 0.5 * CELL_METERS + offset;
        target_world.y = p.first * CELL_METERS + 0.5 * CELL_METERS + offset;
        ROS_INFO("localised: ( %f,%f ) | target_world: ( %f,%f )", localised.x,localised.y, target_world.x,target_world.y);

        if (isValidPos(localised.x, localised.y)) {
            if (plan.front() != p) { // Only move if we're not already at this plan point
                ROS_INFO("Executing (%d,%d)", p.first, p.second);      
                // Keep moving toward target until you're close enough
                while (fabs(localised.x - target_world.x) > 0.2 || fabs(localised.y - target_world.y) > 0.2) {
                    ros::spinOnce(); // Update `localised` pose
                    if (!isValidPos(localised.x, localised.y)){
                        ROS_WARN("Invalid Position received");
                        drive_motor(0, 0);
                        return false;
                    } 
                    ROS_INFO("localised: ( %f,%f ) | target_world: ( %f,%f )",
                             localised.x, localised.y, target_world.x, target_world.y);
                        
                    goToTargetGlobal(localised, target_world, base_speed);
                    drive_motor(motor_omega_left, motor_omega_right);
                    wait_data.sleep(); // Usually ros::Rate
                }

                drive_motor(0, 0); // Stop once target reached
            }
        } else {
            return false; // Invalid start position
        }
    } 

    
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_map_planner_node");
    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);
    ros::ServiceServer moveToPosition_pub = nh.advertiseService("/move_to_position", moveToPositionCallback);
    ros::ServiceServer distanceToNode_pub = nh.advertiseService("/distance_to_node", distanceToNodeCallback);

    ros::Subscriber pose_sub = nh.subscribe("/pose", 1, poseCallback);
    /*Topics Subscriber*/
    ros::Subscriber sub_laser = nh.subscribe("/scan_filtered", 1, laserCallback);
    ros::Subscriber exactPos_sub = nh.subscribe("/exact_pose", 1, exactPoseCallback);
    diffDrive    = nh.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    srv_apf_drive = nh.serviceClient<pink_fundamentals::PID_drive>("Potential_drive");
    pid_drive = nh.serviceClient<pink_fundamentals::PID_drive>("PID_drive");
    marker_pub = nh.advertise<visualization_msgs::Marker>("heading_error_marker", 10);
    storeSongClient = nh.serviceClient<create_fundamentals::StoreSong>("store_song");
    playSongClient = nh.serviceClient<create_fundamentals::PlaySong>("play_song");

    uploadSongs(storeSongClient);

    ros::Rate rate(1.0); // 1 Hz

    while(!map_info){
        ros::spinOnce();
        ROS_INFO("Planner node started. Waiting for map and service calls.");
        rate.sleep();
    }

    while(!laserReceived){
        ros::spinOnce();
        ROS_INFO("Waiting for laser data.");
        rate.sleep();
    }


    ros::spin();
    return 0;
}
