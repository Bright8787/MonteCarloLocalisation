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


enum Direction { UP, DOWN, LEFT, RIGHT };
std::vector<Direction> plan = {}; // this is for testingadvance



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
double attract_mag;
double total_repel_y;
double fx_attract;
double fy_attract;
double attract_gain = 90; // was 90, I try 900
double repel_gain = 0.001;  // η = 0.1
double influence_radius = 0.4; // 0.8 meter
double fx_total;
double fy_total;
double cell_size = 0.8;

double current_encoderLeft = 0;
double current_encoderRight = 0;

double previous_encoderLeft = 0;
double previous_encoderRight = 0;

bool encoder_received = false; // <- This is the key

double motor_omega_right = 0;
double motor_omega_left = 0;
double error_angular = 0;

void drive_motor(double speed_left, double speed_right)
{
    
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


void calculateAVF(double base_speed)
{   

    // Potential field: repulsion
    total_repel_x = total_repel_y = 0.0;
    for (int i = 0; i < cartesian.size(); i++) {
        double dist = distanceToPoint(cartesian[i]);
        if (dist < influence_radius && dist > 0.005) {
            // Apply fade to repulsive gain
            double eff_repel = repel_gain;

            // Repulsion magnitude term
            double scale = (eff_repel * (1.0/dist - 1.0/influence_radius)) / (dist * dist) * 0.5; //*0.5 hinugefügt

            // Direction away from obstacle
            //double ux = -cartesian[i].x / dist; // TODO: positive??
            //double uy = -cartesian[i].y / dist;
            
            double ux = cartesian[i].x / dist; // TODO: positive??
            double uy = cartesian[i].y / dist;

            total_repel_x += scale * ux;
            total_repel_y += scale * uy;
        }
    }

    fx_total =  total_repel_x;
    fy_total =  total_repel_y;
    // Visualize
    publishInfluenceRadius(radius_pub, influence_radius);
    publishVectorArrow(total_repel_x, total_repel_y, "Repel", 1, field_pub, blue_color);

    // Compute angular error
    double heading = atan2(fy_total, fx_total);
    
    double raw_error = heading;
    // double raw_error = heading;   
    // Normalize to [-pi,pi]
    raw_error = atan2(sin(raw_error), cos(raw_error));
    ROS_INFO("Heading Error: %f deg", raw_error * 180.0 / M_PI);
    // Enforce minimum turn of 10° when correcting heading
    // const double min_turn = 30.0 * M_PI / 180.0;
    // if (fabs(raw_error) > min_turn) {
    //     raw_error += (raw_error > 0 ? min_turn : -min_turn);
    // }
    // error_angular = raw_error;
    // ROS_INFO("Error angular (with 10° offset): %f deg", error_angular * 180.0 / M_PI);
    // Normalize to [-pi,pi]
    error_angular = atan2(sin(raw_error), cos(raw_error));

    ROS_INFO("Heading: %f deg", heading * 180.0 / M_PI);
    ROS_INFO("Error angular: %f deg", error_angular * 180.0 / M_PI);

    // Simple P-control on heading
    double omega_command = 5 * error_angular;  // Gain of 1 was the Problem. AVF has too little influence

    // Motor commands
    motor_omega_left  = base_speed + omega_command;
    motor_omega_right = base_speed - omega_command;

}

void avf_wanderer(double base_speed)
{
    /*New Point New Start*/

    ROS_INFO("Drive AVF is called");
    while (ros::ok())
    {
        ros::spinOnce();
        calculateAVF(base_speed);
        // ROS_INFO("motor_speed_left: %f and motor_speed_left: %f\n",motor_speed_left,motor_speed_right);
        drive_motor(motor_omega_left, motor_omega_right);
        ros::Duration(0.1).sleep();
    }
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
    ros::init(argc, argv, "AVF_Server");
    ros::NodeHandle n;

    // Subscribers
    ros::Subscriber sub_rad   = n.subscribe("sensor_packet",   1, sensorCallback);
    ros::Subscriber sub_laser = n.subscribe("scan_filtered",   1, laserCallback);

    // Service clients & publishers
    diffDrive    = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    resetEncoder = n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");
    field_pub    = n.advertise<visualization_msgs::Marker>("wanderer_avf",           10);
    radius_pub   = n.advertise<visualization_msgs::Marker>("influence_radius_marker", 1);

    // Service server (unused in test)
    // ros::ServiceServer service_drive = n.advertiseService("avf_wanderer", avf_wanderer);
    resetEncoder.call(srv_encoder);

    // Marker colors
    red_color.r   = 1.0; red_color.a   = 1.0;
    green_color.g = 1.0; green_color.a = 1.0;
    blue_color.b  = 1.0; blue_color.a  = 1.0;

    ROS_INFO("Start Now");
    ROS_INFO("Waiting for encoder & laser data...");

    // wait until we have both encoder and laser
    while ((!encoder_received || !laserReceived) && ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("Encoder & Laser data received. Running LOCAL TEST…");
    // wait until we have both encoder and laser
    avf_wanderer(8);

    ros::spin();
    return 0;
    // ────────────────────────────────────────────────────────────────
}





