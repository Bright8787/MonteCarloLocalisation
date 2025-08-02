#include "ros/console.h"
#include "ros/init.h"
#include "ros/ros.h"
#include "wanderer.h"

#include <boost/type_traits/alignment_of.hpp>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <iterator>
#include <vector>
#include <optional>

#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/MarkerArray.h>

#include "package/PID_lib.hpp"

#include "pink_fundamentals/align_call.h"
// #include "pink_fundamentals/PID_rotate.h"

/*Struct data structure*/
typedef enum
{
    LEFT,
    RIGHT,
    FRONT,
    UNKNOWN
} WallSide;

typedef struct
{
    double x, y;
} Point2D;

typedef struct
{
    double x, y;
} distance2D;

struct Line
{
    double a, b, c; // Line in form: ax + by + c = 0
    std::vector<Point2D> inliers;
};
struct Wall
{
    Line line;
    double angle_deg;
    WallSide side; // "left", "right", or "front", "unknown"
};
/*Data*/
std::vector<Line> lines;
std::vector<Line> closest_intersect_line;
std::vector<float> laser_data;
std::vector<Line> locked_lines; 
std::vector<Point2D> intersect_points;
bool isTurning = false;
bool laserReceived = false;
double angle_min = 0;
double angle_increment = 0;
bool isIntersect = false;
bool should_intersect = false; // for intersecting
bool align_lock = false;

create_fundamentals::DiffDrive srv;
ros::ServiceClient pid_drive;
ros::ServiceClient diffDrive;
distance2D deltaDistance;
/*Service client*/

/*Publisher*/
ros::Publisher line_array_pub;
ros::Publisher intersect_point_pub;
ros::Publisher perpendicular_line_array_pub;

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

void drive_distance(double distance, double speed)
{
    double time = 0;
    ROS_INFO("Driving distance %f",distance);
    srv.request.left = speed;
    srv.request.right = speed;
    diffDrive.call(srv);

    time = calculate_drive_time(speed, distance);
    ROS_INFO("time: %f", time);
    ros::Duration(time).sleep();
    srv.request.left = 0;
    srv.request.right = 0;
    diffDrive.call(srv);
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

std::vector<Point2D> polarToCartesian(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    std::vector<Point2D> points;
    double angle = msg->angle_min;

    for (size_t i = 0; i < msg->ranges.size(); ++i, angle += msg->angle_increment)
    {
        float r = msg->ranges[i];
        if (std::isnan(r) || std::isinf(r) || r < msg->range_min || r > msg->range_max)
            continue;   // just skip, but still advance angle

        points.push_back({
            r * std::cos(angle),
            r * std::sin(angle)
        });
    }
    return points;
}
// std::vector<Point2D> polarToCartesian(const sensor_msgs::LaserScan::ConstPtr &msg)
// {
//     laser_data = filterLaserReadings(msg);
//     std::vector<Point2D> points;
//     double start_angle = angle_min;
//     for (size_t i = 0; i < laser_data.size(); ++i)
//     {
//         start_angle = start_angle + angle_increment; // Convert degrees to radians
//         //Filter out data points
//         if(laser_data[i] != -999){
//           points.push_back({laser_data[i] * cos(start_angle), laser_data[i] * sin(start_angle)});
//         }
//     }
//     return points;
// }


bool intersectLines(const Line &line1, const Line &line2, Point2D &intersect_point)
{
    double det = line1.a * line2.b - line2.a * line1.b;
    /*Check if 2 lines are parallel*/
    if (det == 0 )
    {
        // Lines are parallel or coincident — no unique intersection
        return false;
    }
    /*Find Intersect point - Cramer Algorithm*/
    /*Eliminate far away intersectipn point*/
    /*Convert 2 Lines to vector to find dot product*/
    double numerator = std::abs(line1.a * line2.b - line2.a * line1.b);
    double denominator = line1.a * line2.a + line1.b * line2.b;

    if (denominator == 0) return false; // Prevent division by zero

    double angle_rad = std::atan(numerator / denominator);     // returns angle between 2 lines in radians
    double angle_deg = angle_rad * 180.0 / M_PI; // convert to degrees

    /*Filter out non-edge intersection point*/
    // if(angle_deg > 110 || angle_deg < 70) return false;

    double x = (line1.b * line2.c - line2.b * line1.c) / det;
    double y = (line2.a * line1.c - line1.a * line2.c) / det;

    intersect_point.x = x;
    intersect_point.y = y;


    return true;
}

double distanceLaserToLine(const Line &line)
{
    double distance = std::abs(line.c) / std::sqrt(line.a * line.a + line.b * line.b);
    return distance;
}

double distanceToPoint(const Point2D &point){
    return std::sqrt(point.x * point.x + point.y * point.y);
}
distance2D distanceToSquareCenter(const Line &line1, const Line &line2)
{
    // double offset = 2.0;
    double offset = 0.0;
    /*Rotate robot to a */
    distance2D distance;
    /*Get the needed distance to the center of square*/
    double distace_wall1 = ((distanceLaserToLine(line1)*100.00) + offset);
    double distace_wall2 = ((distanceLaserToLine(line2)*100.00)  + offset);
    /*Robot has to face each wall direction before performing movement*/
    distance.x = distace_wall1;
    distance.y = distace_wall2;
    // ROS_INFO("The distance to the wall 1 is %f", distace_wall1);

    return distance;
}

double normalizeAngleDeg(double angle_deg)
{
    while (angle_deg > 180.0)
        angle_deg -= 360.0;
    while (angle_deg < -180.0)
        angle_deg += 360.0;
    return angle_deg;
}

double getLineAngleDeg(const Line &line1, const Line &line2)
{
    // Direction vector of the line (tangent): (-b, a)
    double numerator = std::abs(line1.a * line2.b - line2.a * line1.b);
    double denominator = line1.a * line2.a + line1.b * line2.b;

    if (denominator == 0) return 90.0; // Prevent division by zero and return 90.0 (dot product = 0 -> Perpendicular line)

    double angle_rad = std::atan(numerator / denominator);     // returns angle between 2 lines in radians
    double angle_deg = angle_rad * 180.0 / M_PI; // convert to degrees
    ROS_INFO("Angle is: %f", angle_deg);
    // if(angle_deg > 90 ) angle_deg -= 90;
    // if(angle_deg < 90 ) angle_deg -= 90; 
    return angle_deg;
}
Point2D perpendicularPointToOrigin(const Line& line)
{
    double denom = line.a * line.a + line.b * line.b;
    Point2D foot;
    foot.x = -line.a * line.c / denom;
    foot.y = -line.b * line.c / denom;
    return foot;
}
/*Ransac*/
std::vector<Line> RANSAC(std::vector<Point2D> points, double distance_threshold, int max_iterations, int min_inliers)
{
    std::vector<Line> detected_lines;

    while (true)
    {
        Line best_line;
        int best_inlier_count = 0;

        for (int i = 0; i < max_iterations; ++i)
        {
            Point2D p1 = points[rand() % points.size()];
            Point2D p2 = points[rand() % points.size()];
            if (p1.x == p2.x && p1.y == p2.y)
                continue;

            double a = p2.y - p1.y;
            double b = p1.x - p2.x;
            double c = -(a * p1.x + b * p1.y);

            double norm = std::sqrt(a * a + b * b);
            a /= norm;
            b /= norm;
            c /= norm;

            std::vector<Point2D> inliers;
            for (const auto &p : points)
            {
                double dist = std::fabs(a * p.x + b * p.y + c);
                if (dist < distance_threshold)
                {
                    inliers.push_back(p);
                }
            }

            if (inliers.size() > best_inlier_count && inliers.size() > min_inliers)
            {
                best_inlier_count = inliers.size();
                best_line = {a, b, c, inliers};
            }
        }

        if (best_inlier_count < min_inliers)
        {
            break;
        }

        detected_lines.push_back(best_line);

        std::vector<Point2D> remaining;
        for (const auto &p : points)
        {
            bool is_inlier = false;
            for (const auto &in : best_line.inliers)
            {
                if (std::fabs(p.x - in.x) < 1e-4 && std::fabs(p.y - in.y) < 1e-4)
                {
                    is_inlier = true;
                    break;
                }
            }
            if (!is_inlier)
            {
                remaining.push_back(p);
            }
        }

        points = remaining;

        if ((points.size() < (size_t)min_inliers) || detected_lines.size() == 2)
            break;
    }

    return detected_lines;
}



void publishRansacLines(const std::vector<Line> &lines, ros::Publisher &pub)
{
    visualization_msgs::MarkerArray marker_array;

    for (size_t i = 0; i < lines.size(); ++i)
    {
        const auto &line = lines[i];
        if (line.inliers.empty())
            continue;

        Point2D p1 = line.inliers.front();
        Point2D p2 = line.inliers.back();

        visualization_msgs::Marker marker;
        marker.header.frame_id = "laser"; // Adjust as needed
        marker.header.stamp = ros::Time::now();
        marker.ns = "ransac";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;

        marker.lifetime = ros::Duration(0.3);

        geometry_msgs::Point pt1, pt2;

        pt1.x = p1.x;
        pt1.y = p1.y;
        pt1.z = 0;
        pt2.x = p2.x;
        pt2.y = p2.y;
        pt2.z = 0;

        marker.points.push_back(pt1);
        marker.points.push_back(pt2);

        marker.scale.x = 0.03;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker_array.markers.push_back(marker);
    }

    pub.publish(marker_array);
}
/*Intersect*/
void publishIntersectPoints(const std::vector<Point2D>& points, ros::Publisher& pub) {

    for (size_t i = 0; i < points.size(); ++i) {
        visualization_msgs::Marker point_marker;
        point_marker.header.frame_id = "laser";
        point_marker.header.stamp = ros::Time::now();
        point_marker.ns = "Intersect_Point";
        point_marker.id = i;  // Unique ID for each point!

        point_marker.type = visualization_msgs::Marker::SPHERE;
        point_marker.action = visualization_msgs::Marker::ADD;

        point_marker.pose.position.x = points[i].x;
        point_marker.pose.position.y = points[i].y;
        point_marker.pose.position.z = 0.0;
        point_marker.pose.orientation.w = 1.0;

        point_marker.scale.x = 0.05;
        point_marker.scale.y = 0.05;
        point_marker.scale.z = 0.05;

        point_marker.color.r = 1.0f;
        point_marker.color.g = 1.0f;
        point_marker.color.b = 0.0f;
        point_marker.color.a = 1.0;

        point_marker.lifetime = ros::Duration();

        pub.publish(point_marker);
    }

 

}
/*Perpendicular Line to wall*/
void publishPerpendicularLinesToWall(const std::vector<Line> &lines, ros::Publisher &pub)
{
    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < lines.size(); ++i)
    {
        const auto &line = lines[i];
        if (line.inliers.empty())
            continue;

        Point2D perpendicular_point = perpendicularPointToOrigin(lines[i]);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "laser"; // Adjust as needed
        marker.header.stamp = ros::Time::now();
        marker.ns = "Perpendicular Line";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0.3);

        geometry_msgs::Point pt1, pt2;
        pt1.x = 0;
        pt1.y = 0;
        pt1.z = 0;
        pt2.x = perpendicular_point.x;
        pt2.y = perpendicular_point.y;
        pt2.z = 0;

        marker.points.push_back(pt1);
        marker.points.push_back(pt2);

        marker.scale.x = 0.03;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker_array.markers.push_back(marker);
    }

    pub.publish(marker_array);
}




// void rotate(double degrees, PID_lib::Direction dirs)
// {
//         srv_rotate.request.dir = static_cast<int>(dirs);
//         srv_rotate.request.degree = degrees;
//         pid_rotate.call(srv_rotate);
// }


// void drive(double distance)
// {
//     srv_drive.request.distance_left = distance;
//     srv_drive.request.distance_right = distance;
//     pid_drive.call(srv_drive);
// }

double alignRobotToTheWall(const Line& line){

    Line x_axis = {.a=1, .b=0 , .c=0};

    double angleToWall = getLineAngleDeg(line,x_axis);

    return fabs(angleToWall);
}

WallSide classifyWallSide(const Line& wall_line)
{
    Point2D pointOnWall = perpendicularPointToOrigin(wall_line);
    // ROS_INFO("Point on the Wall is: x:%f, y:%f",pointOnWall.x,pointOnWall.y);
    if(pointOnWall.y < 0){
        return RIGHT;
    }
    else if(pointOnWall.y > 0){
        return LEFT;
    }
    else{
        return FRONT;
    }
    return UNKNOWN;
}


void faceWall(const Line& wall) {
    double angle = alignRobotToTheWall(wall);
    WallSide side = classifyWallSide(wall);

    if (angle < 1.0) {
        ROS_INFO("Robot is already aligned to the wall.");
        return;
    }
    PID_lib::Direction dir;
    if (side == RIGHT) {
        dir = PID_lib::Direction::ROTATE_RIGHT;
    } else if (side == LEFT) {
        dir = PID_lib::Direction::ROTATE_LEFT;
    } else if (side == FRONT) {
        ROS_INFO("Wall is in front. No rotation needed.");
        return;
    } else {
        ROS_WARN("Unknown wall side. Skipping rotation.");
        return;
    }
    double angle_rad = angle * M_PI / 180.0;
    ROS_INFO("Rotating %.2f degrees (%s) to face the wall...", angle, (side == RIGHT ? "RIGHT" : "LEFT"));

    rotate_degree(angle,5,dir);
    // rotate(angle,dir);


}


bool isPointNear(const Point2D& a, const Point2D& b, double threshold) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    return distance <= threshold;
}


// returns true if P is between A and B (inclusive) on the segment AB
bool isPointBetween(const Point2D &A, const Point2D &B, const Point2D &P) {
    // axis‐aligned bounding box quick reject
    if ((P.x < std::min(A.x, B.x) - 1e-3) || (P.x > std::max(A.x, B.x) + 1e-3) ||
        (P.y < std::min(A.y, B.y) - 1e-3) || (P.y > std::max(A.y, B.y) + 1e-3))
        return false;
    // check colinearity via cross‐product
    double cross = (P.y - A.y)*(B.x - A.x) - (P.x - A.x)*(B.y - A.y);
    return std::abs(cross) < 1e-2;  // tolerance for numerical noise
}
// bool isPointBetween(const Point2D& A, const Point2D& B, const Point2D& P, double tolerance = 100.0) {
//
//
//
//     // Check if P is colinear with A and B using cross product
//     double cross = (P.y - A.y) * (B.x - A.x) - (P.x - A.x) * (B.y - A.y);
//     if (std::abs(cross) > tolerance)
//         return false; // Not colinear
//
//     // Check if P is within the bounding box of A and B
//     double dot = (P.x - A.x) * (B.x - A.x) + (P.y - A.y) * (B.y - A.y);
//     if (dot < 0)
//         return false;
//
//     double squaredLengthAB = (B.x - A.x) * (B.x - A.x) + (B.y - A.y) * (B.y - A.y);
//     if (dot > squaredLengthAB)
//         return false;
//
//     return true;
// }
//
//
// bool checkIfIntersectInLines(const Line &l1, const Line &l2, const Point2D &p) {
//     // 1) require enough inliers
//     static const size_t MIN_INLIERS = 40;
//     if (l1.inliers.size() < MIN_INLIERS || l2.inliers.size() < MIN_INLIERS)
//         return false;
//
//     // 2) require each segment be at least 0.3m long
//     Point2D A1 = l1.inliers.front(), B1 = l1.inliers.back();
//     Point2D A2 = l2.inliers.front(), B2 = l2.inliers.back();
//     double len1 = std::hypot(A1.x - B1.x, A1.y - B1.y);
//     double len2 = std::hypot(A2.x - B2.x, A2.y - B2.y);
//     if (len1 < 0.3 || len2 < 0.3)
//         return false;
//
//     // 3) must lie between both endpoints
//     if (!isPointBetween(A1,B1,p) || !isPointBetween(A2,B2,p))
//         return false;
//
//     // 4) must be roughly right‐angle
//     double angle = getLineAngleDeg(l1,l2);
//     if (std::abs(angle - 90.0) > 10.0)
//         return false;
//
//     // 5) must be within [0.15m,2.0m] of the robot
//     double d = distanceToPoint(p);
//     if (d < 0.15 || d > 2.0)
//         return false;
//
//     ROS_INFO("Corner OK @ (%.2f,%.2f) angle=%.1f° dist=%.2fm", p.x,p.y, angle, d);
//     return true;
// }
bool checkIfIntersectInLines(const Line &line1, const Line &line2, const Point2D &intersectPoint){

    Point2D p1_line1 = line1.inliers.front();
    Point2D p2_line1 = line1.inliers.back();

    Point2D p1_line2 = line2.inliers.front();
    Point2D p2_line2 = line2.inliers.back();

    // if( isPointBetween(p1_line1,p2_line1,intersectPoint) && isPointBetween(p1_line2,p2_line1,p2_line2)){
    //     ROS_INFO("Intersect Point is in line");
    //     return true;
    // }
    // else{
    //     ROS_INFO("Intersect Point is not in line");
    //     return false;  
    // } 

    double threshhold_p = 0.5;
    double angle = getLineAngleDeg(line1, line2);
    if( (isPointNear(p1_line1,intersectPoint,threshhold_p) || isPointNear(p2_line1,intersectPoint,threshhold_p)) && 
        (isPointNear(p1_line2,intersectPoint,threshhold_p) || isPointNear(p2_line2,intersectPoint,threshhold_p)) &&
        (std::abs(angle - 90.0) <= 10)){
        ROS_INFO("Intersect Point is in line");
        return true;
    }
    else{
        ROS_INFO("Intersect Point is not in line");
        return false;  
    } 

}
/*Call back funciton*/
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    std::vector<Wall> walls;

    intersect_points.clear();
    Point2D intersect_point;
    /*Set Angle increment global parameter*/
    angle_increment = msg->angle_increment;
    angle_min = msg->angle_min;
    /*Filter out nan and out of range values*/
    laserReceived = true;
    // ROS_INFO("Valid readings count: %zu", laser_data.size());
    /*Convert polar data points to cartesian*/
    std::vector<Point2D> cartesian = polarToCartesian(msg);
   
    /*Apply Ransac to find linear line and publish it*/
    lines = RANSAC(cartesian,0.05 , 100, 50);
    publishRansacLines(lines, line_array_pub);
     isIntersect = false;
    if (lines.size() >= 2 && locked_lines.empty()) {
        isIntersect = intersectLines(lines[0], lines[1], intersect_point);
        //  if (isIntersect && checkIfIntersectInLines(lines[0], lines[1], intersect_point))
        if (isIntersect) {
            intersect_points.push_back(intersect_point);
            publishIntersectPoints(intersect_points,intersect_point_pub);
            locked_lines = lines;
        }
        else {
            isIntersect = false;
        }
    }
    /*Publish it all intersecting lines*/
    if(intersect_points.size() != 0){
        publishIntersectPoints(intersect_points,intersect_point_pub);
    }
    /*Find Perpendicular Line and publish it*/
    publishPerpendicularLinesToWall(lines,perpendicular_line_array_pub);

}

void driveFromWall(const Line& line1,const Line& line2){
    /*Wall 1*/
    faceWall(line1);
    ROS_INFO("Drive from the wall");
    distance2D distances_to_walls = distanceToSquareCenter(line1 , line2);
    if(distances_to_walls.x < 40){
        ROS_INFO("distance is smaler: x:%f",(distances_to_walls.x/100 -0.40 + 0.05 ));
        // drive((distances_to_walls.x/100 -0.40 + 0.05));
        drive_distance(((distances_to_walls.x/100) -0.40 + 0.1),-5);

    }
    else{   
        ROS_INFO("distance is bigger: x:%f",(distances_to_walls.x/100 -0.40 + 0.05 ));
        // drive(-1*(distances_to_walls.x/100 -0.40 + 0.05 ));
        drive_distance(((distances_to_walls.x/100) -0.40 + 0.1),5);
    }

     /*Wall 2*/
    WallSide side = classifyWallSide(line2);

    PID_lib::Direction dir;
    if (side == RIGHT) {
        dir = PID_lib::Direction::ROTATE_RIGHT;
    } else if (side == LEFT) {
        dir = PID_lib::Direction::ROTATE_LEFT;
    } else if (side == FRONT) {
        ROS_INFO("Wall is in front. No rotation needed.");
    } else {
        ROS_WARN("Unknown wall side. Skipping rotation.");
    }
    rotate_degree(90.0,5,dir);
    // rotate(90.0,dir);

    if(distances_to_walls.y < 40){
        ROS_INFO("distance y:%f",distances_to_walls.y/100-0.4+ 0.05);
        // drive((distances_to_walls.y/100 -0.40 + 0.05 ));
        drive_distance((distances_to_walls.y/100 -0.40+ 0.08 ),-5);

    }
    else{
        ROS_INFO("distance y:%f",distances_to_walls.y/100-0.4+ 0.05);
        drive_distance((distances_to_walls.y/100 -0.40+ 0.08 ),5);
    }
}
bool aligned = false;

bool align(pink_fundamentals::align_call::Request &req , pink_fundamentals::align_call::Response &res)
{
    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        if(locked_lines.size() >= 2) {
            try {
                Line line1 = locked_lines[0];
                Line line2 = locked_lines[1];
                driveFromWall(line1, line2);
                locked_lines.clear();
                intersect_points.clear();   
                isIntersect = false;
                res.success = true;
                return true;
            } catch (const std::exception &e) {
                ROS_ERROR("Exception during alignment: %s", e.what());
                res.success = false;
                return false;
            }
        }

        else if (wandererThreshold(0.18)) {
            ROS_INFO("enters this shit");
            avoidObstacle(0.18);
            rate.sleep();
            continue;
        }

        ROS_INFO("No intersection yet. Clear path — moving forward to explore.");
        srv.request.left = 5;
        srv.request.right = 5;
        diffDrive.call(srv);

        rate.sleep();
    }

    res.success = false;
    return false;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "align_server");
    ros::NodeHandle n;
    ros::Subscriber sub_laser = n.subscribe("scan_filtered", 1, laserCallback);
    ros::ServiceServer service_align = n.advertiseService("align_call", align);

    line_array_pub = n.advertise<visualization_msgs::MarkerArray>("ransac_lines", 1);
    perpendicular_line_array_pub = n.advertise<visualization_msgs::MarkerArray>("perpenducular_line", 1);
    intersect_point_pub = n.advertise<visualization_msgs::Marker>("intersect_line", 1);
    
    diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    ROS_INFO("Start Now");

    ros::spin();
    if (aligned) {
    return 0;
    }
}





