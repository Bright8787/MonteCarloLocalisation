// #include <ros/ros.h>
// #include <visualization_msgs/Marker.h>
// #include <geometry_msgs/Point.h>
// #include <pink_fundamentals/Grid.h>

// ros::Publisher marker_pub;

// void drawWalls(const pink_fundamentals::Grid::ConstPtr& msg) {


//         ros::Rate rate(10);
//         /*For the maze*/
//         visualization_msgs::Marker marker;
//         marker.header.frame_id = "map";
//         marker.header.stamp = ros::Time::now();
//         marker.ns = "maze";
//         marker.id = 0;
//         marker.type = visualization_msgs::Marker::LINE_LIST;
//         marker.action = visualization_msgs::Marker::ADD;
//         marker.scale.x = 0.05;
//         marker.color.r = 1.0;
//         marker.color.a = 1.0;

//         const double CELL_SIZE = 0.8;
//         /*{Key, {{segment1}, segment2}}*/
//         std::map<int, std::pair<std::pair<double, double>, std::pair<double, double>>> wall_dirs = {
//             {0, {{CELL_SIZE, 0}, {CELL_SIZE, CELL_SIZE}}}, // RIGHT
//             {1, {{0, 0}, {CELL_SIZE, 0}}},               // TOP
//             {2, {{0, CELL_SIZE}, {0, 0}}},                // LEFT
//             {3, {{CELL_SIZE, CELL_SIZE}, {0, CELL_SIZE}}} // BOTTOM
//         };
        
//         int num_rows = msg->rows.size();
//         double origin_y = 0; // top of the grid at y = 0
        
//         for (int i = 0; i < num_rows; ++i) {
//             const auto& row = msg->rows[i];
//             for (int j = 0; j < row.cells.size(); ++j) {
//                 const auto& cell = row.cells[j];
        
//                 // Flip vertically by inverting the row index
//                 double x_base = j * CELL_SIZE;
//                 double y_base = i* CELL_SIZE;

        
//                 for (auto wall : cell.walls) {
//                     auto offset1 = wall_dirs[wall].first;
//                     auto offset2 = wall_dirs[wall].second;
        
//                     geometry_msgs::Point p1, p2;
//                     p1.x = x_base + offset1.first;
//                     p1.y = y_base + offset1.second;
//                     p1.z = 0;
        
//                     p2.x = x_base + offset2.first;
//                     p2.y = y_base + offset2.second;
//                     p2.z = 0;
        
//                     marker.points.push_back(p1);
//                     marker.points.push_back(p2);
//                 }
//             }
//         }
        
//         marker_pub.publish(marker);

//         /*For Global X and Y Axis*/
//         visualization_msgs::Marker marker_x, marker_y;

//         // Common header and namespace
//         marker_x.header.frame_id = "map";
//         marker_x.header.stamp = ros::Time::now();
//         marker_x.ns = "axis";
//         marker_x.type = visualization_msgs::Marker::ARROW;
//         marker_x.action = visualization_msgs::Marker::ADD;

//         marker_y = marker_x; // copy all settings

//         // Scale of arrows (shaft length, diameter, head diameter)
//         marker_x.scale.x = 0.1; // arrow length
//         marker_x.scale.y = 0.1; 
//         marker_x.scale.z = 0.1;
//         marker_y.scale = marker_x.scale;

//         // Colors
//         marker_x.color.r = 1.0; marker_x.color.g = 0; marker_x.color.b = 0; marker_x.color.a = 1.0; // red
//         marker_y.color.r = 0; marker_y.color.g = 1.0; marker_y.color.b = 0; marker_y.color.a = 1.0; // green

//         // Points for X arrow: from (0,0,0) to (0.8,0,0)
//         geometry_msgs::Point start, end;
//         start.x = 0; start.y = 0; start.z = 0;

//         end.x = 0.4; end.y = 0; end.z = 0;

//         marker_x.points.clear();
//         marker_x.points.push_back(start);
//         marker_x.points.push_back(end);

//         // Points for Y arrow: from (0,0,0) to (0,0.8,0)
//         geometry_msgs::Point end_y;
//         end_y.x = 0; end_y.y = 0.4; end_y.z = 0;

//         marker_y.points.clear();
//         marker_y.points.push_back(start);
//         marker_y.points.push_back(end_y);

//         // Different IDs so both markers show
//         marker_x.id = 0;
//         marker_y.id = 1;

//         marker_pub.publish(marker_x);
//         marker_pub.publish(marker_y);

//     }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "maze_visualizer");
//     ros::NodeHandle nh;

//     marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
//     ros::Subscriber sub = nh.subscribe("map", 1, drawWalls);

//     ros::spin();
//     return 0;
// }

/********************************************************************************************/
// #include <ros/ros.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <vector>

// // Constants
// const int CELL_SIZE = 8;              // pixels per cell side (3x3)
// const double CELL_METERS = 0.8;       // size of one logical grid cell in meters (0.8m x 0.8m)
// const double RESOLUTION = CELL_METERS / CELL_SIZE;  // meters per pixel (0.8 / 3 = ~0.2667)
// const int WALL_OCCUPIED = 100;
// const int FREE_SPACE = 0;

// // The wall layout: for each cell, vector of wall sides: 0=right,1=top,2=left,3=bottom
// std::vector<std::vector<std::vector<int>>> walls = {
//     { {1, 2, 0}, {2, 1}, {1, 0, 3} },
//     { {2, 3},    {3},    {1, 0}    },
//     { {2, 1, 3}, {1, 3}, {3, 0}    }
// };

// nav_msgs::OccupancyGrid createOccupancyGrid(const std::vector<std::vector<std::vector<int>>>& walls)
// {
//     int grid_height = walls.size();        // rows
//     int grid_width = walls[0].size();      // cols
    
//     int map_width = grid_height * CELL_SIZE;    // width in pixels now corresponds to original grid_height
//     int map_height = grid_width * CELL_SIZE;    // height in pixels corresponds to original grid_width
    
//     std::vector<int8_t> data(map_width * map_height, FREE_SPACE);
    
//     for (int cell_y = 0; cell_y < grid_height; ++cell_y)
//     {
//         for (int cell_x = 0; cell_x < grid_width; ++cell_x)
//         {
//             // Swapped:
//             int x = cell_y * CELL_SIZE;  // pixel x is now vertical index times cell size
//             int y = cell_x * CELL_SIZE;  // pixel y is now horizontal index times cell size
    
//             const auto& cellWalls = walls[cell_y][cell_x];
    
//             for (int w : cellWalls)
//             {
//                 if (w == 1)  // top wall
//                 {
//                     // horizontal line along "y" axis (currently vertical pixel)
//                     for (int dx = 0; dx < CELL_SIZE; ++dx)
//                     {
//                         int idx = (x) * map_width + (y + dx);
//                         if (idx >= 0 && idx < (int)data.size())
//                             data[idx] = WALL_OCCUPIED;
//                     }
//                 }
//                 else if (w == 2)  // left wall
//                 {
//                     // vertical line along "x" axis (currently horizontal pixel)
//                     for (int dy = 0; dy < CELL_SIZE; ++dy)
//                     {
//                         int idx = (x + dy) * map_width + y;
//                         if (idx >= 0 && idx < (int)data.size())
//                             data[idx] = WALL_OCCUPIED;
//                     }
//                 }
//                 else if (w == 0)  // right wall
//                 {
//                     if (cell_x == grid_width - 1)
//                     {
//                         for (int dy = 0; dy < CELL_SIZE; ++dy)
//                         {
//                             int idx = (x + dy) * map_width + (y + CELL_SIZE - 1);
//                             if (idx >= 0 && idx < (int)data.size())
//                                 data[idx] = WALL_OCCUPIED;
//                         }
//                     }
//                 }
//                 else if (w == 3)  // bottom wall
//                 {
//                     if (cell_y == grid_height - 1)
//                     {
//                         for (int dx = 0; dx < CELL_SIZE; ++dx)
//                         {
//                             int idx = (x + CELL_SIZE - 1) * map_width + (y + dx);
//                             if (idx >= 0 && idx < (int)data.size())
//                                 data[idx] = WALL_OCCUPIED;
//                         }
//                     }
//                 }
//             }
//         }
//     }
    
//     // Update OccupancyGrid info accordingly
//     nav_msgs::OccupancyGrid grid;
//     grid.header.stamp = ros::Time::now();
//     grid.header.frame_id = "map";
    
//     grid.info.resolution = RESOLUTION;
//     grid.info.width = map_width;   // now grid_height * CELL_SIZE
//     grid.info.height = map_height; // now grid_width * CELL_SIZE
    
//     // Origin stays the same as needed
//     grid.info.origin.position.x = 0.0;
//     grid.info.origin.position.y = 0.0;
//     grid.info.origin.position.z = 0.0;
    
//     grid.info.origin.orientation.x = 0.0;
//     grid.info.origin.orientation.y = 0.0;
//     grid.info.origin.orientation.z = 0.0;
//     grid.info.origin.orientation.w = 1.0;
    
//     grid.data = data;
    

  

//     return grid;
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "maze_map_publisher");
//     ros::NodeHandle nh;
//     ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("/maze_map", 1);

//     ros::Rate rate(1.0); // 1 Hz

//     while (ros::ok())
//     {
//         nav_msgs::OccupancyGrid grid = createOccupancyGrid(walls);
//         pub.publish(grid);
//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }



#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pink_fundamentals/Grid.h>
#include <vector>

// Constants
const int CELL_SIZE = 8;              // pixels per cell side (8x8)
const double CELL_METERS = 0.8;       // size of one logical grid cell in meters (0.8m x 0.8m)
const double RESOLUTION = CELL_METERS / CELL_SIZE;  // meters per pixel (0.8 / 8 = 0.1)
const int WALL_OCCUPIED = 100;
const int FREE_SPACE = 0;
bool map_info = false;
ros::Publisher pub;
// The wall layout: for each cell, vector of wall sides: 0=right,1=top,2=left,3=bottom
// std::vector<std::vector<std::vector<int>>> walls = {
//     { {1, 2, 0}, {2, 1}, {1, 0, 3} },
//     { {2, 3},    {3},    {1, 0}    },
//     { {2, 1, 3}, {1, 3}, {3, 0}    }
// };
std::vector<std::vector<std::vector<int>>> walls ;
// std::vector<std::vector<std::vector<int>>> walls;
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

nav_msgs::OccupancyGrid createOccupancyGrid(const std::vector<std::vector<std::vector<int>>>& walls)
{
    // How to avoid drawing duplicate walls:
    // For each cell, only draw the top and left walls.

    // For the right wall, draw it only if the cell is on the last column (because the rightmost cells don’t have a neighboring cell to the right).

    // For the bottom wall, draw it only if the cell is on the last row.
    printWalls();
    
    int grid_height = walls.size(); // Rows of the map
    int grid_width = 0;

    int column_size = 0;
    // Compute max row width(max column size) to determine map width
    for (const auto& row : walls) {
        if ((int)row.size() > grid_width){
            grid_width = row.size();
            column_size = grid_width;
            ROS_INFO("Map width is: %d", grid_width);
        }
    }
    /*For the last cell both row and column*/
    /*Translate to cell - 1 cell needs map_width and map_height element*/
    int map_width = grid_width * CELL_SIZE + 1;  //  Array Element
    int map_height = grid_height * CELL_SIZE + 1; // Array Element

    std::vector<int8_t> data(map_width * map_height, FREE_SPACE); // FREE_SPACE = 0
    
    for (int cell_y = 0; cell_y < grid_height; ++cell_y) {
        const auto& row = walls[cell_y];
        int row_width = row.size();
    
        for (int cell_x = 0; cell_x < row_width; ++cell_x) {

            int x = cell_y * CELL_SIZE;
            int y = cell_x * CELL_SIZE;
    
            const auto& cellWalls = row[cell_x];
            
            for (int w : cellWalls) {
                if (w == 1) {  // top wall
                    for (int dx = 0; dx <= CELL_SIZE; ++dx) {
                        int idx = x * map_width + (y + dx);
                        if (idx >= 0 && idx < (int)data.size())
                            data[idx] = WALL_OCCUPIED;
                    }
                }
                else if (w == 2) {  // left wall
                    for (int dy = 0; dy <= CELL_SIZE; ++dy) {
                        int idx = (x + dy) * map_width + y;
                        if (idx >= 0 && idx < (int)data.size())
                            data[idx] = WALL_OCCUPIED;
                    }
                }
                else if (w == 0) {  // right wall
                    if (cell_x == row_width - 1)
                    {
                    for (int dy = 0; dy < CELL_SIZE; ++dy) {
                        int idx = (x + dy) * map_width + (y + CELL_SIZE );
                        if (idx >= 0 && idx < (int)data.size())
                            data[idx] = WALL_OCCUPIED;
                    }
                }
                }
                else if (w == 3) {  // bottom wall

                    if (cell_y == grid_height - 1) 
                    {
                    for (int dx = 0; dx < CELL_SIZE; ++dx) {
                        int idx = (x + CELL_SIZE ) * map_width + (y + dx + 1);
                        if (idx >= 0 && idx < (int)data.size())
                            data[idx] = WALL_OCCUPIED;
                    }
                    }
                    /*Not the last row*/
                    if (cell_y + 1 < grid_height) {  // Check next row exists
                        if ( !(cell_x < walls[cell_y + 1].size())) {  // Check column exists in next row
                            for (int dx = 0; dx < CELL_SIZE; ++dx) {
                                int idx = (x + CELL_SIZE ) * map_width + (y + dx + 1);
                                if (idx >= 0 && idx < (int)data.size())
                                    data[idx] = WALL_OCCUPIED;
                            }
                        }
                    }
                }
                
            }

        }
        /*Set excess space to occupy space*/
        while(row_width < column_size){
             /*Occupy space*/
             ROS_INFO("row_width :%d, map_width: %d", row_width, map_width);
             double x = cell_y * CELL_SIZE;
             double y = row_width * CELL_SIZE;
             for (int dy = 0; dy < CELL_SIZE; ++dy) {
                 for (int dx = 0; dx <= CELL_SIZE; ++dx) {
                    int idx = (x + dy) * map_width + (y + dx);
                     if (idx >= 0 && idx < (int)data.size())
                         data[idx] = WALL_OCCUPIED;
                 }
             }
                row_width++;
        }
   
    }

    
    // Update OccupancyGrid info accordingly
    nav_msgs::OccupancyGrid grid;
    grid.header.stamp = ros::Time::now();
    grid.header.frame_id = "map";
    
    grid.info.resolution = RESOLUTION;
    grid.info.width = map_width;   // now grid_height * CELL_SIZE
    grid.info.height = map_height; // now grid_width * CELL_SIZE
    
    // Origin stays the same as needed
    grid.info.origin.position.x = 0.0;
    grid.info.origin.position.y = 0.0;
    grid.info.origin.position.z = 0.0;
    
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    
    grid.data = data;
    
    return grid;
}
void createWall(const pink_fundamentals::Grid::ConstPtr& msg) {
    
    walls.clear();  // Clear previous data
    ROS_INFO("Row size %ld", msg->rows.size());
    for (const auto& row : msg->rows)
    {
        std::vector<std::vector<int>> row_cells;

        for (const auto& cell : row.cells)
        {
            std::vector<int> cell_walls(cell.walls.begin(), cell.walls.end());
            row_cells.push_back(cell_walls);
        }

        walls.push_back(row_cells);
    }
    map_info = true;

    // printWalls();

}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "maze_map_publisher");
    ros::NodeHandle nh;
    pub = nh.advertise<nav_msgs::OccupancyGrid>("/maze_map", 1);
    ros::Subscriber sub = nh.subscribe("/map", 1, createWall);
    ros::Rate rate(1.0); // 1 Hz

    while(!map_info){
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok())
    {   
        nav_msgs::OccupancyGrid grid = createOccupancyGrid(walls);
        pub.publish(grid);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
