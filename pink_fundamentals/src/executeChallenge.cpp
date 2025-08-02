
#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include "pink_fundamentals/MoveToPosition.h"
#include "pink_fundamentals/DistanceToNode.h"
#include "pink_fundamentals/Pose.h"
#include <std_msgs/Empty.h>
#include "package/songs_lib.hpp"
#include "create_fundamentals/PlaySong.h"
#include "package/songs_lib.hpp"

ros::ServiceClient distanceToNode;
pink_fundamentals::DistanceToNode distanceToNode_srv;
ros::ServiceClient moveToPosition;
pink_fundamentals::MoveToPosition moveToPosition_srv;
ros::ServiceClient playSongClient;

ros::Publisher pub;
std::list<std::pair<int,int>>  goldPlan;
std::list<std::pair<int,int>>  pickUpPlan;

typedef struct Node{
    int row;
    int column;
}; 
typedef struct NodeTSP{
    Node node;
    std::vector<int> distance;
};
bool isLocalisedPosReady = false;
Node localised_node = {.row = -1, .column = -1};

/*Callback function*/
// Pose is in row and column format
void poseCallBack(const pink_fundamentals::Pose::ConstPtr& msg) {
    // Confition if it is localised or not
    localised_node.row    = msg->row;
    localised_node.column = msg->column;
    if(localised_node.row < 0 || localised_node.column < 0){
        isLocalisedPosReady = false;

        ROS_WARN("In valid Pose Received Pos: (%d, %d)", localised_node.row,localised_node.column);
    }
    else{
        isLocalisedPosReady = true; 
        ROS_INFO("Received Pos: (%d, %d)", localised_node.row,localised_node.column) ;
    }
}

void printNodeTSPVector(const std::vector<NodeTSP>& vec) {
    for (int i = 0; i < vec.size(); ++i) {
        std::ostringstream oss;
        oss << "Node " << i << " (" << vec[i].node.row << "," << vec[i].node.column << "): ";
        for (int d : vec[i].distance) {
            oss << d << " ";
        }
        ROS_INFO("%s", oss.str().c_str());
    }
}

/**********************************************Convert Text to Plan**********************************************/
std::list<std::pair<int,int>> getPlan( std::string content){

    if (!content.empty() && content.front() == '[') content.erase(0,1);
    if (!content.empty() && content.back() == ']') content.pop_back();

    std::list<std::pair<int,int>> plan;  

    std::stringstream ss(content);
    std::string pair_str;

    while (std::getline(ss, pair_str, ']')) { 
        size_t open_bracket = pair_str.find('[');
        if (open_bracket == std::string::npos) continue;

        std::string coord = pair_str.substr(open_bracket + 1);
        std::stringstream coord_ss(coord);
        std::string x_str, y_str;

        if (std::getline(coord_ss, x_str, ',') && std::getline(coord_ss, y_str, ',')) {
            int x = std::stoi(x_str);
            int y = std::stoi(y_str);
            plan.push_back({x,y});
        }
    }

    // Print all plan
    for (const auto& p : plan) {
        std::cout << "x: " << p.first << ", y: " << p.second << std::endl;
    }
    return plan;
}
/**********************************************Service**********************************************/

bool moveToPositionCall(int row, int column){

    moveToPosition_srv.request.row = row;
    moveToPosition_srv.request.column = column;

    if (moveToPosition.call(moveToPosition_srv)) {
        // The service call reached the server successfully
        if (moveToPosition_srv.response.success) {  // Assuming your response has a "success" field
            // ROS_WARN("Gold node is reached");
            return true;
        } else {
            // ROS_WARN("Lost on the way to the Gold node");
            return false;

        }
    } else {
        ROS_ERROR("Failed to call service.");
        return false;
    }

    return true;

}
int distanceToNodeCall(int row_start, int column_start, int row_end, int column_end){

    distanceToNode_srv.request.row_start = row_start;
    distanceToNode_srv.request.column_start = column_start;

    distanceToNode_srv.request.row_end = row_end;
    distanceToNode_srv.request.column_end = column_end;


    if (distanceToNode.call(distanceToNode_srv)) {
        // The service call reached the server successfully
        if (distanceToNode_srv.response.success) {  // Assuming your response has a "success" field
            return distanceToNode_srv.response.distance;
        } else {
            return 0;

        }
    } else {
        ROS_ERROR("#distanceToNodeCall# Failed to call service.");
        return 0;
    }

    return 0;
}
/**********************************************TSP**********************************************/

std::vector<NodeTSP> generateTSPGraph(std::list<std::pair<int,int>> goldPlan){

    std::list<std::pair<int,int>> allNode = goldPlan;
    allNode.push_front({localised_node.row,localised_node.column});
    ROS_INFO("Localised Node is (%d,%d)",localised_node.row,localised_node.column);
    // allNode.push_front({5,0});

    int N = allNode.size();
    ROS_INFO("Graph consisted of %d Nodes",N);

    std::vector<NodeTSP> graphTSP(N);

    std::vector<std::pair<int,int>> allNodeVector(allNode.begin(), allNode.end());

    for (int i = 0; i < N; ++i) {
        graphTSP[i].node = {.row = allNodeVector[i].first, .column = allNodeVector[i].second};
        graphTSP[i].distance.resize(N);
    }

    for(auto i = 0; i < allNodeVector.size(); i++){
        // Distance from node i to all node 
        for(auto j = 0; j < allNodeVector.size(); j++){
            // distanceToNodeCall(int row_start, int column_start, int row_end, int column_end, bool takeLocalisedPos){
                int distance = distanceToNodeCall(allNodeVector[i].first,
                                                  allNodeVector[i].second, 
                                                  allNodeVector[j].first,
                                                  allNodeVector[j].second
                                                  );   
                graphTSP[i].distance[j] =  distance -1;
                graphTSP[j].distance[i] =  distance -1;
                // ROS_INFO("Node (%d,%d) To Node (%d,%d): %d",allNodeVector[i].first,allNodeVector[i].second,allNodeVector[j].first,allNodeVector[j].second,distance);
                                        
        }
    }

    ROS_INFO("Done Creating Graph");
    printNodeTSPVector(graphTSP);
    return graphTSP;
}

const int INF = 1e9;

std::list<std::pair<int,int>> tsp_held_karp(const std::vector<NodeTSP> dist) {
    int N = dist.size();
    int FULL = 1 << N;

    std::vector<std::vector<int>> dp(FULL, std::vector<int>(N, INF));
    std::vector<std::vector<int>> parent(FULL, std::vector<int>(N, -1));

    dp[1][0] = 0; // Start at node 0

    for (int mask = 1; mask < FULL; ++mask) {
        for (int u = 0; u < N; ++u) {

            if (!(mask & (1 << u))) continue;

            for (int v = 0; v < N; ++v) {

                if (mask & (1 << v)) continue;
                int nextMask = mask | (1 << v);
                int newCost = dp[mask][u] + dist[u].distance[v];
                if (newCost < dp[nextMask][v]) {
                    dp[nextMask][v] = newCost;
                    parent[nextMask][v] = u;
                }
            }
        }
    }

    // Find the final node that gives minimum cost returning to 0
    int minCost = INF;
    int lastNode = -1;
    for (int i = 1; i < N; ++i) {
        int cost = dp[FULL - 1][i] + dist[i].distance[0];
        if (cost < minCost) {
            minCost = cost;
            lastNode = i;
        }
    }

    // Reconstruct path
    std::list<std::pair<int,int>> path;
    int mask = FULL - 1;
    int curr = lastNode;
    path.push_back({dist[0].node.row,dist[0].node.column}); // Start at 0

    std::vector<std::pair<int,int>> reversePath;
    while (curr != 0) {
        reversePath.push_back({dist[curr].node.row,dist[curr].node.column});
        int prev = parent[mask][curr];
        mask = mask ^ (1 << curr);
        curr = prev;
    }

    std::reverse(reversePath.begin(), reversePath.end());
    path.insert(path.end(), reversePath.begin(), reversePath.end());
    // No return to start

    return path;
}

void executeChallenge(std::list<std::pair<int,int>> goldPlan, std::list<std::pair<int,int>> pickUpPlan){

    ros::Rate loop_rate(10);  // 10 Hz loop rate
    std::list<std::pair<int,int>> optimalGoldSequence = goldPlan;
    while(ros::ok()){

        ros::spinOnce();
        if(isLocalisedPosReady){

            std::vector<NodeTSP> tspGraph = generateTSPGraph(optimalGoldSequence);
             optimalGoldSequence = tsp_held_karp(tspGraph);
            for (const auto& path : optimalGoldSequence) {
                std::cout << "(" << path.first << ", " << path.second << ") ";
                
                ROS_INFO( "(%d,%d) -> " ,path.first,path.second);
                
            }
            // Remove the starting node
            optimalGoldSequence.pop_front();
            
            while(!optimalGoldSequence.empty() && isLocalisedPosReady){
                ros::spinOnce();
                if(moveToPositionCall(optimalGoldSequence.front().first,optimalGoldSequence.front().second)){
                    ROS_INFO("Gold Node(%d,%d) reached", optimalGoldSequence.front().first,optimalGoldSequence.front().second);
                    optimalGoldSequence.pop_front();
                    // Playing song
                    ros::Time start_time = ros::Time::now();
                    
                    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 8.0) {
                        ros::spinOnce();
                        playSong(4,playSongClient);
                        loop_rate.sleep();
                    }
                }
                else{
                        ROS_INFO("moveToPositionCall Failed");
                        // tspGraph = generateTSPGraph(optimalGoldSequence);
                        // optimalGoldSequence = tsp_held_karp(tspGraph);
                        // optimalGoldSequence.pop_front();

                }
        
            }
            if (optimalGoldSequence.empty()) {
                // All gold collected
                ROS_INFO("All gold collected. Preceding to Pickup point.");
                break; // <- this exits the outer while(ros::ok()) loop
            }
            

        }
        loop_rate.sleep();

    }

    std::list<std::pair<int,int>> optimalPickUpSequence = pickUpPlan;

    while(ros::ok()){

        ros::spinOnce();
        if(isLocalisedPosReady){

            std::vector<NodeTSP> tspGraph = generateTSPGraph(optimalPickUpSequence);
            optimalPickUpSequence = tsp_held_karp(tspGraph);
            for (const auto& path : optimalPickUpSequence) {
                std::cout << "(" << path.first << ", " << path.second << ") ";
                
                ROS_INFO( "(%d,%d) -> " ,path.first,path.second);
                
            }
            // Remove the starting node
            optimalPickUpSequence.pop_front();
            
            while(!optimalPickUpSequence.empty() && isLocalisedPosReady){
                ros::spinOnce();
                if(moveToPositionCall(optimalPickUpSequence.front().first,optimalPickUpSequence.front().second)){
                    // Shutdown Particle filter
                    std_msgs::Empty msg;
                    pub.publish(msg);
                    ROS_INFO("Shutdown message sent.");
                    ROS_INFO("Pickup Node(%d,%d) reached", optimalPickUpSequence.front().first,optimalPickUpSequence.front().second);
                    optimalPickUpSequence.pop_front();
                    // Playing song
                    ros::Time start_time = ros::Time::now();
                    
                    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 8.0) {
                        ros::spinOnce();
                        playSong(5,playSongClient);
                        loop_rate.sleep();
                    }
                     ROS_INFO("Pickup point reached. Mission complete.");

                     return;
                }
                else{
                        ROS_INFO("moveToPositionCall Failed");
                        // tspGraph = generateTSPGraph(optimalPickUpSequence);
                        // optimalPickUpSequence = tsp_held_karp(tspGraph);
                        // optimalPickUpSequence.pop_front();

                }
        
            }
            // if (optimalPickUpSequence.empty()) {
            //     //Pickup Reached
            //     ROS_INFO("All gold collected. Mission complete.");
            //     // Optional: break the main loop or reinitialize the goldPlan
            //     break; // <- this exits the outer while(ros::ok()) loop
            // }
        }
        loop_rate.sleep();
    }
    
     
}
    
    int main(int argc, char **argv)
{
    ros::init(argc, argv, "execute_challenge_server");  

    ros::NodeHandle nh;
    /*Service*/
    playSongClient = nh.serviceClient<create_fundamentals::PlaySong>("play_song");
    moveToPosition = nh.serviceClient<pink_fundamentals::MoveToPosition>("move_to_position");
    distanceToNode = nh.serviceClient<pink_fundamentals::DistanceToNode>("distance_to_node");
    /*Subscribe to Pos Cell*/
    ros::Subscriber pos_sub = nh.subscribe("/pose", 1, poseCallBack);
    pub = nh.advertise<std_msgs::Empty>("/shutdown_driver", 1);

    std::string filepath_gold = "src/pink_fundamentals/gold.txt";
    std::string filepath_pickup = "src/pink_fundamentals/pickup.txt";

    std::ifstream file_gold(filepath_gold);
    if (!file_gold.is_open()) {
        ROS_ERROR("Failed to open file: %s", filepath_gold.c_str());
        return 1;
    }

    std::string goldMap((std::istreambuf_iterator<char>(file_gold)),
                        std::istreambuf_iterator<char>());

    goldPlan = getPlan(goldMap);     
                        
    std::ifstream file_pickup(filepath_pickup);
    if (!file_pickup.is_open()) {
        ROS_ERROR("Failed to open file: %s", filepath_pickup.c_str());
        return 1;
    }

    std::string pickupMap((std::istreambuf_iterator<char>(file_pickup)),
                        std::istreambuf_iterator<char>());
   
    pickUpPlan = getPlan(pickupMap);

    executeChallenge(goldPlan,pickUpPlan);
    
    return 0;
}
