#include "../include/path_generator/path_generator.h"

#include <iostream>

float current_x;
float current_y;
float current_h;

PathGenerator::PathGenerator()
{
    subscribeAndPublish();
}

PathGenerator::~PathGenerator()
{

}

void PathGenerator::subscribeAndPublish(){
    sub_grid_map_ = nh_.subscribe<nav_msgs::OccupancyGrid>("map", 1, &PathGenerator::gridMapHandler, this);
    sub_nav_goal_ = nh_.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, &PathGenerator::navGoalHandler, this); //RVIZ publica en este topico al hacer clic en 2d nav goal
    sub_current_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("flat_map_pose", 1, &PathGenerator::updatePosition, this); //recibe posicion traducida a mapa 2d
    sub_current_pose_height_ = nh_.subscribe<geometry_msgs::PoseStamped>("global_pose", 1, &PathGenerator::updateHeight, this); //recibe altura
    pub_robot_waypoint_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 100);
    pub_robot_path_ = nh_.advertise<nav_msgs::Path>("robot_path", 10);
}

void PathGenerator::gridMapHandler(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
    ROS_INFO("Generating map..");
    map_exsit_ = false;

    map_info_ = map_msg->info;

    // Generate Map, Options
    map_generator_.setWorldSize({(int)map_info_.width, (int)map_info_.height}); //{x, y}
    map_generator_.setHeuristic(AStar::Heuristic::manhattan);
    map_generator_.setDiagonalMovement(true);

    // Add Wall
    int x, y;
    for(int i=0; i<map_info_.width*map_info_.height; i++)
    {
        x = i%map_info_.width;
        y = i/map_info_.width;

        if(map_msg->data[i] != 0)
        {
            map_generator_.addCollision({x, y}, 3);
        }
    }

    ROS_INFO("Success build map!");
    map_exsit_ = true;
}

void PathGenerator::updatePosition(const geometry_msgs::PoseStamped::ConstPtr &current_pose_msg){
    // Round current coordinate
    current_x = round(current_pose_msg->pose.position.x*10)/10;
    current_y = round(current_pose_msg->pose.position.y*10)/10;

}
void PathGenerator::updateHeight(const geometry_msgs::PoseStamped::ConstPtr &current_height_msg){
    // Round current coordinate
    current_h = current_height_msg->pose.position.z;
    
}

void PathGenerator::navGoalHandler(const geometry_msgs::PoseStamped::ConstPtr &goal_msg)
{
    if(!map_exsit_) return;

    ROS_INFO("\033[1;32mGoal received!\033[0m");

    // Round goal coordinate
    float goal_x = round(goal_msg->pose.position.x*10)/10;
    float goal_y = round(goal_msg->pose.position.y*10)/10;

    // Remmaping coordinate
    AStar::Vec2i target;
    target.x = (goal_x - map_info_.origin.position.x) / map_info_.resolution;
    target.y = (goal_y - map_info_.origin.position.y) / map_info_.resolution;

    AStar::Vec2i source;
    source.x = (current_x - map_info_.origin.position.x) / map_info_.resolution;
    source.y = (current_y - map_info_.origin.position.y) / map_info_.resolution;

    // Find Path
    auto path = map_generator_.findPath(source, target);

    geometry_msgs::PoseStamped path_msg;
    nav_msgs::Path path_rviz;

    if(path.empty())
    {
        ROS_INFO("\033[1;31mFail generate path!\033[0m");
        return;
    }

    for(auto coordinate=path.end()-1; coordinate>=path.begin(); --coordinate)
    {
        geometry_msgs::PoseStamped point_pose;

        // Remapping coordinate
        point_pose.pose.position.x = (coordinate->x + map_info_.origin.position.x / map_info_.resolution) * map_info_.resolution;
        point_pose.pose.position.y = (coordinate->y + map_info_.origin.position.y / map_info_.resolution) * map_info_.resolution;
        point_pose.pose.orientation = goal_msg->pose.orientation;
        path_rviz.poses.push_back(point_pose);
        }
        path_rviz.header.frame_id = "map";
        pub_robot_path_.publish(path_rviz);
        ROS_INFO("\033[1;36mSuccess generating path!\033[0m");


    for(auto coordinate=path.end()-1; coordinate>=path.begin(); --coordinate)
    {
        geometry_msgs::PoseStamped point_pose;
        // Remapping coordinate
        point_pose.pose.position.x = (coordinate->x + map_info_.origin.position.x / map_info_.resolution) * map_info_.resolution;
        point_pose.pose.position.y = (coordinate->y + map_info_.origin.position.y / map_info_.resolution) * map_info_.resolution;
        point_pose.pose.position.z = current_h;
        point_pose.pose.orientation = goal_msg->pose.orientation;
        path_msg.pose = point_pose.pose;
        path_msg.header.frame_id = "map";
        pub_robot_waypoint_.publish(path_msg);
        ros::Duration(1.0).sleep();
    }

    ROS_INFO("\033[1;34mSuccess following path!\033[0m");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_generator");

    ROS_INFO("\033[1;32m----> Path Generator Node is Started.\033[0m");

    PathGenerator PG;

    ros::spin();
    return 0;
}
