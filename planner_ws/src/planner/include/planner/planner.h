#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <queue>
#include <vector>
#include <cmath>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <interactive_markers/interactive_marker_server.hpp>

class Planner : public rclcpp::Node{
public:
    Planner();

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    

    std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pose_pub_;
    
    //A* parameters
    const int start_x = 4;
    const int start_y = 4;
    const int goal_x = 69;
    const int goal_y = 75;

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    std::vector<std::pair<int, int>> aStar(int start_x, int start_y, int goal_x, int goal_y);
    void publishPath();
    void createMarker(double x, double y);
    void processMarkerFeedback(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback>& feedback);
};  