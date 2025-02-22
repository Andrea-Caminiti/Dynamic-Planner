#include "planner/planner.h"
#include "planner/laserProcessor.h" 
#include "planner/mapUpdater.h"

MapUpdater mapUp;
LaserScanProcessor laserProcessor;

Planner::Planner(): Node("Planner") {

    laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Planner::laserCallback, this, std::placeholders::_1));
    grid_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    marker_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/marker_pose", 10);

    marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("obstacle_markers",
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_topics_interface(),
        this->get_node_services_interface()
    );
        
    createMarker(0.0, 0.0);

    marker_server_->applyChanges();

};

void Planner::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr laser){
    nav_msgs::msg::OccupancyGrid grid = laserProcessor.processLaserScan(laser);
    mapUp.updateGlobalMap(grid);
    grid_pub->publish(mapUp.getGlobalMap());
    publishPath();
};

void Planner::createMarker(double x, double y) {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.header.stamp = this->get_clock()->now();
    int_marker.name = "obstacle_marker";
    int_marker.description = "Obstacle";
    int_marker.scale = 0.1;
    int_marker.pose.position.x = x;
    int_marker.pose.position.y = y;
    int_marker.pose.position.z = 0.0;

    visualization_msgs::msg::InteractiveMarkerControl control;
    control.always_visible = true;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;

    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    control.markers.push_back(marker);
    int_marker.controls.push_back(control);

    marker_server_->insert(int_marker);
    marker_server_->setCallback(int_marker.name, std::bind(&Planner::processMarkerFeedback, this, std::placeholders::_1));
}

void Planner::processMarkerFeedback(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback>& feedback) {
    if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
        
        geometry_msgs::msg::PoseStamped marker_pose;
        marker_pose.header.stamp = this->get_clock()->now();
        marker_pose.header.frame_id = "map";  // Ensure frame matches your map

        marker_pose.pose = feedback->pose;  // Get marker position from feedback

        marker_pose_pub_->publish(marker_pose);
        
    }
}


std::vector<std::pair<int, int>> Planner::aStar(int start_x, int start_y, int goal_x, int goal_y){
    struct Cell {
        int x;
        int y;
        double cost;
    };
    
    auto compare = [](const Cell& a, const Cell& b){return a.cost > b.cost;};
    std::priority_queue<Cell, std::vector<Cell>, decltype(compare) > open_list(compare);

    std::vector<std::vector<double>> cost(mapUp.getWidth(), std::vector<double>(mapUp.getHeight(), 1e9));
    std::vector<std::vector<std::pair<int, int>>> parent(mapUp.getWidth(), std::vector<std::pair<int, int>>(mapUp.getHeight(), {0, 0}));

    open_list.push({start_x, start_y, 0.0});
    cost[start_x][start_y] = 0.0;
    std::vector<std::pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}}; //right, left, up, down, top-right, top-left, bottom-right, bottom-left
    int width = mapUp.getWidth();
    int height = mapUp.getHeight();
    while (!open_list.empty()) {
        auto current = open_list.top();
        open_list.pop();

        if (current.x == goal_x && current.y == goal_y) {
            std::vector<std::pair<int, int>> path;
            for (int x = goal_x, y = goal_y; x != 0 && y != 0; std::tie(x, y) = parent[x][y]) {
            path.emplace_back(x, y);
            }
            std::reverse(path.begin(), path.end());
            return path;
            };

        for (auto [dx, dy] : directions) {
            int nx = current.x + dx;
            int ny = current.y + dy;

            int c = mapUp.getGlobalMap().data[ny * width + nx];

            if ((nx >= 0 && nx < width && ny >= 0 && ny < height) && c < 100) {
                double mc = (dx != 0 && dy != 0) ? 1.414 : 1.0; //diagonal movement costs more!
                double new_cost = cost[current.x][current.y] + mc;
                if (new_cost < cost[nx][ny]) {
                    cost[nx][ny] = new_cost;
                    parent[nx][ny] = {current.x, current.y};
                    double heuristic = std::hypot(goal_x - nx, goal_y - ny);
                    open_list.push({nx, ny, new_cost + heuristic});
                }
            }
            else continue;
        }
    }
    return {}; //No path found
}   

void Planner::publishPath() {
    auto path = aStar(start_x, start_y, goal_x, goal_y);

    nav_msgs::msg::Path ros_path;
    ros_path.header.frame_id = "map";
    ros_path.header.stamp = this->get_clock()->now();
    double resolution = mapUp.getResolution();
    for (auto [x, y] : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = x * resolution;
        pose.pose.position.y = y * resolution;
        pose.pose.orientation.w = 1.0;
        ros_path.poses.push_back(pose);
    }

    path_pub->publish(ros_path);
}

