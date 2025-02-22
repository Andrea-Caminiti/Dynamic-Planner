#include "planner/laserProcessor.h"

LaserScanProcessor::LaserScanProcessor() {}

nav_msgs::msg::OccupancyGrid LaserScanProcessor::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
{
    nav_msgs::msg::OccupancyGrid local_map;
    rclcpp::Clock clock(RCL_SYSTEM_TIME);
    local_map.header.frame_id = "local_map";
    local_map.header.stamp = clock.now();
    local_map.info.width = 100;
    local_map.info.height = 100;
    local_map.info.resolution = 0.1; // 10 cm per cell
    local_map.data.resize(local_map.info.width * local_map.info.height, 0);

    float angle = scan->angle_min;
    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
        float range = scan->ranges[i];
        if (range < scan->range_max)
        {
            int x = static_cast<int>((range * cos(angle)) / local_map.info.resolution);
            int y = static_cast<int>((range * sin(angle)) / local_map.info.resolution);
            if (x >= 0 && x < 100 && y >= 0 && y < 100)
            {
                local_map.data[y * 100 + x] = 100;
            }
        }
        angle += scan->angle_increment;
    }
    return local_map;
}