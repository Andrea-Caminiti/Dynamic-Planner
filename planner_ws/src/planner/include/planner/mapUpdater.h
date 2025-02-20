#pragma once
#include "nav_msgs/msg/occupancy_grid.hpp"

class MapUpdater
{
public:
    MapUpdater();
    void updateGlobalMap(const nav_msgs::msg::OccupancyGrid &local_map);
    nav_msgs::msg::OccupancyGrid getGlobalMap() const;
    int getWidth() const;
    int getHeight() const;
    double getResolution() const;

private:
    nav_msgs::msg::OccupancyGrid global_map_;
};