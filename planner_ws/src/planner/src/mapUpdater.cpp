#include "planner/mapUpdater.h"

MapUpdater::MapUpdater()
{
    global_map_.info.width = 100;
    global_map_.info.height = 100;
    global_map_.info.resolution = 0.1;
    global_map_.data.resize(global_map_.info.width * global_map_.info.height, 0);
}

void MapUpdater::updateGlobalMap(const nav_msgs::msg::OccupancyGrid &local_map)
{
    for (size_t i = 0; i < local_map.data.size(); ++i)
    {
        global_map_.data[i] = std::min(global_map_.data[i] + local_map.data[i], 100);
    }
}

nav_msgs::msg::OccupancyGrid MapUpdater::getGlobalMap() const
{
    return global_map_;
}

int MapUpdater::getWidth() const {
    return global_map_.info.width;
}

int MapUpdater::getHeight() const {
    return global_map_.info.height;
}

double MapUpdater::getResolution() const {
    return global_map_.info.resolution;
}