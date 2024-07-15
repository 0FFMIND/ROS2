#ifndef MAPIO_H
#define MAPIO_H

#include <string>
#include <vector>
#include <iostream>
#include <cstddef>
#include <Eigen/Eigen>

#include "mapfix.h"
#include "binary_search.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_map_server/map_io.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"

class mapio {
public:
    static nav_msgs::msg::OccupancyGrid loadYamlFile(const std::string &yamlFileName);

    static GridLocation pointToGrid(const Eigen::Vector3d &world_pos, const Map2use &map2use);

    static geometry_msgs::msg::Point gridToPoint(const GridLocation &grid_loc, const Map2use &map2use);

    static Map2use transferFormat(nav_msgs::msg::OccupancyGrid::SharedPtr mapmsg);
};

#endif 
