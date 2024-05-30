#ifndef MAPIO_H
#define MAPIO_H

#include <string>
#include <vector>
#include <iostream>
#include <cstddef>
#include <Eigen/Eigen>

#include "mapfix.h"
#include "nav2_map_server/map_io.hpp"

class mapio {
public:
    static nav_msgs::msg::OccupancyGrid loadYamlFile(const std::string &yamlFileName);

    static Eigen::ArrayXXd occupancyGridToEigenMatrix(const nav_msgs::msg::OccupancyGrid &grid);

    static Map2use transferFormat(nav_msgs::msg::OccupancyGrid::SharedPtr mapmsg);
};

#endif 
