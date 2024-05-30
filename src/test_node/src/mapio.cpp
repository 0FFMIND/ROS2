#include "mapio.h"

nav_msgs::msg::OccupancyGrid mapio::loadYamlFile(const std::string &yamlFileName) {
    nav_msgs::msg::OccupancyGrid mapmsg;
    nav2_map_server::loadMapFromYaml(yamlFileName, mapmsg);
    return mapmsg;
}

Eigen::ArrayXXd mapio::occupancyGridToEigenMatrix(const nav_msgs::msg::OccupancyGrid &grid) {
    // 创建一个Eigen矩阵，尺寸与OccupancyGrid的宽度和高度一致
    Eigen::ArrayXXd matrix(grid.info.height, grid.info.width);
    // 遍历OccupancyGrid数据，并填充到Eigen矩阵中
    for (size_t y = 0; y < grid.info.height; ++y) {
        for (size_t x = 0; x < grid.info.width; ++x) {
            // 计算当前cell在数组中的索引
            size_t index = x + y * grid.info.width;
            // 将OccupancyGrid中的数据转换为整数并赋值给Eigen矩阵
            matrix(y, x) = static_cast<int>(grid.data[index]);
        }
    }
    return matrix;
}

Map2use mapio::transferFormat(nav_msgs::msg::OccupancyGrid::SharedPtr mapmsg) {
    Map2use map2use;
    map2use.map_cols = mapmsg->info.width;
    map2use.map_rows = mapmsg->info.height;
    map2use.mapdata = map2use.inflate(mapmsg);
    mapmsg->data = map2use.mapdata;
    map2use.map_origin_x = mapmsg->info.origin.position.x;
    map2use.map_origin_y = mapmsg->info.origin.position.y;
    map2use.map_resolution = mapmsg->info.resolution;
    return map2use;
}