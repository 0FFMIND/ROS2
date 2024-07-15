#include "mapio.h"

// 在前面加上&yamlFileName，表示传入的为字符串对象的引用，在函数内部可以高效访问但不能修改
nav_msgs::msg::OccupancyGrid mapio::loadYamlFile(const std::string &yamlFileName) {
    nav_msgs::msg::OccupancyGrid mapmsg;
    nav2_map_server::loadMapFromYaml(yamlFileName, mapmsg);
    return mapmsg;
}

// 将Point世界坐标转化为Grid(x,y)坐标用于A星算法
GridLocation mapio::pointToGrid(const Eigen::Vector3d &world_pos, const Map2use &map2use) {
    // 原理是先获取当前点世界坐标的x值，再计算地图的x原点，再计算偏移量，除以地图的分辨率就得到了当前网格坐标x
    // 世界坐标是resolution的整数倍
    int x = static_cast<int>((world_pos.x() - map2use.map_origin_x) / map2use.map_resolution);
    // 世界坐标的y值同理
    int y = static_cast<int>((world_pos.y() - map2use.map_origin_y) / map2use.map_resolution);
    // 使用网格坐标进行A星搜索
    return GridLocation{x, y};
}

// A星算法得到的Gird(x,y)后，转化为世界坐标的Point用来可视化
geometry_msgs::msg::Point mapio::gridToPoint(const GridLocation &grid_loc, const Map2use &map2use) {
    geometry_msgs::msg::Point point;
    // 逆向转化，网格坐标乘上分辨率，再加上偏移量
    point.x = grid_loc.x * map2use.map_resolution + map2use.map_origin_x;
    point.y = grid_loc.y * map2use.map_resolution + map2use.map_origin_y;
    point.z = 0.0;
    // 返回实际点进入可视化
    return point;
}

Map2use mapio::transferFormat(nav_msgs::msg::OccupancyGrid::SharedPtr mapmsg) {
    Map2use map2use;
    map2use.map_cols = mapmsg->info.width;
    map2use.map_rows = mapmsg->info.height;
    map2use.mapdata = map2use.inflate(mapmsg);
    // 在函数内部修改mapmsg，让他现在的data变为膨胀地图的data
    mapmsg->data = map2use.mapdata;
    map2use.map_origin_x = mapmsg->info.origin.position.x;
    map2use.map_origin_y = mapmsg->info.origin.position.y;
    map2use.map_resolution = mapmsg->info.resolution;
    return map2use;
}