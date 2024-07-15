#ifndef MAPFIX_H
#define MAPFIX_H

#include <Eigen/Eigen>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class Map2use {
private:
    float rep_step_dis;             // 地图的步长距离 = resolution
    Eigen::Vector2f totalVector;    // 总向量表示

public:
    // 膨胀半径
    static constexpr double inflation_radius_ = 0.35;
    static constexpr double inflation_radius_Optmized = 0.1;
    static constexpr double inflation_radius_dynamic_Optmized = 0.24;
    // ESDF欧几里德距离场，对栅格化地图进一步表示
    int    Esdf_CheakAngle = 20;
    int    Esdf_CheakUpSteepDiv = 1;
    float  Esdf_CheakDis = 1.1;
    // 地图参数
    double map_width;       // 地图宽度
    double map_height;      // 地图高度
    double map_resolution;  // 地图分辨率
    // 栅格化地图参数
    int  map_rows;                  // 行数
    int  map_cols;                  // 列数
    int  map_size;                  // 地图总大小
    double  map_origin_x;           // 地图x原点
    double  map_origin_y;           // 地图y原点
    std::vector<int8_t> mapdata;    // 地图数据，-1表示未知，0-100表示自由空间-完全占据
    // 激光扫描数据和当前车辆位置
    static inline sensor_msgs::msg::LaserScan::SharedPtr scanData;
    static inline geometry_msgs::msg::PoseStamped::SharedPtr carPoseData;
    // 动态障碍物索引与标志
    static inline std::atomic_bool read_flag = false;   // 读取标志
    static inline std::set<size_t> inflated_data;
    static inline std::set<size_t> bak_inflated_data;
    static inline int8_t *inflated_data1;
    static inline int8_t *inflated_data2;

    // 方法声明
    void map_create(); // 创建地图
    void Init();
    Eigen::Vector2d CalcuDirVector(float x, float y);
    std::vector<Eigen::Vector2f> CalcuDirVectorTest1(float x, float y);
    Eigen::Vector2d get_repulsion_vector(Eigen::Vector2d PointPos,Eigen::Vector2d verticalVector);
    void map_inform_read(double & origin_x,double & origin_y,
                                   int & width,int & height,
                                        int & size,double & resolution);
    float map_if_occupied(float pos_x,float pos_y);
    std::vector<int8_t> inflate(nav_msgs::msg::OccupancyGrid::SharedPtr msg);  // 膨胀静态地图
    void static inflateDynanmic(const nav_msgs::msg::OccupancyGrid &msg,std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> pub);
    void static inflateWithDynamicPoints(const nav_msgs::msg::OccupancyGrid &msg, const std::vector<Eigen::Vector2d>& dpoints,std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> pub);
    
};


#endif
