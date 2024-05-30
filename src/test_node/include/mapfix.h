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
    float rep_step_dis;
    Eigen::Vector2f totalVector;
public:
    static constexpr double inflation_radius_ = 0.35;
    static constexpr double inflation_radius_Optmized = 0.1;
    static constexpr double inflation_radius_dynamic_Optmized = 0.24;
    int    Esdf_CheakAngle = 20;
    int    Esdf_CheakUpSteepDiv = 1;
    float  Esdf_CheakDis = 1.1;
    double map_width; // 宽度为3m
    double map_height; // 高度为3m
    double map_resolution; // 分辨率为10cm
    int  map_rows; // 行数
    int  map_cols; // 列数
    int  map_size;
    double  map_origin_x;
    double  map_origin_y;
    std::vector<int8_t> mapdata;

    inline static sensor_msgs::msg::LaserScan::SharedPtr scanData;
    inline static geometry_msgs::msg::PoseStamped::SharedPtr carPoseData;

    // dynamic obstacle index
    inline static std::atomic_bool read_flag = false;
    inline static std::set<size_t> inflated_data;
    inline static std::set<size_t> bak_inflated_data;
    inline static int8_t *inflated_data1;
    inline static int8_t *inflated_data2;

    void map_create();
    void Init();
    void map_Teminal_cout();
    Eigen::Vector2d CalcuDirVector(float x, float y);
    std::vector<Eigen::Vector2f> CalcuDirVectorTest1(float x, float y);
    Eigen::Vector2d get_repulsion_vector(Eigen::Vector2d PointPos,Eigen::Vector2d verticalVector);
    void map_inform_read(double & origin_x,double & origin_y,
                                   int & width,int & height,
                                        int & size,double & resolution);
    float map_if_occupied(float pos_x,float pos_y);
    std::vector<int8_t> inflate(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void static inflateDynanmic(const nav_msgs::msg::OccupancyGrid &msg,std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> pub);
    void static inflateWithDynamicPoints(const nav_msgs::msg::OccupancyGrid &msg, const std::vector<Eigen::Vector2d>& dpoints,std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> pub);
    
};


#endif
