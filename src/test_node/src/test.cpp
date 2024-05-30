#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "mapio.h"

class MapTestNode : public rclcpp::Node{ // 该类继承自ROS官方节点类
public:
    MapTestNode() : Node("my_test"){ // 表示在派生类中调用基类的构造函数
        // 返回一个智能指针对象
        map_test = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_topic",10); // 使用指针调用基类函数创建名为map的发布者，发布者队列大小为10
    }
    void publishMap(){
        // const表示常量，在publishMap()生命周期中，map_msgs都不会被修改
        const nav_msgs::msg::OccupancyGrid map_msgs = mapio::loadYamlFile("/home/offmind/ros2_ws/src/test_node/map/map.yaml");
        // 以map_msgs为值，创建了一个grid unique指针指向map_msgs，该指针对象拥有独占所有权，grid对象在生命周期内被销毁时，指针指向的对象也会被销毁
        std::unique_ptr<nav_msgs::msg::OccupancyGrid> grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(map_msgs);
        // 在控制台输出地图的resolution相关信息，resolution:0.05，表示地图的每个像素单位为0.05m
        // free_thresh，表示被认为是自由空间的阈值为0.49，occupied_thresh，被认为是占用空间的阈值为0.51
        // info.height和info.width表示行数和列数，分别为1640和1268
        std::cout << grid->info.resolution << std::endl;
        // grid将指向的map_msg所有权转移到map_test节点，之后grid变成nullptr
        map_test->publish(std::move(grid));
    }
private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_test;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    // 使用make_shared标准库创建class对象的时候同时创建智能指针
    std::shared_ptr<MapTestNode> node = std::make_shared<MapTestNode>();
    // 智能指针调用方法的方式
    node->publishMap();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
