#include <memory>
#include <iostream>
#include <vector>
#include <chrono>
#include <Eigen/Eigen>
#include <thread>
#include <atomic>
#include <future>
#include <exception>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "astar_manage.h"
#include "mapio.h"

class MapTestNode : public rclcpp::Node {
public:
    MapTestNode() : Node("my_test") {
        // 创建发布者，并将智能指针赋值给pub对象
        map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_topic", 10);
        inflated_map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("inflated_map_topic", 10);
        marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 30);
        // 创建定时器对象
        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(retry_time_ * 1000)), 
            [this](){
                if(retry_flag_ == true) {
                    retry_flag_ = false;
                    start_search(pos_prev_x, pos_prev_y, pos_goal_x, pos_goal_y);
                }
                timer_->cancel();
            }
        );
        timer_->cancel();
        // 创建RViz接受者，每接收到新Rviz消息/goal_pose后，触发callback回调函数
        goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, 
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                this->goalPoseCallback(msg);
            });
    }

    void publishMap(){
        // map_msgs为指向常量栅格化地图的智能指针
        map_msgs = std::make_shared<const nav_msgs::msg::OccupancyGrid>(mapio::loadYamlFile("/home/offmind/ros2_ws/src/test_node/map/map.yaml"));
        // 在控制台输出地图的resolution相关信息，resolution:0.05，表示地图的每个像素单位为0.05m，info.height和info.width表示行数和列数，分别为1640和1268
        std::cout << map_msgs->info.resolution << std::endl;
        // 解引用指针，得到map_msgs指向的OccupancyGrid对象，将其发布
        map_pub->publish(*map_msgs);
        RCLCPP_INFO(this->get_logger(), "订阅/map_topic以接收地图，订阅/visualization_marker以接收marker，接收RViz订阅/goal_topic");
    }

    void inflateMapinThread(){
        std::thread inflate_thread([this, const_map_msgs = map_msgs](){
            // 使用const_cast将指针转为非const指针，传递给mapio::transferFormat
            auto non_const_map_msgs = std::const_pointer_cast<nav_msgs::msg::OccupancyGrid>(const_map_msgs);
            map2use = mapio::transferFormat(non_const_map_msgs);
            // 发布膨胀后的地图
            inflated_map_pub->publish(*non_const_map_msgs);
            RCLCPP_INFO(this->get_logger(), "订阅/inflated_map_topic以接收膨胀后的地图");
        });
        // 将线程放在后台运行
        inflate_thread.detach();
    }
private:

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "获得Position信息，x位置：%f，y位置：%f", msg->pose.position.x, msg->pose.position.y);
        timer_->cancel();
        path_searcher.status = PLANNER_STATE::INTERRUPTED;

        if(init_receive_flag_ == false) {
            pos_goal_x = msg->pose.position.x;
            pos_goal_y = msg->pose.position.y;
            init_receive_flag_ = true;
        }
        else {
            pos_prev_x = pos_goal_x;
            pos_prev_y = pos_goal_y;
            pos_goal_x = msg->pose.position.x;
            pos_goal_y = msg->pose.position.y;
            start_search(pos_prev_x, pos_prev_y, pos_goal_x, pos_goal_y);
        }
    }

    void start_search(float pos_start_x, float pos_start_y, float pos_end_x, float pos_end_y) {
        Eigen::Vector3d start_pt(pos_start_x, pos_start_y, 0.0);    // 起始点坐标
        Eigen::Vector3d end_pt(pos_end_x, pos_end_y, 0.0);          // 终点坐标
        RCLCPP_INFO(this->get_logger(), "[AStar plan]：开始A星规划");
        // 每次规划前需要清除之前的数据
        path_searcher.planner_reset();
        retry_flag_ = false;
        SearchResult astarResult = path_searcher.search(start_pt, end_pt, map2use);
        for (const auto& point : astarResult.points) {
    std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
}
        std::cout << astarResult.status << std::endl;
        // 当没有返回坐标点则意味着当前没有可行路径，而若规划成功则进入可视化
        if (astarResult.status == PLANNER_STATE::FAILED) {
            RCLCPP_INFO(this->get_logger(), "[AStar plan]: A星规划失败，将在%f后重新规划", retry_time_);
            retry_flag_ = true;
            // 启动主函数中的定时器，在1秒后(规定的retry_time_数值)重新规划路径
            timer_->reset();
        } else if (astarResult.status == PLANNER_STATE::COMPLETE) {
            RCLCPP_INFO(this->get_logger(), "[AStar plan]: A星规划成功，进入路径可视化");
            visualization_callback(marker_pub, astarResult.points);
        }
    }

    void visualization_callback(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_, const std::vector<geometry_msgs::msg::Point>& points) {
        // 创建点ponits_marker消息
        visualization_msgs::msg::Marker points_marker;
        points_marker.header.frame_id = "map";                        // 设置参考坐标系
        points_marker.header.stamp = rclcpp::Clock().now();           // 设置时间戳
        points_marker.ns = "points_and_lines";                        // 设置命名空间名字
        points_marker.id = 0;                                         // 设置唯一标识符为0
        points_marker.type = visualization_msgs::msg::Marker::POINTS; // 设置marker类型为点
        // 设置marker的动作为添加
        points_marker.action = visualization_msgs::msg::Marker::ADD;
        // 设置marker参数
        points_marker.scale.x = 0.1;
        points_marker.scale.y = 0.1;
        points_marker.color.r = 1.0;
        points_marker.color.g = 0.0;
        points_marker.color.b = 0.0;
        points_marker.color.a = 0.5;
        // 创建线line_marker消息
        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = rclcpp::Clock().now();
        line_marker.ns = "points_and_lines";
        line_marker.id = 1;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;// 设置marker类型为线
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.scale.x = 0.05;                                    // 设置线条宽度
        line_marker.color.r = 0.0f;
        line_marker.color.g = 0.6f;
        line_marker.color.b = 0.0f;
        line_marker.color.a = 0.5;
        // 将点添加到points_marker和line_marker中
        for (const auto& p : points) {
            points_marker.points.push_back(p);
            line_marker.points.push_back(p);
        }
        marker_pub_->publish(points_marker);
        marker_pub_->publish(line_marker);
    }

    // 声明成员变量

    Map2use map2use;
    AStarPlannerManager path_searcher;

    // 定时器相关
    rclcpp::TimerBase::SharedPtr timer_;
    float retry_time_ = 1.0;

    // 异步搜索相关
    std::future_status future_status;
    // 创建promise对象，表示在线程中设置一个SearchResult的返回值，让future可以获取
    std::promise<SearchResult> search_promise;
    std::future<SearchResult> search_future;
    
    // 使用atomic<bool>确保线程安全性，而static声明的静态成员变量必须在类外初始化
    static std::atomic<bool> retry_flag_;
    static std::atomic<bool> init_receive_flag_;

    // float基本类型用来储存简单的数值类型变量，不需要指针或者复杂的数据结构
    float pos_prev_x = 0, pos_prev_y = 0, pos_goal_x = 0, pos_goal_y = 0;

    // 来自ROS2官方库的智能指针写法，SharedPtr是智能指针的一种实现
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_map_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;

    // std::shared_ptr为来自标准库<memory>下面的智能指针，可以保证多个指针指向同一个对象，同时避免悬空指针的问题
    // 当最后一个指针被销毁时，该对象会被自动销毁，而const表示该对象是只读的不能被修改
    std::shared_ptr<const nav_msgs::msg::OccupancyGrid> map_msgs;
};

std::atomic<bool> MapTestNode::retry_flag_ = false;
std::atomic<bool> MapTestNode::init_receive_flag_ = false;

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    // 使用make_shared标准库创建class对象的时候同时创建智能指针
    std::shared_ptr<MapTestNode> node = std::make_shared<MapTestNode>();
    // 智能指针node调用publishMap方法
    node->publishMap();
    // 启动地图膨胀线程
    node->inflateMapinThread();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
