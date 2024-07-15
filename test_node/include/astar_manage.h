#ifndef ASTAR_MANAGE_H
#define ASTAR_MANAGE_H

#include <Eigen/Eigen>
#include <thread>
#include <chrono>
#include <future>

#include "mapfix.h"
#include "mapio.h"
#include "binary_search.h"
#include "rclcpp/rclcpp.hpp"

enum PLANNER_STATE {WAITING, IN_PROCESS, COMPLETE, INTERRUPTED, FAILED};

// 返回值
struct SearchResult{
  std::vector<geometry_msgs::msg::Point> points;
  PLANNER_STATE status;
};

class AStarPlannerManager 
{
public:
  // 主要搜索函数
  SearchResult search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, Map2use map2use);
  // 重置status，points
  void planner_reset();
  // 当被中断时
  SearchResult createInterruptResult();
  // 需要的值，当前的状态和拥有的座标点
  PLANNER_STATE status;
  std::vector<geometry_msgs::msg::Point> points;
};  

#endif