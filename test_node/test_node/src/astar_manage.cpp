#include <chrono>
#include <iostream>

#include "astar_manage.h"

SearchResult AStarPlannerManager::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, Map2use mapmsg) {

    status = IN_PROCESS;

    GridLocation start = mapio::pointToGrid(start_pt, mapmsg);
    GridLocation goal = mapio::pointToGrid(end_pt, mapmsg);
    std::cout << "坐标开始点：" << start.x << "," << start.y << std::endl;
    std::cout << "坐标终点：" << goal.x << "," << goal.y << std::endl;
    gridGraph graph;
    graph.initGridGraph(mapmsg.mapdata, mapmsg.map_cols, mapmsg.map_rows);

    GridLocationMap<GridLocation> came_from;
    GridLocationMap<double> cost_so_far;

    auto start_time = std::chrono::high_resolution_clock::now();

    a_star_search(graph, start, goal, came_from, cost_so_far);

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    std::cout << "A*搜索耗时：" << duration.count() << "秒" << std::endl;

    GridLocation current = goal;
    while (!(current == start)) {
        points.push_back(mapio::gridToPoint(current, mapmsg));
        current = came_from.get(current);
    }
    points.push_back(mapio::gridToPoint(start, mapmsg));
    std::reverse(points.begin(), points.end());
    status = COMPLETE;

    return{points, status};
}

void AStarPlannerManager::planner_reset() {
    status = WAITING;
    points.clear();
}

SearchResult AStarPlannerManager::createInterruptResult() {
    status = INTERRUPTED;
    points.clear();
    return{points, status};
}
