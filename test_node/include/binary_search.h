#ifndef BINARY_SEARCH_H
#define BINARY_SEARCH_H

#include <mutex>
#include <condition_variable>
#include <queue>
#include <unordered_map>
#include <cmath>

#include "nav_msgs/msg/occupancy_grid.hpp"

// 定义结构体，储存A星算法的(x,y)坐标
struct GridLocation {
    int x, y;
    // 在结构体内部声明的函数都为隐式内联，但可以加上inline显示内联
    // 重载运算符，判断两个GridLocation是否相等
    // 前面const表示参数other为常量引用，意味着在函数内部不会修改other参数，而末尾的const确保相等运算符不会修改调用对象
    bool operator==(const GridLocation &other) const {
        return x == other.x && y == other.y;
    }
    // 该函数将当前定位转化为字符串
    std::string toString() const {
        return std::to_string(x) + "," + std::to_string(y);
    }
};

// 声明使用模板类的结构体，可以储存不同的数据类型
template<typename T>
struct GridLocationMap {
    // 使用哈希表创建键值对，键类型为string字符串，值类型为模板参数T
    std::unordered_map<std::string, T> data;
    // 声明put的函数，传入GridLocation的结构体，添加/更新对应GridLocation的键值对
    void put(GridLocation &location, const T value) {
        std::string key = location.toString();
        data[key] = value;
    }
    // 通过传进来的GridLocation检索对应键的储存值
    T get(const GridLocation &location){
        std::string key = location.toString();
        // 在哈希表中查找是否有这个键，find()函数返回为迭代器类型
        auto it = data.find(key);
        // data.end()指向unordered_map最后一个元素的下一个位置，是一个无效的迭代器
        if(it != data.end()) {
            return it->second;
        }
        // 没找到的时候抛出异常，说明未在读取的地图中
        throw std::runtime_error("该点未被当前地图读取：" + location.toString());
    }
    // 检查GridLocation键值对是否存在于data中
    bool contains(const GridLocation &location) const{
        std::string key = location.toString();
        // 查找key是否存在于data中
        return data.find(key) != data.end();
    }
    // 在data中移除该GridLocation的键值对
    void remove(GridLocation &location){
        std::string key = location.toString();
        // 从data中移除key
        data.erase(key);
    }
};

// 声明优先队列
struct ComparePriority {
    bool operator()(const std::pair<double, GridLocation> &a, const std::pair<double, GridLocation> &b) const {
        return a.first > b.first;
    }
};

struct PriorityQueue {
    typedef std::pair<double, GridLocation> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>, ComparePriority> elements;
    GridLocationMap<double> best_priority;

    inline bool empty() {
        return elements.empty();
    }

    inline void put(GridLocation item, double priority) {
        if (!best_priority.contains(item) || priority < best_priority.get(item)) {
            best_priority.put(item, priority);
            elements.emplace(priority, item);
        }
    }

    GridLocation get() {
        GridLocation best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};

struct gridGraph {
    unsigned int width;
    unsigned int height;
    std::vector<int8_t> grid_data;

    std::vector<GridLocation> neighbors(GridLocation l) {
        std::vector<GridLocation> results;
        std::vector<std::pair<int, int>> directions{{1,  0}, {-1, 0}, {0,  1}, {0,  -1}, {-1, 1}, {1,  1}, {1,  -1}, {-1, -1}};
        for (auto &[dx, dy] : directions) {
            int nx = l.x + dx;
            int ny = l.y + dy;
            if (nx >= 0 && ny >= 0) {
                auto unx = static_cast<uint32_t>(nx);
                auto uny = static_cast<uint32_t>(ny);
                if (unx < width && uny < height) {
                    auto value = grid_data[uny * width + unx];
                    if (value < 49) {
                        results.push_back({nx, ny});
                    }
                }
            }
        }
        return results;
    }

    void initGridGraph(const std::vector<int8_t> &data, int w, int h) {
        grid_data = data;
        width = w;
        height = h;
    }

    explicit gridGraph() = default;
};

inline double heuristic(GridLocation a, GridLocation b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

inline void a_star_search(gridGraph graph, GridLocation start, GridLocation goal, GridLocationMap<GridLocation> &came_from, GridLocationMap<double> &cost_so_far) {
    PriorityQueue frontier;
    frontier.put(start, 0);

    came_from.put(start, start);
    cost_so_far.put(start, 0);

    while (!frontier.empty()) {
        GridLocation current = frontier.get();

        if (current == goal) {
            break;
        }

        for (GridLocation next : graph.neighbors(current)) {
            double new_cost = cost_so_far.get(current) + 1;
            if (!cost_so_far.contains(next) || new_cost < cost_so_far.get(next)) {
                cost_so_far.put(next, new_cost);
                double priority = new_cost + 10 * heuristic(next, goal);
                frontier.put(next, priority);
                came_from.put(next, current);
            }
        }
    }
}

#endif