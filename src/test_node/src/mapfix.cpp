#include <iostream>
#include <cmath>

#include "mapfix.h"

void Map2use::map_create() {
    //地图尺寸和分辨率
    map_origin_x = 0;
    map_origin_y = 0;
    map_width = 3.0; // 宽度为3m
    map_height = 3.0; // 高度为3m
    map_resolution = 0.1; // 分辨率为3cm
    // 计算地图的行数和列数
    map_rows = static_cast<int>(map_height / map_resolution);
    map_cols = static_cast<int>(map_width / map_resolution);
    map_size = map_rows * map_cols;
}

void Map2use::Init() {
    rep_step_dis = map_resolution;
}

void Map2use::inflateDynanmic(const nav_msgs::msg::OccupancyGrid &msg,
                              std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> pub) {

    std::vector<Eigen::Vector2d> obstaclePoints;

    if (carPoseData == nullptr || scanData == nullptr) {
        return;
    }


    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

    // Translation
    transform(0, 3) = carPoseData->pose.position.x;
    transform(1, 3) = carPoseData->pose.position.y;
    transform(2, 3) = carPoseData->pose.position.z;

    // Rotation (quaternion)
    Eigen::Quaterniond quat(carPoseData->pose.orientation.w, carPoseData->pose.orientation.x,
                            carPoseData->pose.orientation.y, carPoseData->pose.orientation.z);
    Eigen::Matrix3d rotation = quat.toRotationMatrix();
    transform.block<3, 3>(0, 0) = rotation;

//        std::cout << transform << std::endl;

    float minAngle = scanData->angle_min;
//        std::cout << minAngle << "-" << scanData->angle_max << std::endl;
    //in laser valid range

//        std::cout << "debug" << std::endl;
    float dis;

    for (const auto &range: scanData->ranges) {
        if (fabs(minAngle) <= (15 / 57.3f))dis = 4.0f;
        else if (fabs(minAngle) > (15 / 57.3f) && fabs(minAngle) <= (30 / 57.3f)) dis = 3.2f;
        else if (fabs(minAngle) > (30 / 57.3f) && fabs(minAngle) <= (60 / 57.3f)) dis = 2.6f;
        else if (fabs(minAngle) > (60 / 57.3f) && fabs(minAngle) <= (120 / 57.3f))dis = 1.6f;
        else dis = 1.0f;


        if (range < scanData->range_min || range > scanData->range_max || fabs(minAngle) > (135 / 57.3f) ||
            range > dis || range < 0.07) {
            minAngle += scanData->angle_increment;
            continue;
        }



        // transform
        double dx = range * std::cos(minAngle);
        double dy = range * std::sin(minAngle);
        Eigen::Vector4d point(dx, dy, 0, 1);
        Eigen::Vector4d transformed_point = transform * point;
//        printf("x: %f, y: %f\n", transformed_point(0), transformed_point(1));
        obstaclePoints.emplace_back(transformed_point(0), transformed_point(1));
        minAngle += scanData->angle_increment;

        //
    }

//    for (const auto &item: obstaclePoints){
//        std::cout << item(0) << "," << item(1) << std::endl;
//    }

    inflateWithDynamicPoints(msg, obstaclePoints, std::move(pub));


}

void Map2use::inflateWithDynamicPoints(const nav_msgs::msg::OccupancyGrid &msg,
                                       const std::vector<Eigen::Vector2d> &dpoints,
                                       std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> pub) {


//    bak_inflated_data.clear();
    memset(inflated_data1, 0, msg.info.width * msg.info.height * sizeof(bool));


    auto temp_msg = msg;
    uint32_t width = msg.info.width;
    uint32_t height = msg.info.height;
    double resolution_ = 0.05;

//    int inflation_radius = int(car_radius / resolution_);


    int inflation_grid = int(inflation_radius_ / resolution_);
    int esdf_inflation_grid = int(inflation_radius_dynamic_Optmized / resolution_);


    for (const auto &item: dpoints) {
        int x = floor((item(0) - msg.info.origin.position.x) / msg.info.resolution), y = floor(
                (item(1) - msg.info.origin.position.y) / msg.info.resolution);
        size_t index = x + y * msg.info.width;
//        bak_inflated_data.insert(index);
        inflated_data1[index] = 100;
        for (int i = -inflation_grid; i <= inflation_grid; ++i) {
            for (int j = -inflation_grid; j <= inflation_grid; ++j) {
                int nx = x + j;
                int ny = y + i;
                if (nx >= 0 && ny >= 0) {
                    auto ux = static_cast<uint32_t>(nx);
                    auto uy = static_cast<uint32_t>(ny);
                    if (ux < width && uy < height) {
                        auto dis = sqrt(i * i + j * j);
                        size_t nindex = ny * width + nx;
                        if (inflated_data1[nindex] != 100) {
                            if (dis <= esdf_inflation_grid) {
                                inflated_data1[nindex] = 99;
                            } else if (dis <= inflation_grid && inflated_data1[nindex] != 99) {
                                inflated_data1[nindex] = 90;
                            }
                        }
                    }

                }
//                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
//                    if (sqrt(i * i + j * j) <= inflation_radius) {
//                        int nindex = ny * width + nx;
//                        inflated_data.insert(nindex);
//                    }
//                }
            }
        }


    }
    static uint8_t ii = 0;
    ii++;
    if (ii >= 2) {
        ii = 0;

        for (size_t i = 0; i < msg.info.width * msg.info.height; i++) {
            if (inflated_data1[i] > 0) {
                temp_msg.data[i] = inflated_data1[i];
            }
        }
        pub->publish(temp_msg);
    }

    if (!read_flag) {
        std::swap(inflated_data1, inflated_data2);
//        inflated_data1.swap(inflated_data2);
    }
}

std::vector<int8_t> Map2use::inflate(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {

    inflated_data1 = new int8_t[msg->info.width * msg->info.height];
    inflated_data2 = new int8_t[msg->info.width * msg->info.height];

//    double inflation_radius_use = inflation_radius_;
//    if (if_optize == 1) { inflation_radius_use = inflation_radius_Optmiz; }
    // std::cout<<"inflation_radius_use:!! "<<inflation_radius_use <<std::endl;
    const auto &data = msg->data;
    uint32_t width = msg->info.width;
    uint32_t height = msg->info.height;

    double resolution_ = 0.05;

    int inflation_grid = int(inflation_radius_ / resolution_);
    int esdf_inflation_grid = int(inflation_radius_Optmized / resolution_);

    std::vector<int8_t> inflated_grid = data; // Copy the original data

    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            uint32_t index = y * width + x;
            if (data[index] == 100) { // Assuming 100 represents "occupied"
                // Inflate around the occupied cell
                for (int i = -inflation_grid; i <= inflation_grid; ++i) {
                    for (int j = -inflation_grid; j <= inflation_grid; ++j) {

                        int64_t nx = static_cast<int64_t >(x) + j;
                        int64_t ny = static_cast<int64_t >(y) + i;
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            auto dis = sqrt(i * i + j * j);
                            size_t nindex = ny * width + nx;
                            if (data[nindex] != 100) {
                                if (dis <= esdf_inflation_grid) {
                                    inflated_grid[nindex] = 99;
                                } else if (dis <= inflation_grid && inflated_grid[nindex] != 99) {
                                    inflated_grid[nindex] = 90;
                                }
                            }

                        }
                    }
                }
            }
        }
    }

    return inflated_grid;

}


//获取地图点位占据信息
float Map2use::map_if_occupied(float pos_x, float pos_y) {
    //2024//4.1重大程序跟新 kh 。哈哈哈这个老6终于被发现了
    //判断是否超过了地图的边界，如果超过了的话，就直接停下
    // std::cout<<"pos:"<< pos_x<<","<<pos_y<<std::endl;
    //通过origon原点计算相对于数组的坐标。
    pos_x = pos_x - map_origin_x;
    pos_y = pos_y - map_origin_y;
    // int x=floor(pos_x*(1/map_resolution)); int y=floor(pos_y*(1/map_resolution));
    //  std::cout<<"map_resolution:"<< map_resolution<<std::endl;
    // std::cout<<"pos:"<< x<<","<< y <<std::endl;
    // std::cout<<"xfanwei:"<< map_origin_x<<","<<  map_origin_x+map_cols*map_resolution <<"xPos"<<pos_x<<std::endl;
    // std::cout<<"yfanwei:"<< map_origin_y<<","<<  map_origin_y+map_rows*map_resolution <<"xPos"<<pos_y<<std::endl;
    //    if((pos_x<fabs(map_cols*map_resolution))&&(pos_y<fabs(map_rows*map_resolution)))
    {
        //向下取整+获取具体的缩影数值
        int x = floor(pos_x * (1 / map_resolution));
        int y = floor(pos_y * (1 / map_resolution));
        // std::cout<<"pos:"<< x<<","<< y <<std::endl;
        //计算其中的数值是怎么回事
        int which_num = x + y * map_cols;
        float map_date = mapdata[which_num];
//        sem_reader.x;
//        sem_reader.acquire();
//        if (inflated_data.find(static_cast<size_t>(which_num)) != inflated_data.end()) {
//            return 1.0f;
//        }
        // auto path_start_cnt = std::chrono::high_resolution_clock::now();
        read_flag = true;
        float occ_dynamic = inflated_data2[which_num] * 0.01;
        read_flag = false;
        if (occ_dynamic > 0.5) { return occ_dynamic; }


        // auto path_end_cnt = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> duration = path_end_cnt - path_start_cnt;
        // std::chrono::duration<double, std::milli> ms = duration;
        // std::cout << "[kino replan] Search_time：" << ms.count() << " ms" << std::endl;

//        sem_writer.release();
        //标定选中的地图位置
        // map(which_num)=9;
        if (map_date <= 0)map_date = 0;
        // std::cout<<"map_datein:"<< map_date*0.01f<<std::endl;
        return map_date * 0.01f; //返回数据值得大小。
    }
    //    else 
    //     return 1;//地图边界之外的直接赋值为0
}

Eigen::Vector2d Map2use::CalcuDirVector(float x, float y) {
    float maxDistance = Esdf_CheakDis + 0.01f; // meters
    int maxSteps = (float(maxDistance / map_resolution) / Esdf_CheakUpSteepDiv); // convert meters to grid steps
    float stepdis = map_resolution * Esdf_CheakUpSteepDiv;
    float ver_x = 0;
    float ver_y = 0;
    for (int angle = -180; angle < 180; angle += Esdf_CheakAngle) {
        float rad = angle / 57.29f;
        float dx = cos(rad) * stepdis;
        float dy = sin(rad) * stepdis;
        for (int step = 1; step <= maxSteps; ++step) {
            if (step == maxSteps || map_if_occupied(x + dx * step, y + dy * step) >= 0.91f) {
                ver_x += (step - maxSteps) * dx;
                ver_y += (step - maxSteps) * dy;
                break;
            }
        }
    }
    Eigen::Vector2d direction(ver_x, ver_y);
    return direction;
}

std::vector<Eigen::Vector2f> Map2use::CalcuDirVectorTest1(float x, float y) {
    std::vector<Eigen::Vector2f> vectors;
    float maxDistance = Esdf_CheakDis + 0.01f; // meters
    int maxSteps = (float(maxDistance / map_resolution) / Esdf_CheakUpSteepDiv); // convert meters to grid steps
    float stepdis = map_resolution * Esdf_CheakUpSteepDiv;
    for (int angle = -180; angle < 180; angle += Esdf_CheakAngle) {
        float rad = angle / 57.29f;
        float dx = cos(rad) * stepdis;
        float dy = sin(rad) * stepdis;
        // std::cout << "sin= " << sin(rad)<< "cos=" <<cos(rad)<<std::endl;
        for (int step = 1; step <= maxSteps; ++step) {
            if (step == maxSteps || map_if_occupied(x + dx * step, y + dy * step) >= 0.5f) {
                Eigen::Vector2f direction(dx, dy);
                vectors.push_back(direction * step);
                break;
            }
        }
        //std::cout << "Angle= " << angle << "dis=" << step*map_resolution <<"testvic=" << testvic.norm() <<std::endl; 
    }
    return vectors;
}

//地图使用终端打印
void Map2use::map_Teminal_cout() {
    //     std::cout << "地图：" << std::endl;
    // for (int i = 0; i < map_size; ++i) {
    //     std::cout << map(i) << " ";
    //     if ((i + 1) % 30 == 0) {
    //         std::cout << std::endl;
    //     }
    // }
}
