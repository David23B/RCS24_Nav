#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <queue>
#include <my_msgs/Path.h>

const int cost = 1;

my_msgs::Path flag_path;
std::vector<std::vector<int>> map;

int width, height;
float resolution;
int origin_x, origin_y;
bool isEndPointUpdated = false;

struct Point{
    int x,y;
    int g,h,f;
    Point* parent;

    Point(int x = -1, int y = -1, int g = 0, int h = 0, int f = 0) 
        : x(x), y(y), g(g), h(h), f(f), parent(nullptr) {}
};
Point startPoint,endPoint;

void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    ROS_INFO("Map is loading...");
    width = msg->info.width;                  // 地图的宽度，单位：像素
    height = msg->info.height;                // 地图的高度，单位：像素
    resolution = msg->info.resolution;        // 地图的分辨率，单位：米/像素
    origin_x = msg->info.origin.position.x / resolution;   // 地图的原点x坐标，单位：像素
    origin_y = msg->info.origin.position.y / resolution;   // 地图的原点y坐标，单位：像素
    ROS_INFO("Map is loaded! Map info: width = %d, height = %d, resolution = %f, origin_x = %d, origin_y = %d", width, height, resolution, origin_x, origin_y);
    // 清除map中的数据
    map.clear();
    
    for(int i=0;i<height;i++) map.push_back(std::vector<int>(width));
    for(int i = 0; i < height; i++)
    {
        for(int j = 0; j < width; j++)
        {
            // 计算当前单元格在data数组中的索引
            int index = i * width + j;

            // 获取当前单元格的值
            int value = msg->data[index];

            // 将当前单元格的值存储到二维向量中
            map[i][j] = value;
        }
    }
	
    // 膨胀处理

    // //将map打印到txt文件中
    // FILE *fp;
    // fp = fopen("/home/jason/Project/github/RCS24_Nav/map.txt", "w");
    // for(int i=0;i<height;i++){
    //     for(int j=0;j<width;j++){
    //         fprintf(fp, "%d ", map[i][j]);
    //     }
    //     fprintf(fp, "\n");
    // }
    // fclose(fp);
}

std::vector<Point*> getNeighbors(Point* p){
    std::vector<Point*> neighbors;
    for(int i=-1;i<=1;i++){
        for(int j=-1;j<=1;j++){
            if(i == 0 && j == 0) continue;
            if(i != 0 && j != 0) continue;
            neighbors.push_back(new Point(p->x + i, p->y + j));
        }
    }
    return neighbors;
}

std::vector<Point*> getPath(Point* start, Point* end){
    ros::Time start_time = ros::Time::now();
    // 初始化table，0-未访问，-1-已访问（在closelist中），>0-代价g
    std::vector<std::vector<int>> table(height, std::vector<int>(width));
    for(auto& row : table) {
        std::fill(row.begin(), row.end(), 0);
    }

    std::priority_queue<Point*, std::vector<Point*>, bool(*)(Point*, Point*)> openList([](Point* a, Point* b){return a->f > b->f;});
    openList.push(start);
    table[start->y][start->x] = start->g;

    while(!openList.empty()){
        // 取出f值最小的点
        Point* current = openList.top();
        
        // 将当前点放入closelist中
        openList.pop();
        table[current->y][current->x] = -1;

        // 判断当前点是否目标点
        if(current->x == end->x && current->y == end->y){
            ros::Duration duration = ros::Time::now() - start_time;
            ROS_INFO("Time taken by found path: %f", duration.toSec());
            // 回溯路径
            end->parent = current;
            std::vector<Point*> path;
            Point* p = end;
            while(p){
                path.push_back(p);
                p = p->parent;
            }
            // ROS_INFO("path's length is : %ld", path.size());
            return path;
        }

        // 获取周围点
        std::vector<Point*> neighbors = getNeighbors(current);

        // 遍历周围点
        for(auto neighbor : neighbors){
            // 判断是否在网格内
            if(neighbor->x < 0 || neighbor->x >= width || neighbor->y < 0 || neighbor->y >= height) continue;

            // 判断是否是障碍物
            if(map[neighbor->y][neighbor->x] == 100) continue;

            // 判断是否在closelist中
            if(table[neighbor->y][neighbor->x] == -1) continue;

            // 计算代价
            neighbor->g = current->g + cost;
            neighbor->h = abs(neighbor->x - end->x) + abs(neighbor->y - end->y);
            neighbor->f = neighbor->g + neighbor->h;
            neighbor->parent = current;

            // 判断否在openlist中，如果在，需要比较当前是否更优
            if(table[neighbor->y][neighbor->x] > 0){
                if(neighbor->g < table[neighbor->y][neighbor->x]){
                    table[neighbor->y][neighbor->x] = neighbor->g;
                    neighbor->parent = current;
                }
            }
            // 如果不在openlist中，将其放入openlist
            else{
                openList.push(neighbor);
                table[neighbor->y][neighbor->x] = neighbor->g;
            }

        }
        
    }
    ROS_INFO("Path not found!");
    return {};
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom){
    if(odom == nullptr) return;
    startPoint.y = odom->pose.pose.position.y / resolution - origin_y;
    startPoint.x = odom->pose.pose.position.x / resolution - origin_x;
    // ROS_INFO("Init pose: x = %d, y = %d", startPoint.x, startPoint.y);
}

void GoalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if(msg == nullptr) return;
    // 目标点在图外面或者在障碍物上
    if(msg->pose.position.x/resolution < origin_x || msg->pose.position.x/resolution >= origin_x+width || msg->pose.position.y/resolution < origin_y || msg->pose.position.y/resolution >= origin_y+height){
        ROS_INFO("Goal pose is out of map!");
        return;
    }
    if(map[msg->pose.position.y/resolution - origin_y][msg->pose.position.x/resolution - origin_x] == 100){
        ROS_INFO("Goal pose is on the obstacle!");
        return;
    }
    endPoint.y = msg->pose.position.y / resolution - origin_y;
    endPoint.x = msg->pose.position.x / resolution - origin_x;
    startPoint.h = (abs(startPoint.x - endPoint.x) + abs(startPoint.y - endPoint.y))/resolution;
    isEndPointUpdated = true;
    
    //lzt:
    flag_path.flag++;
    // ROS_INFO("Goal posi: x = %f, y = %f", msg->pose.position.x, msg->pose.position.y);
    // ROS_INFO("Goal pose: x = %d, y = %d", endPoint.x, endPoint.y);
}

int main(int argc, char** argv){
    // lzt:
    flag_path.flag = 0;
    
    ros::init(argc, argv, "global_planner");
    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe("/map", 1, MapCallback);
    ros::Subscriber init_pose_sub = nh.subscribe("/state_estimation", 1, odometryCallback);
    ros::Subscriber goal_pose_sub = nh.subscribe("/move_base_simple/goal", 1, GoalPoseCallback);
    ros::Publisher fir_path_pub = nh.advertise<nav_msgs::Path>("/global_path", 5);
    ros::Publisher path_pub = nh.advertise<my_msgs::Path>("/l_path", 5);

    ros::Rate loop_rate(10);
    
    while(ros::ok()){
    if(map.size() > 0){
        if(startPoint.x != -1 && startPoint.y != -1 && endPoint.x != -1 && endPoint.y != -1 && isEndPointUpdated){
            std::vector<Point*> path = getPath(&startPoint, &endPoint);
            std::vector<Point*> my_path = path;

            if(my_path.size() > 0){
                // path publisher
                nav_msgs::Path path_msg;
                path_msg.header.stamp = ros::Time::now();
                path_msg.header.frame_id = "map";
                for(auto it = my_path.rbegin(); it != my_path.rend(); it++){
                    geometry_msgs::PoseStamped pose;
                    pose.header.stamp = ros::Time::now();
                    pose.header.frame_id = "map";
                    pose.pose.position.x = ((*it)->x + origin_x) * resolution;
                    pose.pose.position.y = ((*it)->y + origin_y) * resolution;
                    pose.pose.position.z = 0;
                    path_msg.poses.push_back(pose);
                }
                
                // lzt:
                flag_path.path = path_msg;
                //flag_path.flag = false; 
                
                fir_path_pub.publish(path_msg);
                path_pub.publish(flag_path);
                
            }
            isEndPointUpdated = false;
        }
    }
    ros::spinOnce();
    loop_rate.sleep();
}

    return 0;

}
