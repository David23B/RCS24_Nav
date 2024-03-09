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

my_msgs::Path flag_path;
geometry_msgs::Point cur_position;
std::vector<geometry_msgs::PoseStamped> global_path;
int way_point_index = 0;
// lzt:
bool if_new = false;
int before = 0;
int now = 0;

void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom){
    if(odom == nullptr) return;
    cur_position = odom->pose.pose.position;
}

void globalPathCallback(const nav_msgs::Path::ConstPtr& path){
    if(path == nullptr) return;
    global_path = path->poses;
}

void l_globalPathCallback(const my_msgs::Path::ConstPtr& Path){
    if(&Path->path == nullptr) return;
    global_path = Path->path.poses;
    now = Path->flag;
}

bool isNearby(geometry_msgs::Point p1, geometry_msgs::Point p2){
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2)) < 4;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "way_point");
    ros::NodeHandle nh;
    ros::Publisher waypoint_pub = nh.advertise<geometry_msgs::PointStamped>("/way_point", 5);
    ros::Subscriber init_pose_sub = nh.subscribe("/state_estimation", 1, odometryCallback);
    ros::Subscriber global_path_sub = nh.subscribe("/global_path", 1, globalPathCallback);
    ros::Subscriber l_global_path_sub = nh.subscribe("/l_path", 1, l_globalPathCallback);
    ros::Rate rate(100);
    while(ros::ok()){
    	if (now-before == 1)
    	{
    		if_new = true;
    		before = now;
    	}
    	else
    		if_new = false;
    	if (if_new)
    		way_point_index = 0;
    	ROS_INFO("if new: %d, now: %d, before: %d", if_new, now, before);
    	
        if(global_path.size() > 0){
            if(isNearby(cur_position, global_path[way_point_index].pose.position)){
                way_point_index++;
                if(way_point_index >= global_path.size()){
                    way_point_index = global_path.size() - 1;
                }
            }
            geometry_msgs::PointStamped way_point_msg;
            way_point_msg.header.stamp = ros::Time::now();
            way_point_msg.header.frame_id = "map";
            way_point_msg.point = global_path[way_point_index].pose.position;
            waypoint_pub.publish(way_point_msg);
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
