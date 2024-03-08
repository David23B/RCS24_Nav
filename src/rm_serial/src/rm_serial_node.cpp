#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <serial/serial.h>
#include <iomanip>
#include <sstream>

serial::Serial ser;

std::string formatData(double value) {
    std::stringstream ss;
    int intValue = static_cast<int>(value * 1000);  // 将值转换为整数（毫米）
    ss << std::fixed << std::setfill('0') << std::setw(3) << std::abs(intValue);
    return ss.str().substr(0, 3);  // 确保字符串长度为3
}

void write_callback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    std::string data;
    data += "A";

    // 处理linear.x的符号位和数据
    data += (msg->twist.linear.x < 0) ? "1" : "0";
    data += formatData(msg->twist.linear.x);

    // 处理linear.y
    data += (msg->twist.linear.y < 0) ? "1" : "0";
    data += formatData(msg->twist.linear.y);

    // 处理angular.z
    data += (msg->twist.angular.z < 0) ? "1" : "0";
    data += formatData(msg->twist.angular.z);
    data += "a";
    // 打印data到终端
    ROS_INFO("Data to be sent: %s", data.c_str());

    ser.write(data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "rm_serial");
    ros::NodeHandle nh;

    std::string serial_port;
    nh.param<std::string>("/rm_serial/serial_port", serial_port, "/dev/ttyUSB0");

    // 打开串口
    try{
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port " << serial_port);
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    } else {
        return -1;
    }

    // 订阅 /cmd_vel
    ros::Subscriber write_sub = nh.subscribe("/cmd_vel", 1000, write_callback);

    ros::spin();
}

