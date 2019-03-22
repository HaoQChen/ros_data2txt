#include <fstream>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <signal.h>
#include <tf/tf.h>

#include <sensor_msgs/LaserScan.h>

using namespace std;

static std::ofstream out_file;


void MySigintHandler(int sig)
{
    //这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
    ROS_INFO("laser_scan2txt shutting down!");
    ros::shutdown();
    out_file.close();
}


void SubLaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    static double begin_time = msg->header.stamp.toSec();
    out_file << (msg->header.stamp.toSec() - begin_time) << ", ";

    out_file << msg->ranges.size() << ", ";

    out_file << msg->angle_min << ", ";
    out_file << msg->angle_max << ", ";
    
    out_file << msg->range_min << ", ";
    out_file << msg->range_max << ", ";

    for(int i = 0; i < msg->ranges.size(); ++i){
        out_file << msg->ranges[i] << ", ";
    }

    out_file << endl;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_scan2txt");
    ros::NodeHandle nh("~");

    signal(SIGINT, MySigintHandler);

    std::string file_name;
    std::string topic_name;
    nh.param<std::string>("file_name", file_name, "./laser_scan_data.txt");
    nh.param<std::string>("topic_name", topic_name, "/scan");

    out_file.open(file_name, ios::out);
    if(!out_file.is_open()){
        cout << "open file failed!" << endl;
        return -1;
    }else{
        out_file << "time" << ", ";

        out_file << "size" << ", ";

        out_file << "angle_min" << ", ";
        out_file << "angle_max" << ", ";

        out_file << "range_min" << ", ";
        out_file << "range_max" << ", ";

        out_file << "range data" << endl;
    }

    ros::Subscriber sub_scan = nh.subscribe(topic_name, 50, SubLaserScanCallback);

    ros::spin();
    return 0;
}
