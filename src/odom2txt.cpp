#include <fstream>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <signal.h>
#include <tf/tf.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
using geometry_msgs :: Quaternion;
using geometry_msgs :: Pose;

using namespace std;


static std::ofstream out_file;


void MySigintHandler(int sig)
{
    //这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
    ROS_INFO("odom2txt shutting down!");
    ros::shutdown();
    out_file.close();
}


void SubOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    static double begin_time = msg->header.stamp.toSec();
    out_file << (msg->header.stamp.toSec() - begin_time) << ", ";

    out_file << msg->pose.pose.position.x << ", ";
    out_file << msg->pose.pose.position.y << ", ";
    
    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, tf_quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    out_file << yaw << endl;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom2txt");
    ros::NodeHandle nh("~");

    signal(SIGINT, MySigintHandler);

    std::string file_name;
    std::string topic_name;
    nh.param<std::string>("file_name", file_name, "./odom_data.txt");
    nh.param<std::string>("topic_name", topic_name, "/odom");

    out_file.open(file_name, ios::out);
    if(!out_file.is_open()){
        cout << "open file failed!" << endl;
        return -1;
    }else{
        out_file << "time" << ", ";

        out_file << "position.x" << ", ";
        out_file << "position.y" << ", ";

        out_file << "yaw" << endl;
    }

    ros::Subscriber sub_odom = nh.subscribe(topic_name, 50, SubOdomCallback);

    ros::spin();
    return 0;
}
