#include <fstream>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <signal.h>
#include <tf/tf.h>

using namespace std;


static std::ofstream out_file;


void MySigintHandler(int sig)
{
    //这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
    ROS_INFO("imu2txt shutting down!");
    ros::shutdown();
    out_file.close();
}


void SubImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    static double begin_time = msg->header.stamp.toSec();
    out_file << (msg->header.stamp.toSec() - begin_time) << ", ";

    out_file << msg->linear_acceleration.x << ", ";
    out_file << msg->linear_acceleration.y << ", ";
    out_file << msg->linear_acceleration.z << ", ";

    out_file << msg->angular_velocity.x << ", ";
    out_file << msg->angular_velocity.y << ", ";
    out_file << msg->angular_velocity.z << ", ";

    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(msg->orientation, tf_quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    out_file << roll << ", ";
    out_file << pitch << ", ";
    out_file << yaw << endl;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu2txt");
    ros::NodeHandle nh("~");

    signal(SIGINT, MySigintHandler);

    std::string file_name;
    std::string topic_name;
    nh.param<std::string>("file_name", file_name, "./imu_data.txt");
    nh.param<std::string>("topic_name", topic_name, "/imu");

    out_file.open(file_name, ios::out);
    if(!out_file.is_open()){
        cout << "open file failed!" << endl;
        return -1;
    }else{
        out_file << "time" << ", ";

        out_file << "linear_acceleration.x" << ", ";
        out_file << "linear_acceleration.y" << ", ";
        out_file << "linear_acceleration.z" << ", ";

        out_file << "angular_velocity.x" << ", ";
        out_file << "angular_velocity.y" << ", ";
        out_file << "angular_velocity.z" << ", ";

        out_file << "roll" << ", ";
        out_file << "pitch" << ", ";
        out_file << "yaw" << endl;
    }

    ros::Subscriber sub_imu = nh.subscribe(topic_name, 50, SubImuCallback);

    ros::spin();
    return 0;
}
