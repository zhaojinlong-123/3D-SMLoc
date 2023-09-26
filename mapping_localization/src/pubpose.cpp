#include <ros/ros.h>
#include <stdlib.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Path.h>
#include <fstream>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
float licurrentX=0;
float licurrentY=0;
float licurrentZ=0;
float licurrentRoll=0;
float licurrentPitch=0;
float licurrentYaw=0;
long double time_stamp=0;
long double time_stamp2=0;
float licurrentX2=0;
float licurrentY2=0;
float licurrentZ2=0;
std::string localization_folder;
ros::Publisher pubCurrentPose;
ros::Publisher pubgtpose;
int scene;
int start_frame = 99;
int end_frame = 3570;

#define MAX_LINE 8*13153 

long double buf[MAX_LINE];
void readposes(){
    int i; 
    std::string trajpath = localization_folder+"2.txt";
    FILE *fp=fopen(trajpath.c_str(),"r");
    if(fp==NULL) {
        printf(" Open file Error !\n");
        exit(0);
    }
    for(i=0;i<MAX_LINE;i++){
        fscanf(fp,"%Lf",&buf[i]);
    }
}

long double buf2[MAX_LINE];
void readposes2(){
    int i; 
    std::string trajpath2 = localization_folder+"estimation.txt";
    FILE *fp2=fopen(trajpath2.c_str(),"r");
    if(fp2==NULL) {
        printf(" Open file Error !\n");
        exit(0);
    }
    for(i=0;i<MAX_LINE;i++){
        fscanf(fp2,"%Lf",&buf2[i]);
    }
}

int main(int argc, char** argv)
{
        ros::init(argc, argv, "pose_publisher");
        ros::NodeHandle nh;
        nh.param<int>("start_frame", start_frame, 99);
        nh.param<int>("end_frame", end_frame, 3570);
        nh.param<int>("scene", scene, 1);
        nh.param<std::string> ("/localization_folder", localization_folder, "/home/zjl/Desktop/parking/avp_data/");
        pubCurrentPose = nh.advertise<nav_msgs::Odometry>("/rawPose", 100);
        pubgtpose = nh.advertise<nav_msgs::Path>("/gtpose", 1000);
        readposes();
        readposes2();
        ros::Rate loop_rate(10);
        nav_msgs::Path path_msg;
        path_msg.header.stamp=ros::Time::now();
        path_msg.header.frame_id="world";

        for(int i = start_frame-11400; i < end_frame-11400; i = i + 1) //99-600 820-1920 820-1220 99-1220
        {
            time_stamp2 = buf2[8*i];
            licurrentX2 = buf2[8*i+1];
            licurrentY2 = buf2[8*i+2];
            licurrentZ2 = buf2[8*i+3];
            Eigen::Quaterniond q2;
            q2.x() = buf2[8*i+4];
            q2.y() = buf2[8*i+5];
            q2.z() = buf2[8*i+6];
            q2.w() = buf2[8*i+7];
            geometry_msgs::PoseStamped optimized_odom;
            optimized_odom.header.stamp = ros::Time().fromSec(time_stamp2);
            optimized_odom.header.frame_id = "world";
            //odomAftMapped.header.stamp = ros::Time::now();
            optimized_odom.pose.position.x = licurrentX2;
            optimized_odom.pose.position.y = licurrentY2;
            optimized_odom.pose.position.z = licurrentZ2;
            optimized_odom.pose.orientation.w = q2.w();
            optimized_odom.pose.orientation.x = q2.x();
            optimized_odom.pose.orientation.y = q2.y();
            optimized_odom.pose.orientation.z = q2.z();
            path_msg.poses.push_back(optimized_odom);
        } 
        int flag = 0;
        while(nh.ok()){
        if(flag == 0){
        for(int i = start_frame-11400; i < end_frame-11400; i = i + 1) //99-600 820-1920 820-1220 99-1220
        {
            time_stamp = buf[8*i];
            licurrentX = buf[8*i+1];
            licurrentY = buf[8*i+2];
            licurrentZ = buf[8*i+3];
            Eigen::Quaterniond q;
            q.x() = buf[8*i+4];
            q.y() = buf[8*i+5];
            q.z() = buf[8*i+6];
            q.w() = buf[8*i+7];
            nav_msgs::Odometry odomAftMapped;
            odomAftMapped.header.frame_id = "/world";
            odomAftMapped.child_frame_id = "/camera0_link";
            //odomAftMapped.header.stamp = ros::Time::now();
            odomAftMapped.header.stamp = ros::Time().fromSec(time_stamp);
            odomAftMapped.pose.pose.orientation.x = q.x();
            odomAftMapped.pose.pose.orientation.y = q.y();
            odomAftMapped.pose.pose.orientation.z = q.z();
            odomAftMapped.pose.pose.orientation.w = q.w();
            odomAftMapped.pose.pose.position.x = licurrentX;
            odomAftMapped.pose.pose.position.y = licurrentY;
            odomAftMapped.pose.pose.position.z = licurrentZ;
            pubCurrentPose.publish(odomAftMapped);
            pubgtpose.publish(path_msg); 
            ros::spinOnce();
            loop_rate.sleep();
        }
        flag = 1;
        }

        pubgtpose.publish(path_msg);  
        ros::spinOnce();
        loop_rate.sleep();        
        }

}