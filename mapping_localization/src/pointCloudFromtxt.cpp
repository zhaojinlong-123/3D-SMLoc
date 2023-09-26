#include <iostream>
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include<opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <pcl/filters/voxel_grid.h>
#include <mutex>
#include <queue>
#include <cmath>
#include <vector>
#include <limits>
#include <cstdlib>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sstream>
#include <iomanip>
#include <fstream>
//  synchronism images of multi cameras according to timestamp
//  generate point cloud according to IPM principle
//  broadcast point cloud of current frame


ros::Publisher pubCameraCloudFrame;

typedef pcl::PointXYZRGB PointType;

pcl::PointCloud<PointType>::Ptr cameraFrameCloud(new pcl::PointCloud<PointType>());


float pointCloudLeafSize=0.2;

std::string txt_path = "/media/zjl/Elements SE/avp-dataset/processed_data/2022-3-24-11-57-11/parking_detection_txt/";

std::string num2str(int i)
{
        char ss[10];
		sprintf(ss,"%06d",i);
        return ss;
}

ros::Time string2Time(std::string& str)
{
        std::string sec_string = str.substr(0,10);
        std::string nsec_string = str.substr(11,6);
        while(nsec_string.length() < 9){
            nsec_string += "0";
        }
        return ros::Time(std::stoi(sec_string),std::stoi(nsec_string));
}

int main(int argc, char *argv[]){

    ros::init(argc, argv, "test_frameSyn");
    ros::NodeHandle nh1;

    // get parameter from config file 
    nh1.param<std::string>("txt_path", txt_path, "/media/zjl/Elements SE/avp-dataset/processed_data/2022-3-24-11-57-11/parking_detection_txt/");
    nh1.param<float>("pointCloudLeafSize", pointCloudLeafSize, 0.1);
    image_transport::ImageTransport it(nh1);

    pubCameraCloudFrame = nh1.advertise<sensor_msgs::PointCloud2>("/cameraCloudFrame", 100);
    ros::Rate rate(10);
    std::ifstream in("/home/zjl/avpslam_ws/front.txt");
    std::string linetime;
    double col = 1104;
    double row = 1457;
    for(int i = 99; i < 3570; i = i + 1) {
        if(!nh1.ok())
            break;
        std::ostringstream txt_name;
        txt_name << txt_path << num2str(i) << ".txt";
        std::ifstream file(txt_name.str());
        cameraFrameCloud->clear();
        std::string line;
        while(getline(file, line)){
            std::stringstream ss(line);//点云赋值
            PointType po;
            double confidence;
            double imgi;
            double imgj;
            ss >> imgi;
            ss >> imgj;
            ss >> confidence;
            po.x = double(imgi-col/2)/100.0;
            po.y = double(-imgj+row/2)/100.0;
            po.z = 0;
            po.r = 128;
            po.g = 0;
            po.b = 0;
            if(confidence>0.8){
                cameraFrameCloud->push_back(po);
            }
        }
        file.close();
        std::getline(in,linetime);
        //filter point cloud, decrease number of point cloud 
        // pcl::PointCloud<PointType> cameraFrameCloudDS;
        // pcl::VoxelGrid<PointType> downSizeFilter;
        // downSizeFilter.setInputCloud(cameraFrameCloud);
        // downSizeFilter.setLeafSize(pointCloudLeafSize, pointCloudLeafSize, pointCloudLeafSize);
        // downSizeFilter.filter(cameraFrameCloudDS);
        // *cameraFrameCloud=cameraFrameCloudDS;

        // broad point cloud of current frame
        int cameraPointSize=cameraFrameCloud->points.size();
        //std::cout<<"point size is"<<cameraPointSize<<std::endl;
        sensor_msgs::PointCloud2 cameraCloudFrameMsg;
        pcl::toROSMsg(*cameraFrameCloud, cameraCloudFrameMsg);
        cameraCloudFrameMsg.header.stamp = string2Time(linetime);
        cameraCloudFrameMsg.header.frame_id = "/camera0_link";
        pubCameraCloudFrame.publish(cameraCloudFrameMsg);
        ros::spinOnce();
        rate.sleep();
    }
    in.close();

    std::cout<<"hello slam"<<std::endl;
    return 0;
}