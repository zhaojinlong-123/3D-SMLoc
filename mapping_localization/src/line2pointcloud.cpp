#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
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
using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
typedef pcl::PointXYZRGB PointType;
std::string vectormap_path = "/home/zjl/Desktop/parking/avp_data/lidar_map_fast_lio/ZJG2_vector_map/points.txt";
void creatsegment(PointType& p1, PointType& p2, pcl::PointCloud<PointType> ::Ptr cloud, double delt) {    
    Eigen::Vector3d k(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
    Eigen::Vector3d kv1(-(p1.y - p2.y), p1.x - p2.x, p1.z - p2.z);
    //int width = 20;
    kv1 = kv1 / kv1.norm();
    int numw = 5;

    int num = k.norm() / delt;
    k = k / k.norm();

    for (auto w=-numw;w<numw+1;w++){
        PointType p2w,p1w;
        p2w.x = p2.x + delt*w*kv1[0];
        p2w.y = p2.y + delt*w*kv1[1];
        p2w.z = p2.z + delt*w*kv1[2];
        p2w.rgb = p2.rgb;
        p1w.x = p1.x + delt*w*kv1[0];
        p1w.y = p1.y + delt*w*kv1[1];
        p1w.z = p1.z + delt*w*kv1[2];
        p1w.rgb = p1.rgb;

        PointType po;
        for (auto i = 0; i < num; ++i) {
            po.x = p2w.x + delt * i * k[0];
            po.y = p2w.y + delt * i * k[1];
            po.z = p2w.z + delt * i * k[2];
            po.r = p2w.r;
            po.g = p2w.g;
            po.b = p2w.b;
            cloud->push_back(po);
        }
        cloud->push_back(p1w);
        cloud->push_back(p2w);
    }


}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "test_frameSyn");
    ros::NodeHandle nh1;
    // get parameter from config file 
    nh1.param<std::string>("vectormap_path", vectormap_path, "/home/zjl/Desktop/parking/avp_data/lidar_map_fast_lio/ZJG2_vector_map/points.txt");
    std::ostringstream txt_name;
    txt_name << vectormap_path;
    std::ifstream file(txt_name.str());
    std::string line;
    pcl::PointCloud<PointType>::Ptr vectormap(new pcl::PointCloud<PointType>);
    while(getline(file, line)){
        std::stringstream ss(line);
        PointType p1,p2;
        double x1,y1,x2,y2;
        ss >> x1;
        ss >> y1;
        ss >> x2;
        ss >> y2;
        p1.x = x1;
        p1.y = y1;
        p1.z = 0;
        p1.r = 128;
        p1.g = 128;
        p1.b = 0;
        p2.x = x2;
        p2.y = y2;
        p2.z = 0;
        p2.r = 128;
        p2.g = 128;
        p2.b = 0;
        creatsegment(p1, p2, vectormap, 0.01);
    }
    pcl::io::savePCDFile("vectormap_zjg.pcd", *vectormap);
    std::stringstream sk;
    sk <<"/home/zjl/avpslam_ws/vectormap_zjg2.pcd"; 
    pcl::PCDWriter writer;
    writer.write<PointType>(sk.str(), *vectormap, false);
    std::cout << "map save over" << std::endl;
    ros::spin();
    return 0;
}
