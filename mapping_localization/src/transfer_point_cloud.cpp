#include <iostream>
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv/cv.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mutex>
#include <queue>
#include <cmath>
#include <string>
typedef pcl::PointXYZ PointType;
pcl::PointCloud<PointType>::Ptr currFeatureCloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr globalFeatureCloud(new pcl::PointCloud<PointType>());
std::string mapSaveLocation="/home/zjl/Desktop/parking/avp_data/lidar_map_fast_lio/YT/1648095563.223131657.pcd";
std::string mapSaveLocation2="/home/zjl/Desktop/parking/avp_data/lidar_map_fast_lio/YT/lego-loam-world.pcd";
Eigen::Affine3f transWorldCurrent;
//   transform point cloud according to pose
pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, Eigen::Affine3f& transCur)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            // cloudOut->points[i].r = pointFrom.r;
            // cloudOut->points[i].g = pointFrom.g;
            // cloudOut->points[i].b = pointFrom.b;

        }
        return cloudOut;
    }
int main(int argc, char *argv[]){

    ros::init(argc, argv, "test_odom");
    ros::NodeHandle nh;
   

    // load prior global map
    currFeatureCloud->clear();
    std::cout<<"load map begin ******************"<<std::endl;
    pcl::io::loadPCDFile(mapSaveLocation, *currFeatureCloud);
    std::cout<<"load map over ******************"<<std::endl;
    Eigen::Matrix4f transform;
    transform << 0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,1;
    transWorldCurrent = transform;
    *globalFeatureCloud =*transformPointCloud(currFeatureCloud, transWorldCurrent);
    pcl::io::savePCDFileASCII(mapSaveLocation2, *globalFeatureCloud);
    return 0;
}