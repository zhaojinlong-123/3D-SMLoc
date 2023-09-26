#include <iostream>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
typedef pcl::PointXYZRGB PointType;
std::string ransac_file_directory;
std::string ransac_file_name;
float pointCloudLeafSize=0.2;
bool transfer_surround = true;
float tx =0;
float ty =0;
float tz =0;
float troll=0;
float tpitch=0;
float tyaw=0;
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
            cloudOut->points[i].r = pointFrom.r;
            cloudOut->points[i].g = pointFrom.g;
            cloudOut->points[i].b = pointFrom.b;

        }
        return cloudOut;
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

pcl::PointCloud<PointType>::Ptr
    pcd_cloud_color(new pcl::PointCloud<PointType>);

pcl::PointCloud<PointType>::Ptr
    parkingline_cloud(new pcl::PointCloud<PointType>);
int main(int argc, char** argv) {

    ros::init(argc, argv, "color");  //初始化了一个节点，名字为color
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");
    n.param<float>("pointCloudLeafSize", pointCloudLeafSize, 0.2);
    n.param<bool>("transfer_surround", transfer_surround, true);
    n.param<float>("tx", tx, 0);
    n.param<float>("ty", ty, 0);
    n.param<float>("tz", tz, 0);
    n.param<float>("troll", troll, 0);
    n.param<float>("tpitch", tpitch, 0);
    n.param<float>("tyaw", tyaw, 0);

    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/cameraCloudFrame", 100);  //建立了一个发布器，主题是color_cloud，方便之后发布点云
    std::ifstream in("/home/zjl/avpslam_ws/color.txt");
    std::string line;
    ros::Rate rate(10); 
    for(int i = 99; i < 3570; i = i + 1)
    {
        pcd_cloud_color->clear();
        parkingline_cloud->clear();
        if(!n.ok())
            break;
        std::ostringstream stringStream;
        stringStream << "/home/zjl/Desktop/parking/avp_data/seg_line_connection/" << i << ".pcd";
        // std::string pcd_file;
        // pcd_file = "/home/zjl/Desktop/parking/avp_data/seg_line/" + i + ".pcd";
        if (pcl::io::loadPCDFile<PointType>(stringStream.str(), *pcd_cloud_color) == -1) {
            PCL_ERROR("Couldn't read file: %s \n", stringStream.str());
            return (-1);
        }  
        //filter parking line
        // for(int i=0; i<pcd_cloud_color->points.size();i++){
        //     if(pcd_cloud_color->points[i].r==128 && pcd_cloud_color->points[i].g==128 && pcd_cloud_color->points[i].b==0){
        //         parkingline_cloud->push_back(pcd_cloud_color->points[i]);
        //     }
        // }        

        //filter point cloud, decrease number of point cloud 
        pcl::PointCloud<PointType> cameraFrameCloudDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(pcd_cloud_color);
        downSizeFilter.setLeafSize(pointCloudLeafSize, pointCloudLeafSize, pointCloudLeafSize);
        downSizeFilter.filter(cameraFrameCloudDS);
        *pcd_cloud_color=cameraFrameCloudDS;

        int cameraPointSize=pcd_cloud_color->points.size();
        std::cout<<"point size is"<<cameraPointSize<<std::endl;

        if(transfer_surround){
            Eigen::Affine3f transCurrent=pcl::getTransformation(tx,ty,tz,troll,tpitch,tyaw);
            *pcd_cloud_color = *transformPointCloud(pcd_cloud_color, transCurrent);
        }

        std::getline(in,line);
        //std::cout<<"time: "<<string2Time(line)<<std::endl;
        sensor_msgs::PointCloud2 output_msg;  //建立一个可以直接发布的点云
        pcl::toROSMsg(*pcd_cloud_color, output_msg);
        output_msg.header.frame_id="camera0_link";
        output_msg.header.stamp=string2Time(line);
        pub.publish(output_msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}