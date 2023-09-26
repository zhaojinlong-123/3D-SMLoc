#include <iostream>
#include<string>
#include <pcl/io/pcd_io.h>                    
#include <pcl/point_types.h>                         
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
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
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
typedef pcl::PointXYZRGB PointType;
int imageRowIncrease=1;
int imageColIncrease=1;
float pointCloudLeafSize=0.2;
int line_num = 20;
float inner_part = 0.1;
int line_point_num = 20;
float dist = 0.05;
pcl::PointCloud<PointType>::Ptr cameraFrameCloud(new pcl::PointCloud<PointType>());
void fitMultipleLines(pcl::PointCloud<PointType>::Ptr cloud, std::vector<pcl::ModelCoefficients>& lineCoff, int frameid)
{
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::SACSegmentation<PointType> seg;               // 创建拟合对象
	seg.setOptimizeCoefficients(true);                     // 设置对估计模型参数进行优化处理
	seg.setModelType(pcl::SACMODEL_LINE);                  // 设置拟合模型为直线模型
	seg.setMethodType(pcl::SAC_RANSAC);                    // 设置拟合方法为RANSAC
	seg.setMaxIterations(1000);                             // 设置最大迭代次数
	seg.setDistanceThreshold(0.05);                       // 判断是否为模型内点的距离阀值/设置误差容忍范围

	int i = 0, nr_points = cloud->points.size();
	int k = 0;
    pcl::PointCloud<PointType>::Ptr whole_cloud_line(new pcl::PointCloud<PointType>);
	while (k < line_num && cloud->points.size() > 0.1 * nr_points)// 从0循环到5执行6次，并且每次cloud的点数必须要大于原始总点数的0.1倍
	{
		pcl::ModelCoefficients coefficients;
		seg.setInputCloud(cloud);                         // 输入点云						 
		seg.segment(*inliers, coefficients);              // 内点的索引，模型系数

		pcl::PointCloud<PointType>::Ptr cloud_line(new pcl::PointCloud<PointType>);
		pcl::PointCloud<PointType>::Ptr outside(new pcl::PointCloud<PointType>);
		if (inliers->indices.size() > 20) // 判断提取直线上的点数是否小于20个点，小于的话该直线不合格
		{
			lineCoff.push_back(coefficients);             // 将参数保存进vector中
			pcl::ExtractIndices<PointType> extract;   // 创建点云提取对象
			extract.setInputCloud(cloud);
			extract.setIndices(inliers);
			extract.setNegative(false);                   // 设置为false，表示提取内点
			extract.filter(*cloud_line);

			extract.setNegative(true);                    // true提取外点（该直线之外的点）
			extract.filter(*outside);                     // outside为外点点云
			cloud.swap(outside);                          // 将cloud_f中的点云赋值给cloud
		}
		else
		{
			PCL_ERROR("Could not estimate a line model for the given dataset.\n");
			break;
		}
        *whole_cloud_line=*whole_cloud_line+*cloud_line;
		// std::stringstream ss;
		// ss << "line_" << i + 1 << ".pcd"; // 记录提取的是第几条直线，并以该序号命名输出点云
		// pcl::PCDWriter writer;
		// writer.write<PointType>(ss.str(), *cloud_line, false);

		i++;
		k++;
	}

    std::stringstream ss;
    ss <<"/home/zjl/Desktop/parking/avp_data/seg_line/" << frameid << ".pcd"; // 记录提取的是第几条直线，并以该序号命名输出点云
    pcl::PCDWriter writer;
    writer.write<PointType>(ss.str(), *whole_cloud_line, false);
    std::cout << "map save over" << std::endl;
    //-----------------点云可视化---------------
    // pcl::visualization::PCLVisualizer viewer;
    // viewer.addPointCloud(cloud, "cloud");  // 加载比对点云
    // pcl::visualization::PointCloudColorHandlerCustom<PointType> line_color(whole_cloud_line, 65, 183, 172);
    // viewer.addPointCloud(whole_cloud_line, line_color, "line");
    // //可视化拟合出来的直线
    // int count = 0;
    // for (auto l : lineCoff){
    // std::string co = std::to_string(count);
    // viewer.addLine(l,co);
    // count++;
    // }
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "line");
    // viewer.spin();
    // while (!viewer.wasStopped())
    // {
    //     viewer.spinOnce(100);
    //     // 可添加其他操作
    // }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_frameSyn");
    ros::NodeHandle nh1;
    nh1.param<float>("pointCloudLeafSize", pointCloudLeafSize, 0.1);
    nh1.param<int>("imageRowIncrease", imageRowIncrease, 1);
    nh1.param<int>("imageColIncrease", imageColIncrease, 1);
    nh1.param<int>("line_num", line_num, 20);
    nh1.param<float>("inner_part", inner_part, 0.1);
    nh1.param<float>("dist", dist, 0.1);
    nh1.param<int>("line_point_num", line_point_num, 20);
	// 加载点云
    for(int frameid = 99; frameid < 100; frameid = frameid + 1){
        std::ostringstream stringStream;
        stringStream << "/home/zjl/Desktop/parking/avp_data/whole_traj/" << frameid << ".png";
        cv::Mat image = cv::imread(stringStream.str(), CV_LOAD_IMAGE_COLOR);
        if (image.empty()) {
            ROS_ERROR_STREAM("Failed to capture image!");
            ros::shutdown();
        }
        int row=image.rows;
        int col=image.cols;
        cameraFrameCloud->clear();
        for(int i=0;i<row;i=i+imageRowIncrease){
        const uchar* p=image.ptr<uchar>(i);
        for(int j=0;j<col;j=j+imageColIncrease){
            int b=p[3*j];
            int g=p[3*j+1];
            int r=p[3*j+2];
            if(b==0 && g==0 && r==0){
                    continue;
                }
            PointType po;
            po.x = double(j-col/2)/100.0;
            po.y = double(-i+row/2)/100.0;
            po.z = 0;
            po.r=r;
            po.g=g;
            po.b=b;
            cameraFrameCloud->push_back(po);
            }
        }

        //filter point cloud, decrease number of point cloud 
        // pcl::PointCloud<PointType> cameraFrameCloudDS;
        // pcl::VoxelGrid<PointType> downSizeFilter;
        // downSizeFilter.setInputCloud(cameraFrameCloud);
        // downSizeFilter.setLeafSize(pointCloudLeafSize, pointCloudLeafSize, pointCloudLeafSize);
        // downSizeFilter.filter(cameraFrameCloudDS);
        // *cameraFrameCloud=cameraFrameCloudDS;

	    std::vector<pcl::ModelCoefficients> LinesCoefficients;
	    fitMultipleLines(cameraFrameCloud, LinesCoefficients, frameid);
        std::cout << "frameid: " << frameid << " " << "一共拟合出" << LinesCoefficients.size() << "条直线，直线系数分别为：\n" << std::endl;
        for (auto l : LinesCoefficients)
        {
            std::cout << l.values[0] << "," << l.values[1]
                << "," << l.values[2] << "," << l.values[3]
                << "," << l.values[4] << "," << l.values[5] << std::endl;
        }
    }
    ros::spin();
    std::cout<<"hello slam"<<std::endl;
	return 0;
}

