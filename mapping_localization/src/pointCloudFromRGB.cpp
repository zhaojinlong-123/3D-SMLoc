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
#include <pcl/filters/passthrough.h>                 //直通滤波器头文件

//  synchronism images of multi cameras according to timestamp
//  generate point cloud according to IPM principle
//  broadcast point cloud of current frame


ros::Publisher pubCameraCloudFrame;

typedef pcl::PointXYZRGB PointType;

pcl::PointCloud<PointType>::Ptr cameraFrameCloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cloud_PassThrough(new pcl::PointCloud<PointType>());
std::queue<sensor_msgs::ImageConstPtr> cameraImageBuf;


std::mutex mBuf;


cv_bridge::CvImageConstPtr image_ptr;


int imageRowIncrease=1;
int imageColIncrease=1;
float pointCloudLeafSize=0.2;
int imagecutrow = 200;
int imagecutrow1 = 200;
int imagecutcol = 200;
bool usewhite = false;
bool refacity = false;
bool transfer_surround = true;
bool dynacut = true;
float thre_low=0;
float thre_high=0;
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

void cameraImageHandler(const sensor_msgs::ImageConstPtr& imageMsg){
    ros::Time start = ros::Time::now();
    image_ptr = cv_bridge::toCvShare(imageMsg,"bgr8");
    int row=image_ptr->image.rows;
    int col=image_ptr->image.cols;
    ros::Time time_stamp = image_ptr->header.stamp;

    cameraFrameCloud->clear();
    for(int i=0+imagecutrow1;i<row-imagecutrow;i=i+imageRowIncrease){
    const uchar* p=image_ptr->image.ptr<uchar>(i);
        for(int j=0+imagecutcol;j<col-imagecutcol;j=j+imageColIncrease){

            int b=p[3*j];
            int g=p[3*j+1];
            int r=p[3*j+2];

            if(b==0 && g==0 && r==0){
                    continue;
                }
            if(usewhite){
            if(r<50&&g>50&b<50){
                r=65;//r=65;
                g=183;//g=183;
                b=172;//b=172;
            }
            if(r>50&&g<50&b<50){
                r=36;// r=247;
                g=85;// g=77;
                b=172;// b=77;
            }
            if(r>50&&g>50&b<50){
                r=255;// r=255;
                g=165;// g=165;
                b=16;// b=16;
            }
            if(r<50&&g<50&b>50){
                r=0;// r=12;
                g=44;// g=132;
                b=83;// b=198;
            }
            }
            PointType po;
            po.x = double(j-col/2)/100.0;
            po.y = double(-i+row/2)/100.0;
            if(refacity){
            if(po.x>0){
                po.y=po.y+0.2;
            }
            }
            po.z = 0;
            po.r=r;
            po.g=g;
            po.b=b;
            cameraFrameCloud->push_back(po);
            }
    }

    //filter point cloud, decrease number of point cloud 
    pcl::PointCloud<PointType> cameraFrameCloudDS;
    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setInputCloud(cameraFrameCloud);
    downSizeFilter.setLeafSize(pointCloudLeafSize, pointCloudLeafSize, pointCloudLeafSize);
    downSizeFilter.filter(cameraFrameCloudDS);
    *cameraFrameCloud=cameraFrameCloudDS;

    // broad point cloud of current frame
    int cameraPointSize=cameraFrameCloud->points.size();

    if(transfer_surround){
        Eigen::Affine3f transCurrent=pcl::getTransformation(tx,ty,tz,troll,tpitch,tyaw);
        *cameraFrameCloud = *transformPointCloud(cameraFrameCloud, transCurrent);
    }

    if(dynacut&&cameraPointSize>200){
        // 创建滤波器对象
        pcl::PassThrough<PointType> passthrough;
        //输入点云
        passthrough.setInputCloud(cameraFrameCloud);
        //设置对y轴进行操作
        passthrough.setFilterFieldName("y");
        //设置滤波范围
        passthrough.setFilterLimits(thre_low, thre_high);
        // true表示保留滤波范围外，false表示保留范围内
        passthrough.setFilterLimitsNegative(false);
        //执行滤波并存储
        passthrough.filter(*cloud_PassThrough);
        std::cout << "直通滤波后点云数据点数："
                    << cloud_PassThrough->points.size() << std::endl;
        cameraFrameCloud = cloud_PassThrough;
    }
    ros::Time end = ros::Time::now();
    //std::cout<<"point time: "<<end-start<<std::endl;
    //std::cout<<"point size is"<<cameraPointSize<<std::endl;
    sensor_msgs::PointCloud2 cameraCloudFrameMsg;
    pcl::toROSMsg(*cameraFrameCloud, cameraCloudFrameMsg);
    cameraCloudFrameMsg.header.stamp = time_stamp;
    cameraCloudFrameMsg.header.frame_id = "/camera0_link";
    pubCameraCloudFrame.publish(cameraCloudFrameMsg);
}

int main(int argc, char *argv[]){

    ros::init(argc, argv, "test_frameSyn");
    ros::NodeHandle nh1;

    // get parameter from config file 
    nh1.param<float>("pointCloudLeafSize", pointCloudLeafSize, 0.1);
    nh1.param<int>("imageRowIncrease", imageRowIncrease, 1);
    nh1.param<int>("imageColIncrease", imageColIncrease, 1);
    nh1.param<int>("imagecutrow", imagecutrow, 200);
    nh1.param<int>("imagecutrow1", imagecutrow1, 200);
    nh1.param<int>("imagecutcol", imagecutcol, 200);
    nh1.param<bool>("usewhite", usewhite, true);
    nh1.param<bool>("reficity", refacity, true);
    nh1.param<bool>("transfer_surround", transfer_surround, true);
    nh1.param<bool>("dynacut", dynacut, true);
    nh1.param<float>("thre_low", thre_low, 0);
    nh1.param<float>("thre_high", thre_high, 0);
    nh1.param<float>("tx", tx, 0);
    nh1.param<float>("ty", ty, 0);
    nh1.param<float>("tz", tz, 0);
    nh1.param<float>("troll", troll, 0);
    nh1.param<float>("tpitch", tpitch, 0);
    nh1.param<float>("tyaw", tyaw, 0);
    image_transport::ImageTransport it(nh1);
    image_transport::Subscriber sub = it.subscribe("/camera", 1,cameraImageHandler);

    pubCameraCloudFrame = nh1.advertise<sensor_msgs::PointCloud2>("/cameraCloudFrame", 100);
    
    ros::spin();
    // ros::Rate rate(100);
    //     while (ros::ok()) {
    //         ros::spinOnce();
    //         rate.sleep();

    //     }


        std::cout<<"hello slam"<<std::endl;
        return 0;
}