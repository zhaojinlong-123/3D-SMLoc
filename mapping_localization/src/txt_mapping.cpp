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
#include <mutex>
#include <queue>
#include <cmath>
#include <gazebo_msgs/ModelStates.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//   receive point cloud ,then extract feature; 
//   match current feature point cloud to  global map;
//   compute robot's pose with method of ndt or icp; 
//   create new keyFrame, add pointCloud to global map

using namespace std;
typedef pcl::PointXYZRGB PointType;

ros::Publisher pubCurrentFeature;
ros::Publisher pubCurrentFeatureInWorld;

ros::Publisher pubGlobalFeature;
ros::Publisher pubCurrentPose;

pcl::PointCloud<PointType>::Ptr currentFeatureCloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr currentFeatureCloudInWorld(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr globalFeatureCloud(new pcl::PointCloud<PointType>());

bool systemInitial=true;

double odomKeyFramDisThresh=1.0;

double lastPoseX=0;
double lastPoseY=0;
double lastPoseZ=0;
double lastPoseRoll=0;
double lastPosePitch=0;
double lastPoseYaw=0;

float currentX=0;
float currentY=0;
float currentZ=0;
float currentRoll=0;
float currentPitch=0;
float currentYaw=0;

float position_x=0;
float position_y=0;
float position_z=0;
float orientation_x=0;
float orientation_y=0;
float orientation_z=0;
float orientation_w=0;

int frameCount=0;


std::string mapSaveLocation="/home/zjl/avpslam_ws/src/AVP-SLAM-PLUS/avp_slam_plus/data/laserCloudMap.pcd";
//char * posedir="/home/zjl/avpslam_ws/lego-loam-world.txt";
float pointCloudLeafSize=0.1;
bool mapSave=true;
bool uselidar=true;
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
            cloudOut->points[i].r = pointFrom.r;
            cloudOut->points[i].g = pointFrom.g;
            cloudOut->points[i].b = pointFrom.b;

        }
        return cloudOut;
    }

#define MAX_LINE 8*13153 //8*13153
//#define MAX_LINE 8*1169

long double buf[MAX_LINE];
void readposes(){
    int i; 
    //FILE *fp=fopen("/home/zjl/avpslam_ws/vins_no_loop_world_interpolation.txt","r");
    FILE *fp=fopen("/home/zjl/avpslam_ws/lego-loam-world.txt","r");
    //FILE *fp=fopen("/home/zjl/avpslam_ws/11-18/lego-loam-world-5.txt","r");
    if(fp==NULL) {
        printf(" Open file Error !\n");
        exit(0);
    }

    for(i=0;i<MAX_LINE;i++){
    //for(i=0;i<8*1169;i++){
        fscanf(fp,"%Lf",&buf[i]);
    }
}


int time_count = -1;
//   feature extration
//   feature registration
//   compute robot's pose
//   create new keyFrame, and add pointCloud to global map

void cameraCloudHandler(const sensor_msgs::PointCloud2ConstPtr &cameraCloudMsg)
{
    currentFeatureCloud->clear();
    pcl::fromROSMsg(*cameraCloudMsg, *currentFeatureCloud);
    ros::Time time_stamp = cameraCloudMsg->header.stamp;
   
    //  broadcast  feature point cloud of current frame
    sensor_msgs::PointCloud2 cameraCloudFrameMsg;
    pcl::toROSMsg(*currentFeatureCloud, cameraCloudFrameMsg);
    cameraCloudFrameMsg.header.stamp = time_stamp;
    cameraCloudFrameMsg.header.frame_id = "/camera0_link";
    pubCurrentFeature.publish(cameraCloudFrameMsg);

    if(!systemInitial){
        
        //  global map initialize
        *globalFeatureCloud=*globalFeatureCloud+*currentFeatureCloud;

        lastPoseX=0;
        lastPoseY=0;
        lastPoseZ=0;
        lastPoseRoll=0;
        systemInitial=true;
        return ;
    }
     
    time_count++;
    if(uselidar){
        currentX = buf[8*time_count+1];
        currentY = buf[8*time_count+2];
        currentZ = buf[8*time_count+3];
        orientation_x = buf[8*time_count+4];
        orientation_y = buf[8*time_count+5];
        orientation_z = buf[8*time_count+6];
        orientation_w = buf[8*time_count+7];
        Eigen::Quaterniond q;
        q.x() = orientation_x;
        q.y() = orientation_y;
        q.z() = orientation_z;
        q.w() = orientation_w;
        Eigen::Vector3d eulerAngle4 = q.toRotationMatrix().eulerAngles(2,1,0);
        currentRoll = eulerAngle4[2];
        currentPitch = eulerAngle4[1];
        currentYaw = eulerAngle4[0];
        transWorldCurrent = pcl::getTransformation(currentX,currentY,currentZ,currentRoll,currentPitch,currentYaw);
        //std::cout<<"ok"<<std::endl;
    }

    pcl::getTranslationAndEulerAngles(transWorldCurrent,currentX,currentY,currentZ,currentRoll,currentPitch,currentYaw);

    


    tf::TransformBroadcaster tfMap2Camera;
    tf::Transform mapToCamera = tf::Transform(tf::createQuaternionFromRPY(currentRoll,currentPitch,currentYaw), tf::Vector3(currentX,currentY,currentZ));
    tfMap2Camera.sendTransform(tf::StampedTransform(mapToCamera, time_stamp, "/world", "/camera0_link"));


    // broacast pose of robot
    geometry_msgs::Quaternion cameraPoseQuat=tf::createQuaternionMsgFromRollPitchYaw(currentRoll,currentPitch,currentYaw);
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "/world";
    odomAftMapped.child_frame_id = "/camera0_link";
    odomAftMapped.header.stamp = time_stamp;
    odomAftMapped.pose.pose.orientation.x = cameraPoseQuat.x;//orientation_x;
    odomAftMapped.pose.pose.orientation.y = cameraPoseQuat.y;//orientation_y;
    odomAftMapped.pose.pose.orientation.z = cameraPoseQuat.z;//orientation_z;
    odomAftMapped.pose.pose.orientation.w = cameraPoseQuat.w;//orientation_w;
    odomAftMapped.pose.pose.position.x = currentX;
    odomAftMapped.pose.pose.position.y = currentY;
    odomAftMapped.pose.pose.position.z = currentZ;
    std::ofstream out("/home/zjl/avpslam_ws/estimation.txt",std::ios::app);
    out<<time_stamp<<" "<<currentX<<" "<<currentY<<" "<<currentZ<<" "<<cameraPoseQuat.x<<" "<<cameraPoseQuat.y<<" "<<cameraPoseQuat.z<<" "<<cameraPoseQuat.w<<std::endl; 
    out.close();
    pubCurrentPose.publish(odomAftMapped);

    //   transform  point cloud from current  coordinate to world coordinate
    //std::cout<<"now robot is in x "<<transWorldCurrent(0,3)<< " y "<< transWorldCurrent(1,3)<< "  z  "<<transWorldCurrent(2,3)<<std::endl;
    *currentFeatureCloudInWorld =*transformPointCloud(currentFeatureCloud, transWorldCurrent);
    sensor_msgs::PointCloud2 cameraCloudCurrentInWorldMsg;
    pcl::toROSMsg(*currentFeatureCloudInWorld, cameraCloudCurrentInWorldMsg);
    cameraCloudCurrentInWorldMsg.header.stamp = time_stamp;
    cameraCloudCurrentInWorldMsg.header.frame_id = "/world";
    pubCurrentFeatureInWorld.publish(cameraCloudCurrentInWorldMsg);

    //  create new keyFrame, and add pointCloud to global map
    *globalFeatureCloud = *globalFeatureCloud+*currentFeatureCloudInWorld;
    int cameraPointSize=globalFeatureCloud->points.size();
    std::cout<<"point size is "<<cameraPointSize<<std::endl;
        // pcl::PointCloud<PointType> globalMapDS;
        // pcl::VoxelGrid<PointType> downSizeFilter;
        // downSizeFilter.setInputCloud(globalFeatureCloud);
        // downSizeFilter.setLeafSize(pointCloudLeafSize, pointCloudLeafSize, pointCloudLeafSize);
        // downSizeFilter.filter(globalMapDS);
        // *globalFeatureCloud=globalMapDS;
    lastPoseX=currentX;
    lastPoseY=currentY;
    lastPoseZ=currentZ;
    lastPoseRoll=currentRoll;
    lastPosePitch=currentPitch;
    lastPoseYaw=currentYaw;

    //  broadcast prior  global map information 
    sensor_msgs::PointCloud2 cameraCloudGlobalMapMsg;
    pcl::toROSMsg(*globalFeatureCloud, cameraCloudGlobalMapMsg);
    cameraCloudGlobalMapMsg.header.stamp = time_stamp;
    cameraCloudGlobalMapMsg.header.frame_id = "/world";
    pubGlobalFeature.publish(cameraCloudGlobalMapMsg);


}


int main(int argc, char *argv[]){

    ros::init(argc, argv, "test_odom");
    ros::NodeHandle nh;
    
    
    // get parameter from config file 
    nh.param<std::string>("mapSaveLocation", mapSaveLocation, "/home/zjl/avpslam_ws/src/AVP-SLAM-PLUS/avp_slam_plus/data/");
    nh.param<double>("odomKeyFramDisThresh", odomKeyFramDisThresh, 1.0);
    nh.param<float>("pointCloudLeafSize", pointCloudLeafSize, 0.1);
    nh.param<bool>("uselidar", uselidar, true);
    nh.param<bool>("mapSave", mapSave, true);
    readposes();

    ros::Subscriber subcameraCloud = nh.subscribe<sensor_msgs::PointCloud2>("/cameraCloudFrame", 100, cameraCloudHandler);

    pubCurrentFeature = nh.advertise<sensor_msgs::PointCloud2>("/currentFeature", 100);
    pubCurrentFeatureInWorld= nh.advertise<sensor_msgs::PointCloud2>("/currentFeatureInWorld", 100);
    pubGlobalFeature=nh.advertise<sensor_msgs::PointCloud2>("/globalFeatureMap", 100);
    pubCurrentPose = nh.advertise<nav_msgs::Odometry>("/currentPose", 100);

    //transWorldCurrent=pcl::getTransformation(0, 0, 0,0,0,0);

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    //  save map for relocation
    if(mapSave){
       std::cout << "map save start" << std::endl;
       pcl::PointCloud<PointType>  globalMap=*globalFeatureCloud;
       pcl::io::savePCDFileASCII(mapSaveLocation, *globalFeatureCloud);        
	   std::cout << "map save over" << std::endl;
    }

    std::cout<<"hello slam"<<std::endl;
    return 0;
}
