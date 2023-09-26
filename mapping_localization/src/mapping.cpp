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

double odomKeyFramDisThresh=0.1; //1
double odomKeyFramAngThresh=0.1; //1
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
int invalidColorThresh=60;

double icpMaxCorrespondenceDistance=20;
int icpMaximumIterations=100;
double icpTransformationEpsilon=1e-10;
double icpEuclideanFitnessEpsilon=0.001;
double icpFitnessScoreThresh=0.3;

double ndtTransformationEpsilon=1e-9;
double ndtResolution=0.1;
double ndtFitnessScoreThresh=0.3;

std::string mapSaveLocation;
std::string data_folder;
//char * posedir="/home/zjl/avpslam_ws/lego-loam-world.txt";
float pointCloudLeafSize=0.1;
bool useICP=false;
bool useNDT=false; 
bool mapSave=true;
bool uselidar=true;
bool useplane = true;
bool useglobalfilter = true;
bool time_aligen = false;
bool myuseNDT_ICP=true;
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
#define MAX_LINE 8*13153 //8*13153 3570
//#define MAX_LINE 8*181 //8*13153
//#define MAX_LINE 8*1169

long double buf[MAX_LINE];
void readposes(){
    int i; 
    std::string trajpath = data_folder+"TUM_odmetry_interpolation.txt";
    FILE *fp=fopen(trajpath.c_str(),"r");
    if(fp==NULL) {
        printf(" Open file Error !\n");
        exit(0);
    }

    for(i=0;i<MAX_LINE;i++){
        fscanf(fp,"%Lf",&buf[i]);
    }
}


int time_count = -1;
int countframe = -1;
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

     // only number of point of current frame is sufficient ,compute pose 
    // if(currentFeatureCloud->points.size()<10){
    //     std::cout<<"no enough points"<<std::endl;
    //     return ;
    // }
        

    if(!systemInitial){ 
        //  global map initialize
         *currentFeatureCloudInWorld =*transformPointCloud(currentFeatureCloud, transWorldCurrent);
         *globalFeatureCloud=*globalFeatureCloud+*currentFeatureCloudInWorld;
        //*globalFeatureCloud=*globalFeatureCloud+*currentFeatureCloud;
        lastPoseX=0;
        lastPoseY=0;
        lastPoseZ=0;
        lastPoseRoll=0;
        systemInitial=true;
        return ;
    }
    
    // compute robot's pose with method of ndt
    if(useNDT){
       pcl::NormalDistributionsTransform<PointType, PointType> ndt_new;
       ndt_new.setTransformationEpsilon(ndtTransformationEpsilon);
       ndt_new.setResolution(ndtResolution);
       ndt_new.setMaximumIterations(100);
       ndt_new.setStepSize(0.1); 
       ndt_new.setInputSource(currentFeatureCloud);
       ndt_new.setInputTarget(globalFeatureCloud);
       pcl::PointCloud<PointType>::Ptr transCurrentCloudInWorld(new pcl::PointCloud<PointType>());
       ndt_new.align(*transCurrentCloudInWorld,transWorldCurrent.matrix());
       if (ndt_new.hasConverged() == false || ndt_new.getFitnessScore() > ndtFitnessScoreThresh) {
                std::cout << "ndt locolization failed    the score is   " << ndt_new.getFitnessScore() << std::endl;
                return ;
        } 
        else {
                transWorldCurrent =ndt_new.getFinalTransformation();
                std::cout << "NDT OK！" << ndt_new.getFitnessScore() << std::endl;
        }
               
    }

     
    time_count++;
    //aligen time
    const double eps=1e-6;
    while(!time_aligen){
        double time_s = time_stamp.toSec();
        for(int i=0;i<MAX_LINE;i++){
            if((time_s - buf[8*i])<eps) {
                cout<<time_s<<" "<<buf[8*i]<<endl;
                time_count = i;
                time_aligen = true;
                break;
            }
        }
    }
    
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
        if(useplane){
            currentZ = 0;
        }
        transWorldCurrent = pcl::getTransformation(currentX,currentY,currentZ,currentRoll,currentPitch,currentYaw);
        //std::cout<<"ok"<<std::endl;
    }

    //   compute robot's pose with method of icp   
    if(useICP){
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(icpMaxCorrespondenceDistance); 
        icp.setMaximumIterations(icpMaximumIterations);
        icp.setTransformationEpsilon(icpTransformationEpsilon);
        icp.setEuclideanFitnessEpsilon(icpEuclideanFitnessEpsilon);

        icp.setInputSource(currentFeatureCloud);
        icp.setInputTarget(globalFeatureCloud);
        pcl::PointCloud<PointType>::Ptr transCurrentCloudInWorld(new pcl::PointCloud<PointType>());
        icp.align(*transCurrentCloudInWorld,transWorldCurrent.matrix());
            

        if (icp.hasConverged() == false) {
                std::cout << "ICP locolization failed    the score is   " << icp.getFitnessScore() << std::endl;
                return ;
        } 
        else {
             transWorldCurrent = icp.getFinalTransformation();
             std::cout << "ICP OK！" << icp.getFitnessScore() << std::endl;
        }
           
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
    std::string pose_out = data_folder+"pose_truth.txt";
    std::ofstream out(pose_out.c_str(),std::ios::app);
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
    double dis=(currentX-lastPoseX)*(currentX-lastPoseX)+(currentY-lastPoseY)*(currentY-lastPoseY)+(currentZ-lastPoseZ)*(currentZ-lastPoseZ);
    double angle=abs(currentRoll-lastPoseRoll)+abs(currentPitch-lastPosePitch)+abs(currentYaw-lastPoseYaw);
    frameCount++;
    if(dis>odomKeyFramDisThresh){
        *globalFeatureCloud = *globalFeatureCloud+*currentFeatureCloudInWorld;
        if(useglobalfilter){
        pcl::PointCloud<PointType> globalMapDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(globalFeatureCloud);
        downSizeFilter.setLeafSize(pointCloudLeafSize, pointCloudLeafSize, pointCloudLeafSize);
        downSizeFilter.filter(globalMapDS);
        *globalFeatureCloud=globalMapDS;
        }
        frameCount=0;
        lastPoseX=currentX;
        lastPoseY=currentY;
        lastPoseZ=currentZ;
        lastPoseRoll=currentRoll;
        lastPosePitch=currentPitch;
        lastPoseYaw=currentYaw;


    }
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
    nh.param<std::string> ("/data_folder", data_folder, "/home/zjl/Desktop/parking/avp_data/");
    nh.param<double>("odomKeyFramDisThresh", odomKeyFramDisThresh, 1.0);
    nh.param<double>("odomKeyFramAngThresh", odomKeyFramAngThresh, 1.0);
    nh.param<int>("invalidColorThresh", invalidColorThresh, 60);
    nh.param<bool>("useICP", useICP, false);
    nh.param<double>("icpMaxCorrespondenceDistance", icpMaxCorrespondenceDistance, 20);
    nh.param<int>("icpMaximumIterations", icpMaximumIterations, 100);
    nh.param<double>("icpTransformationEpsilon", icpTransformationEpsilon, 1e-10);
    nh.param<double>("icpEuclideanFitnessEpsilon", icpEuclideanFitnessEpsilon, 0.001);
    nh.param<double>("icpFitnessScoreThresh", icpFitnessScoreThresh, 0.3);
    nh.param<float>("pointCloudLeafSize", pointCloudLeafSize, 0.1);
    nh.param<bool>("useNDT", useNDT, false);
    nh.param<bool>("uselidar", uselidar, true);
    nh.param<bool>("useplane", useplane, true);
    nh.param<bool>("myuseNDT_ICP", myuseNDT_ICP, false);
    nh.param<bool>("useglobalfilter", useglobalfilter, true);
    nh.param<double>("ndtTransformationEpsilon", ndtTransformationEpsilon, 1e-10);
    nh.param<double>("ndtResolution", ndtResolution, 0.1);
    nh.param<double>("ndtFitnessScoreThresh", ndtFitnessScoreThresh, 0.001);
    nh.param<bool>("mapSave", mapSave, true);
    
    readposes();

    ros::Subscriber subcameraCloud = nh.subscribe<sensor_msgs::PointCloud2>("/cameraCloudFrame", 100, cameraCloudHandler);
    

    pubCurrentFeature = nh.advertise<sensor_msgs::PointCloud2>("/currentFeature", 100);
    pubCurrentFeatureInWorld= nh.advertise<sensor_msgs::PointCloud2>("/currentFeatureInWorld", 100);
    pubGlobalFeature=nh.advertise<sensor_msgs::PointCloud2>("/globalFeatureMap", 100);
    pubCurrentPose = nh.advertise<nav_msgs::Odometry>("/currentPose", 100);


    // Eigen::Quaterniond q;
    // q.x() = 0.000342631;   
    // q.y() = -0.0136614;
    // q.z() =-0.588772;
    // q.w() =0.808184;
    // Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);  
    //transWorldCurrent=pcl::getTransformation(142.947,52.4267,0,euler[2],euler[1],euler[0]);
    //transWorldCurrent=pcl::getTransformation(-0.2569611357, 6.2492737338, 0,0,0,0);

    //ros::spin();
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



























