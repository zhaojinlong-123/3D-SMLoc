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



//   receive point cloud ,then extract feature; 
//   match current feature point cloud to prior global map;
//   compute robot's pose with method of ndt or icp; 

typedef pcl::PointXYZRGB PointType;

ros::Publisher pubCurrentFeature;
ros::Publisher pubCurrentFeatureInWorld;
ros::Publisher pubGlobalFeature;
ros::Publisher pubCurrentPose;



pcl::PointCloud<PointType>::Ptr currentFeatureCloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr currentFeatureCloudInWorld(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr globalFeatureCloud(new pcl::PointCloud<PointType>());

bool systemInitial=false;


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

float icurrentX=0;
float icurrentY=0;
float icurrentZ=0;
float iorientation_x=0;
float iorientation_y=0;
float iorientation_z=0;
float iorientation_w=0;
float icurrentRoll=0;
float icurrentPitch=0;
float icurrentYaw=0;

int invalidColorThresh=60;

double icpMaxCorrespondenceDistance=20;
int icpMaximumIterations=100;
double icpTransformationEpsilon=1e-10;
double icpEuclideanFitnessEpsilon=0.001;
double icpFitnessScoreThresh=0.3;

double ndtTransformationEpsilon=1e-10;
double ndtResolution=0.1;
double ndtFitnessScoreThresh=0.3;
float pointCloudLeafSize=0.1;
int frameCount=0;

std::string mapSaveLocation="/home/zjl/avpslam_ws/src/AVP-SLAM-PLUS/avp_slam_plus/data-lego-loam/laserCloudMap.pcd";
bool globalMapLoad=false;
bool initPose=false;
bool useICP=true;
bool useNDT=false; 
bool usemyICP=true;
Eigen::Affine3f transWorldCurrent;
Eigen::Affine3f itransWorldCurrent;
#define MAX_LINE 8*13153 //8*13153

long double buf[MAX_LINE];
void readposes(){
    int i; 
    FILE *fp=fopen("/home/zjl/avpslam_ws/localization_test/2022-11-30/fast-lio-k0.1-plane/estimation_interpolation.txt","r");
    if(fp==NULL) {
        printf(" Open file Error !\n");
        exit(0);
    }

    for(i=0;i<MAX_LINE;i++){
        fscanf(fp,"%Lf",&buf[i]);
    }
}

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

int time_count = -1;
//   feature extration
//   feature registration
//   compute robot's pose
void cameraCloudHandler(const sensor_msgs::PointCloud2ConstPtr &cameraCloudMsg)
{
    pcl::PointCloud<pcl::PointXYZRGB> cameraCloudIn;
    pcl::fromROSMsg(*cameraCloudMsg, cameraCloudIn);
    
    ros::Time time_stamp = cameraCloudMsg->header.stamp;
    time_count++;
    icurrentX = buf[8*time_count+1];
    icurrentY = buf[8*time_count+2];
    icurrentZ = buf[8*time_count+3];
    iorientation_x = buf[8*time_count+4];
    iorientation_y = buf[8*time_count+5];
    iorientation_z = buf[8*time_count+6];
    iorientation_w = buf[8*time_count+7];
    std::cout<<"current pose: "<<icurrentX<<icurrentY<<icurrentZ<<std::endl;
    Eigen::Quaterniond q;
    q.x() = iorientation_x;
    q.y() = iorientation_y;
    q.z() = iorientation_z;
    q.w() = iorientation_w;
    Eigen::Vector3d eulerAngle4 = q.toRotationMatrix().eulerAngles(2,1,0);
    icurrentRoll = eulerAngle4[2];
    icurrentPitch = eulerAngle4[1];
    icurrentYaw = eulerAngle4[0];    
    itransWorldCurrent = pcl::getTransformation(icurrentX,icurrentY,icurrentZ,icurrentRoll,icurrentPitch,icurrentYaw);
    //   extract feature ;
    //   there,for simplicity,just according to color;
    //   for real scene,should adapt deep learning method,detecting valid feature point  
    currentFeatureCloud->clear();
    for(size_t i=0;i<cameraCloudIn.points.size();i++){
         PointType pi=cameraCloudIn.points[i];
         int r=int(pi.r);
         int g=int(pi.g);
         int b=int(pi.b);
        if(r<invalidColorThresh && b<invalidColorThresh && g<invalidColorThresh){
            continue;
        }
        currentFeatureCloud->push_back(pi);
    }
    
    //  broadcast  feature point cloud of current frame
    sensor_msgs::PointCloud2 cameraCloudFrameMsg;
    pcl::toROSMsg(*currentFeatureCloud, cameraCloudFrameMsg);
    cameraCloudFrameMsg.header.stamp = time_stamp;
    cameraCloudFrameMsg.header.frame_id = "/camera0_link";
    pubCurrentFeature.publish(cameraCloudFrameMsg);

    //  broadcast prior  global map information 
    if(globalMapLoad){
        sensor_msgs::PointCloud2 cameraCloudGlobalMapMsg;
        pcl::toROSMsg(*globalFeatureCloud, cameraCloudGlobalMapMsg);
        cameraCloudGlobalMapMsg.header.stamp = ros::Time::now();
        cameraCloudGlobalMapMsg.header.frame_id = "/world";
        pubGlobalFeature.publish(cameraCloudGlobalMapMsg);
    }

    if(!systemInitial){
        return ;
    }

    // compute robot's pose with method of ndt
    if(useNDT){
       pcl::NormalDistributionsTransform<PointType, PointType> ndt;
       ndt.setTransformationEpsilon(ndtTransformationEpsilon);
       ndt.setResolution(ndtResolution);
       ndt.setInputSource(currentFeatureCloud);
       ndt.setInputTarget(globalFeatureCloud);
       pcl::PointCloud<PointType>::Ptr transCurrentCloudInWorld(new pcl::PointCloud<PointType>());
       ndt.align(*transCurrentCloudInWorld, itransWorldCurrent.matrix());
       
       if (ndt.hasConverged() == false || ndt.getFitnessScore() > ndtFitnessScoreThresh) {
               std::cout << "ndt locolization failed    the score is   " << ndt.getFitnessScore() << std::endl;
               transWorldCurrent = itransWorldCurrent;
        } 
        else 
                transWorldCurrent =ndt.getFinalTransformation();
    }  

    //   compute robot's pose with method of icp   
    if(useICP){
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(icpMaxCorrespondenceDistance); 
        icp.setMaximumIterations(icpMaximumIterations);
        icp.setTransformationEpsilon(icpTransformationEpsilon);
        icp.setEuclideanFitnessEpsilon(icpEuclideanFitnessEpsilon);

        icp.setInputSource(currentFeatureCloud);
        icp.setInputTarget(globalFeatureCloud);
        pcl::PointCloud<PointType>::Ptr transCurrentCloudInWorld(new pcl::PointCloud<PointType>());
        icp.align(*transCurrentCloudInWorld,itransWorldCurrent.matrix());

        if (icp.hasConverged() == false || icp.getFitnessScore() > icpFitnessScoreThresh) {
            std::cout << "ICP locolization failed    the score is   " << icp.getFitnessScore() << std::endl;
            transWorldCurrent = itransWorldCurrent;
        } 
        else 
            transWorldCurrent = icp.getFinalTransformation();
    }

    if(usemyICP){
        if(currentFeatureCloud->points.size()>3)
        {
        pcl::KdTreeFLANN<PointType> kd_tree;
        kd_tree.setInputCloud(globalFeatureCloud);

        float max_correspond_distance_ = 0.2;
        pcl::PointCloud<PointType>::Ptr cloud_S_Trans(new pcl::PointCloud<PointType>);
        pcl::transformPointCloud(*currentFeatureCloud, *cloud_S_Trans, itransWorldCurrent);
        std::vector<int> match_indexs;
        for (auto i = 0; i < cloud_S_Trans->size(); ++i)
        {
            std::vector<float> k_distances(2);
            std::vector<int> k_indexs(2);
            kd_tree.nearestKSearch(cloud_S_Trans->at(i), 1, k_indexs, k_distances);
            if (k_distances[0] < max_correspond_distance_)
            {
                match_indexs.emplace_back(k_indexs[0]);
            }
        }

        pcl::PointCloud<PointType>::Ptr cloud_match_from_S_trans(new pcl::PointCloud<PointType>);
        pcl::copyPointCloud(*cloud_S_Trans, match_indexs, *cloud_match_from_S_trans);
        pcl::PointCloud<PointType>::Ptr cloud_match_from_T(new pcl::PointCloud<PointType>);
        pcl::copyPointCloud(*globalFeatureCloud, match_indexs, *cloud_match_from_T);

        // 3d center of two clouds
        Eigen::Vector4f mu_S_temp, mu_T_temp;
        pcl::compute3DCentroid(*cloud_match_from_S_trans, mu_S_temp);
        pcl::compute3DCentroid(*cloud_match_from_T, mu_T_temp);
        Eigen::Vector3f mu_S(mu_S_temp[0], mu_S_temp[1], mu_S_temp[2]);
        Eigen::Vector3f mu_T(mu_T_temp[0], mu_T_temp[1], mu_T_temp[2]);

        // H += (S_i) * (T_i^T)
        Eigen::Matrix3f H_icp = Eigen::Matrix3f::Identity();
        for (auto i = 0; i < match_indexs.size(); ++i)
        {
            Eigen::Vector3f s_ci(cloud_match_from_S_trans->at(i).x - mu_S[0],
                                cloud_match_from_S_trans->at(i).y - mu_S[1],
                                cloud_match_from_S_trans->at(i).z - mu_S[2]);
            Eigen::Vector3f t_ci(cloud_match_from_T->at(i).x - mu_T[0],
                                cloud_match_from_T->at(i).y - mu_T[1],
                                cloud_match_from_T->at(i).z - mu_T[2]);
            Eigen::Matrix3f H_temp = s_ci * t_ci.transpose();
            H_icp += H_temp;
        }

        Eigen::JacobiSVD<Eigen::MatrixXf> svd(H_icp, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix3f R_ts = svd.matrixU() * (svd.matrixV().transpose());
        Eigen::Vector3f t_ts = mu_T - R_ts * mu_S;// T = R_ts * S + t_ts
        Eigen::Quaternionf q_ts(R_ts);
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform = itransWorldCurrent.matrix();
        transform.block<3, 3>(0,0) *= q_ts.normalized().toRotationMatrix().inverse();
        transform.block<3, 1>(0, 3) += t_ts;
        transWorldCurrent = transform;
        std::cout<<"icp success"<<std::endl;
        }
        else
        {
        std::cout<<"no icp"<<std::endl;
        transWorldCurrent = itransWorldCurrent;
        }
    }


    pcl::getTranslationAndEulerAngles(transWorldCurrent,currentX,currentY,currentZ,currentRoll,currentPitch,currentYaw);

    if(currentPitch>10)
      currentPitch=0;
    transWorldCurrent=pcl::getTransformation(currentX,currentY,currentZ,currentRoll,currentPitch,currentYaw);

    
    tf::TransformBroadcaster tfMap2Camera;
    tf::Transform mapToCamera = tf::Transform(tf::createQuaternionFromRPY(currentRoll,currentPitch,currentYaw), tf::Vector3(currentX,currentY,currentZ));
    //tfMap2Camera.sendTransform(tf::StampedTransform(mapToCamera, ros::Time::now(), "/world", "/camera0_link"));
    tfMap2Camera.sendTransform(tf::StampedTransform(mapToCamera, time_stamp, "/world", "/camera0_link"));
    // broacast pose of robot
    geometry_msgs::Quaternion cameraPoseQuat=tf::createQuaternionMsgFromRollPitchYaw(currentRoll,currentPitch,currentYaw);
    nav_msgs::Odometry odomAftMapped;
	odomAftMapped.header.frame_id = "/world";
	odomAftMapped.child_frame_id = "/camera0_link";
	//odomAftMapped.header.stamp = ros::Time::now();
    odomAftMapped.header.stamp = time_stamp;
	odomAftMapped.pose.pose.orientation.x = cameraPoseQuat.x;
	odomAftMapped.pose.pose.orientation.y = cameraPoseQuat.y;
	odomAftMapped.pose.pose.orientation.z = cameraPoseQuat.z;
	odomAftMapped.pose.pose.orientation.w = cameraPoseQuat.w;
	odomAftMapped.pose.pose.position.x = currentX;
	odomAftMapped.pose.pose.position.y = currentY;
	odomAftMapped.pose.pose.position.z = currentZ;
    std::ofstream out("/home/zjl/avpslam_ws/localization_result.txt",std::ios::app);
    out<<time_stamp<<" "<<currentX<<" "<<currentY<<" "<<currentZ<<" "<<cameraPoseQuat.x<<" "<<cameraPoseQuat.y<<" "<<cameraPoseQuat.z<<" "<<cameraPoseQuat.w<<std::endl; 
    out.close();    
	pubCurrentPose.publish(odomAftMapped);

    std::cout<<"now robot is in x "<<currentX<< " y "<< currentY<< "  z  "<<currentZ<<std::endl;
    std::cout<<" now robot is in  roll "<<currentRoll<<"  pitch "<<currentPitch<<"  yaw  "<<currentYaw<<std::endl;
   
   
    //   transform  point cloud from current  coordinate to world coordinate
    *currentFeatureCloudInWorld =*transformPointCloud(currentFeatureCloud, transWorldCurrent);
    sensor_msgs::PointCloud2 cameraCloudCurrentInWorldMsg;
    pcl::toROSMsg(*currentFeatureCloudInWorld, cameraCloudCurrentInWorldMsg);
    //cameraCloudCurrentInWorldMsg.header.stamp = ros::Time::now();
    cameraCloudCurrentInWorldMsg.header.stamp = time_stamp;
    cameraCloudCurrentInWorldMsg.header.frame_id = "/world";
    pubCurrentFeatureInWorld.publish(cameraCloudCurrentInWorldMsg);

    

}

//   system initial
void systemInit(){
   
    //   for simplicity,at first time, arrange the robot at origin area;
    //   if want to be relocated everywhere, should introduced other method. machine learing method is a good choice  
    transWorldCurrent=pcl::getTransformation(-0.255103,6.18726,-0.154268,0,0,0);
    
      
    systemInitial= true;
}



// load prior global map 
void loadMap(){
        globalFeatureCloud->clear();
        std::cout<<"load map begin ******************"<<std::endl;
        pcl::io::loadPCDFile(mapSaveLocation, *globalFeatureCloud);
        std::cout<<"load map over ******************"<<std::endl;
        pcl::PointCloud<PointType> globalMapDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(globalFeatureCloud);
        downSizeFilter.setLeafSize(pointCloudLeafSize, pointCloudLeafSize, pointCloudLeafSize);
        downSizeFilter.filter(globalMapDS);
        *globalFeatureCloud=globalMapDS;
        globalMapLoad=true;
     }


// get initial pose from outer program ;For example ,judging position of robot from gps or mannual marker
void initposeHandler(const geometry_msgs::PoseWithCovarianceStampedConstPtr &initpose){
         
         double tempw = initpose->pose.pose.orientation.w;
         double tempx = initpose->pose.pose.orientation.x;
         double tempy= initpose->pose.pose.orientation.y;
         double tempz = initpose->pose.pose.orientation.z;
         
         double x = initpose->pose.pose.position.x;
         double y = initpose->pose.pose.position.y;
         double z = initpose->pose.pose.position.z;

         double rol,pit,yaw;
         tf::Matrix3x3(tf::Quaternion(tempx, tempy, tempz, tempw)).getRPY(rol, pit,yaw);
  
        transWorldCurrent=pcl::getTransformation(x,y,z,rol,pit,yaw);

        pcl::getTranslationAndEulerAngles(transWorldCurrent,currentX,currentY,currentZ,currentRoll,currentPitch,currentYaw);


        std::cout << "********* set init pose  start**********" << std::endl;
        std::cout<<"now robot is in x "<<currentX<< " y "<< currentY<< "  z  "<<currentZ<<std::endl;
        std::cout<<" now robot is in  roll "<<currentRoll<<"  pitch "<<currentPitch<<"  yaw  "<<currentYaw<<std::endl;
        std::cout << "********* set init pose  end**********" << std::endl;

        systemInitial= true;
}

int main(int argc, char *argv[]){

    ros::init(argc, argv, "test_odom");
    ros::NodeHandle nh;
   
    // get parameter from config file 
    nh.param<std::string>("mapSaveLocation", mapSaveLocation, "/home/lgt/multicamera_ws/src/avp_slam/data/");
    nh.param<int>("invalidColorThresh", invalidColorThresh, 60);
    nh.param<bool>("useICP", useICP, true);
    nh.param<double>("icpMaxCorrespondenceDistance", icpMaxCorrespondenceDistance, 20);
    nh.param<int>("icpMaximumIterations", icpMaximumIterations, 100);
    nh.param<double>("icpTransformationEpsilon", icpTransformationEpsilon, 1e-10);
    nh.param<double>("icpEuclideanFitnessEpsilon", icpEuclideanFitnessEpsilon, 0.001);
    nh.param<double>("icpFitnessScoreThresh", icpFitnessScoreThresh, 0.3);
    nh.param<bool>("useNDT", useNDT, true);
    nh.param<bool>("usemyICP", usemyICP, true);
    nh.param<double>("ndtTransformationEpsilon", ndtTransformationEpsilon, 1e-10);
    nh.param<double>("ndtResolution", ndtResolution, 0.1);
    nh.param<double>("ndtFitnessScoreThresh", ndtFitnessScoreThresh, 0.001);
    nh.param<float>("pointCloudLeafSize", pointCloudLeafSize, 0.1);
    ros::Subscriber subcameraCloud = nh.subscribe<sensor_msgs::PointCloud2>("/cameraCloudFrame", 100, cameraCloudHandler);
    ros::Subscriber subInitpose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialposeGive", 1,  initposeHandler,  ros::TransportHints().tcpNoDelay());
    
    pubCurrentFeature = nh.advertise<sensor_msgs::PointCloud2>("/currentFeature", 100);
    pubCurrentFeatureInWorld= nh.advertise<sensor_msgs::PointCloud2>("/currentFeatureInWorld", 100);
    pubGlobalFeature=nh.advertise<sensor_msgs::PointCloud2>("/globalFeatureMap", 100);
    pubCurrentPose = nh.advertise<nav_msgs::Odometry>("/currentPose", 100);
    readposes();
    // load prior global map
    loadMap();
    
    // initialize pose and system
    systemInit();

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        rate.sleep();
    }

    std::cout<<"hello slam"<<std::endl;
    return 0;
}