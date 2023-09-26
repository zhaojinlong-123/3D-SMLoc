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
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/features/fpfh_omp.h>     // fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/ia_ransac.h>// sac_ia算法
#include <boost/thread/thread.hpp>

#include "matrix_utils.h"

#include <Eigen/Dense>
#include <Eigen/Geometry> 

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <algorithm>
//   receive point cloud ,then extract feature; 
//   match current feature point cloud to prior global map;
//   compute robot's pose with method of ndt or icp; 

typedef pcl::PointXYZRGB PointType;
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<float, 6, 1> Vector6d;
ros::Publisher pubCurrentFeature;
ros::Publisher pubCurrentFeatureInWorld;

ros::Publisher pubGlobalFeature;
ros::Publisher pubCurrentPose;

ros::Publisher pub_cube;

pcl::PointCloud<PointType>::Ptr currentFeatureCloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr currentFeatureCloudInWorld(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr globalFeatureCloud(new pcl::PointCloud<PointType>());

bool systemInitial=false;

double init_x=0;
double init_y=0;
double init_z=0;
double init_rx=0;
double init_ry=0;
double init_rz=0;
double init_rw=0;

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

float licurrentX=0;
float licurrentY=0;
float licurrentZ=0;
float licurrentRoll=0;
float licurrentPitch=0;
float licurrentYaw=0;

float cubecurrentX=0;
float cubecurrentY=0;
float cubecurrentZ=0;
float cubecurrentRoll=0;
float cubecurrentPitch=0;
float cubecurrentYaw=0;

int invalidColorThresh=60;

double icpMaxCorrespondenceDistance=20;
int icpMaximumIterations=100;
double icpTransformationEpsilon=1e-10;
double icpEuclideanFitnessEpsilon=0.001;
double icpFitnessScoreThresh=0.3;

double ndtTransformationEpsilon=1e-10;
double ndtResolution=0.1;
double ndtFitnessScoreThresh=0.3;
bool useplane=true;
int frameCount=0;

std::string mapSaveLocation="";
std::string data_folder="";
std::string localization_folder="";
bool globalMapLoad=false;
bool initPose=false;
bool useICP=true;
bool useNDT=false; 
bool useNDT_ICP=true;
bool useNDT_GICP=true;
bool useNDT_PICP=true;
bool use_sac_ndt=true;
bool cube_localization=true;
bool cube_ass=false;
Eigen::Affine3f transWorldCurrent;
Eigen::Affine3f litransWorldCurrent;
Eigen::Affine3f cubetransWorldCurrent;
int start_frame=0;
int end_frame=0;
#define MAX_LINE 8*13153 

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
int current_frame=0;
bool time_aligen = false;
bool uselidar = false;


visualization_msgs::MarkerArray cuboids_to_marker(std::vector<Vector9d> obj_landmark, Eigen::Vector3d rgbcolor) 
{
    visualization_msgs::MarkerArray plane_markers;
    visualization_msgs::Marker marker;
    marker.header.frame_id="/world";  marker.header.stamp=ros::Time::now();
    bool once = true;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;    
    
    //marker.lifetime = ros::Duration(0.2);
    marker.color.r = rgbcolor(0); marker.color.g = rgbcolor(1); marker.color.b = rgbcolor(2); marker.color.a = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 3;
    for(int i=0; i < obj_landmark.size(); i++)
    {
        marker.id = i;
        marker.pose.position.x = obj_landmark[i][0];
        marker.pose.position.y = obj_landmark[i][1];
        marker.pose.position.z = 1.5; //position[2];
        plane_markers.markers.push_back(marker);
    }
    return plane_markers;
}

Eigen::MatrixXd object_map(5,9);
std::vector<Vector9d> all_objects;
void readcubemap(){
    if(cube_localization){
        std::string object_path = localization_folder+"all_objects.txt";
        read_pose_truth_txt(object_path, object_map);
        if(object_map.rows()!=0){
            for (int i=0; i<object_map.rows();i++)
            {
                Vector9d cube_pose;
                cube_pose << object_map(i, 0), object_map(i, 1), object_map(i, 2), 0, 0, object_map(i, 5),
				object_map(i, 6), object_map(i, 7), object_map(i, 8); // xyz roll pitch yaw scale
                all_objects.push_back(cube_pose);
            }
        }
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

void cloud_with_normal(pcl::PointCloud<PointType>::Ptr& cloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals) 
{   //---------------------拼接点云数据与法线信息-------------------
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;//OMP加速
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	n.setNumberOfThreads(10);//设置openMP的线程数
	//n.setViewPoint(0,0,0);//设置视点，默认为（0，0，0

    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
    copyPointCloud(*cloud, *output);
	n.setInputCloud(output);
	n.setSearchMethod(tree);
	n.setKSearch(10);//点云法向计算时，需要所搜的近邻点大小
	//n.setRadiusSearch(0.03);//半径搜素
	n.compute(*normals);//开始进行法向计
	//将点云数据与法向信息拼接
	pcl::concatenateFields(*output, *normals, *cloud_normals);
}

//   feature extration
//   feature registration
//   compute robot's pose
void cameraCloudHandler(const sensor_msgs::PointCloud2ConstPtr &cameraCloudMsg)
{
    ros::Time start = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZRGB> cameraCloudIn;
    pcl::fromROSMsg(*cameraCloudMsg, cameraCloudIn);
    
    ros::Time time_stamp = cameraCloudMsg->header.stamp;
   
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
    //cameraCloudFrameMsg.header.stamp = ros::Time::now();
    cameraCloudFrameMsg.header.stamp = time_stamp;
    cameraCloudFrameMsg.header.frame_id = "/camera0_link";
    pubCurrentFeature.publish(cameraCloudFrameMsg);

    time_count++;
    //aligen time
    if(uselidar){
    const double eps=1e-6;
    while(!time_aligen){
        double time_s = time_stamp.toSec();
        for(int i=0;i<MAX_LINE;i++){
            if((time_s - buf[8*i])<eps) {
                time_count = i;
                time_aligen = true;
                break;
            }
        }
    }
    }

    if(uselidar){        
        licurrentX = buf[8*time_count+1];
        licurrentY = buf[8*time_count+2];
        licurrentZ = buf[8*time_count+3];
        Eigen::Quaterniond q;
        q.x() = buf[8*time_count+4];
        q.y() = buf[8*time_count+5];
        q.z() = buf[8*time_count+6];
        q.w() = buf[8*time_count+7];
        Eigen::Vector3d eulerAngle4 = q.toRotationMatrix().eulerAngles(2,1,0);
        licurrentRoll = eulerAngle4[2];
        licurrentPitch = eulerAngle4[1];
        licurrentYaw = eulerAngle4[0];
        //std::cout<<"current R: "<<licurrentRoll<<" "<<licurrentPitch<<" "<<licurrentYaw<<std::endl;
        if(useplane){
            licurrentZ = 0;
        }
        litransWorldCurrent = pcl::getTransformation(licurrentX,licurrentY,licurrentZ,licurrentRoll,licurrentPitch,licurrentYaw);
    }
    //std::cout<<"time count: "<<time_count<<std::endl;
    // only number of point of current frame is sufficient ,compute pose 
    if(currentFeatureCloud->points.size()<10)
       return;

    //  broadcast prior  global map information 
    if(globalMapLoad){
        sensor_msgs::PointCloud2 cameraCloudGlobalMapMsg;
        pcl::toROSMsg(*globalFeatureCloud, cameraCloudGlobalMapMsg);
        cameraCloudGlobalMapMsg.header.stamp = ros::Time::now();
        cameraCloudGlobalMapMsg.header.frame_id = "/world";
        pubGlobalFeature.publish(cameraCloudGlobalMapMsg);
    }

    if(cube_localization){
        pub_cube.publish(cuboids_to_marker(all_objects,Eigen::Vector3d(1,1,1)));
        if(cube_ass){
            current_frame=time_count+3;
            Eigen::MatrixXd pred_frame_objects;
            pred_frame_objects = all_offline_object_cubes[current_frame];
            //std::cout<<"curr frmae: "<<current_frame<<std::endl;
            if(pred_frame_objects.rows()==0) std::cout<<"no cube current frame"<<std::endl;
            else{
                Vector9d current_cube;
                current_cube << pred_frame_objects(0, 0), pred_frame_objects(0, 1), pred_frame_objects(0, 2), 0, 0, pred_frame_objects(0, 3),
				pred_frame_objects(0, 4), pred_frame_objects(0, 5), pred_frame_objects(0, 6); // xyz roll pitch yaw scale 
                Eigen::Affine3f tran_cube2cam = pcl::getTransformation(current_cube[0],current_cube[1],current_cube[2],current_cube[3],current_cube[4],current_cube[5]);
                Eigen::Affine3f tran_cube2world = transWorldCurrent*tran_cube2cam;
                Vector6d current_cube_world;
                pcl::getTranslationAndEulerAngles(tran_cube2world,current_cube_world[0],current_cube_world[1],current_cube_world[2],current_cube_world[3],current_cube_world[4],current_cube_world[5]);
                double diff=INT_MAX;
                Vector9d ass_cube;
                for(int k=0;k<all_objects.size();k++){
                    Vector9d tmp_global_cube = all_objects[k];
                    double squre = (current_cube_world[0]-tmp_global_cube[0])*(current_cube_world[0]-tmp_global_cube[0])+(current_cube_world[1]-tmp_global_cube[1])*(current_cube_world[1]-tmp_global_cube[1]);
                    if(squre<diff){
                        diff=squre;
                        ass_cube=tmp_global_cube;
                    }
                }
                if(diff>20){
                    cubetransWorldCurrent=litransWorldCurrent;
                    std::cout<<"no cube ass,diff is: "<<diff<<std::endl;
                }
                else{
                    //std::cout<<"current cube: "<<current_cube[0]<<" "<<current_cube[1]<<" "<<current_cube[2]<<std::endl;
                    //std::cout<<"current cube world: "<<current_cube_world[0]<<" "<<current_cube_world[1]<<" "<<current_cube_world[2]<<std::endl;
                    //std::cout<<"ass cube: "<<ass_cube[0]<<" "<<ass_cube[1]<<" "<<ass_cube[2]<<std::endl;
                    cubecurrentX = ass_cube[0]-current_cube[0];
                    cubecurrentY = ass_cube[1]-current_cube[1];
                    cubecurrentZ = 0;
                    cubecurrentRoll = currentRoll;
                    cubecurrentPitch = currentPitch;
                    cubecurrentYaw = currentYaw;
                    cubetransWorldCurrent = pcl::getTransformation(cubecurrentX,cubecurrentY,cubecurrentZ,cubecurrentRoll,cubecurrentPitch,cubecurrentYaw);
                }               
            }
        }
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
       ndt.align(*transCurrentCloudInWorld, transWorldCurrent.matrix());
       
       //  initial registration should be strictly 
       if(!initPose){
           if(ndt.getFitnessScore() > 0.1)
               return;
         }
        else{
               initPose=true;
       }
       if (ndt.hasConverged() == false || ndt.getFitnessScore() > ndtFitnessScoreThresh) {
               std::cout << "ndt locolization failed    the score is   " << ndt.getFitnessScore() << std::endl;
                return ;
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
        icp.align(*transCurrentCloudInWorld,transWorldCurrent.matrix());
         //   initial registration should be strictly 
        if(!initPose){
           if(icp.getFitnessScore() > 0.1)
               return;
         }
        else{
               initPose=true;
       }

        if (icp.hasConverged() == false || icp.getFitnessScore() > icpFitnessScoreThresh) {
                std::cout << "ICP locolization failed    the score is   " << icp.getFitnessScore() << std::endl;
                return ;
        } 
        else 
            transWorldCurrent = icp.getFinalTransformation();
    }

    if(useNDT_ICP){
        //先Ndt进行粗配准
        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        ndt.setTransformationEpsilon(ndtTransformationEpsilon);
        ndt.setResolution(ndtResolution);
        ndt.setInputSource(currentFeatureCloud);
        ndt.setInputTarget(globalFeatureCloud);
        pcl::PointCloud<PointType>::Ptr transCurrentCloudInWorld_ndt(new pcl::PointCloud<PointType>());
        ndt.align(*transCurrentCloudInWorld_ndt, transWorldCurrent.matrix()); 
        transWorldCurrent =ndt.getFinalTransformation(); 
        // if (ndt.hasConverged() == false || ndt.getFitnessScore() > ndtFitnessScoreThresh){
        //        std::cout << "ndt locolization failed    the score is   " << ndt.getFitnessScore() << std::endl;
        //        if(cube_ass) {
        //            transWorldCurrent=cubetransWorldCurrent;
        //            std::cout<<"use cube localization"<<std::endl;
        //        }
        //        else return ;
        // }
        // else{
        //     transWorldCurrent =ndt.getFinalTransformation(); 
        //     std::cout << "NDT OK！" << ndt.getFitnessScore() << std::endl;
        // }            
        //再icp进行精配准

        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        //---------------------KD树加速搜索--------------------
        pcl::search::KdTree<PointType>::Ptr tree1(new pcl::search::KdTree<PointType>);
        tree1->setInputCloud(currentFeatureCloud);
        pcl::search::KdTree<PointType>::Ptr tree2(new pcl::search::KdTree<PointType>);
        tree2->setInputCloud(globalFeatureCloud);
        icp.setSearchMethodSource(tree1);
        icp.setSearchMethodTarget(tree2);

        icp.setMaxCorrespondenceDistance(icpMaxCorrespondenceDistance); 
        icp.setMaximumIterations(icpMaximumIterations);
        icp.setTransformationEpsilon(icpTransformationEpsilon);
        icp.setEuclideanFitnessEpsilon(icpEuclideanFitnessEpsilon);
        icp.setInputSource(currentFeatureCloud);
        icp.setInputTarget(globalFeatureCloud);
        pcl::PointCloud<PointType>::Ptr transCurrentCloudInWorld(new pcl::PointCloud<PointType>());
        icp.align(*transCurrentCloudInWorld,transWorldCurrent.matrix()); 
         //   initial registration should be strictly 
        if(!initPose){
        if(icp.getFitnessScore() > 0.1)
            return;
        }
        else{
            initPose=true;
        }

        if (icp.hasConverged() == false || icp.getFitnessScore() > icpFitnessScoreThresh) {
                std::cout << "ICP locolization failed    the score is   " << icp.getFitnessScore() << std::endl;
                if(cube_ass) {
                    transWorldCurrent=cubetransWorldCurrent;
                    std::cout<<"icp failed, use cube localization"<<std::endl;
                }
                else return ;
        } 
        else 
            transWorldCurrent = icp.getFinalTransformation();     
            std::cout << "ICP OK！" << icp.getFitnessScore() << std::endl;
    }

    if(useNDT_GICP){
        //先Ndt进行粗配准
        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        ndt.setTransformationEpsilon(ndtTransformationEpsilon);
        ndt.setResolution(ndtResolution);
        ndt.setInputSource(currentFeatureCloud);
        ndt.setInputTarget(globalFeatureCloud);
        pcl::PointCloud<PointType>::Ptr transCurrentCloudInWorld_ndt(new pcl::PointCloud<PointType>());
        ndt.align(*transCurrentCloudInWorld_ndt, transWorldCurrent.matrix()); 
        transWorldCurrent =ndt.getFinalTransformation();    
        //再gicp进行精配准
        //使用LMICP
        pcl::GeneralizedIterativeClosestPoint<PointType, PointType> lmicp;
        lmicp.setMaxCorrespondenceDistance(icpMaxCorrespondenceDistance); 
        lmicp.setMaximumIterations(icpMaximumIterations);
        lmicp.setTransformationEpsilon(icpTransformationEpsilon);
        lmicp.setEuclideanFitnessEpsilon(icpEuclideanFitnessEpsilon);
        lmicp.setInputSource(currentFeatureCloud);
        lmicp.setInputTarget(globalFeatureCloud);
        pcl::PointCloud<PointType>::Ptr transCurrentCloudInWorld(new pcl::PointCloud<PointType>());
        lmicp.align(*transCurrentCloudInWorld,transWorldCurrent.matrix()); 
         //   initial registration should be strictly 
        if(!initPose){
        if(lmicp.getFitnessScore() > 0.1)
            return;
        }
        else{
            initPose=true;
        }

        if (lmicp.hasConverged() == false || lmicp.getFitnessScore() > icpFitnessScoreThresh) {
                std::cout << "ICP locolization failed    the score is   " << lmicp.getFitnessScore() << std::endl;
                return ;
        } 
        else 
            transWorldCurrent = lmicp.getFinalTransformation();             
    }

    if(useNDT_PICP){
        //先Ndt进行粗配准
        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        ndt.setTransformationEpsilon(ndtTransformationEpsilon);
        ndt.setResolution(ndtResolution);
        ndt.setInputSource(currentFeatureCloud);
        ndt.setInputTarget(globalFeatureCloud);
        pcl::PointCloud<PointType>::Ptr transCurrentCloudInWorld_ndt(new pcl::PointCloud<PointType>());
        ndt.align(*transCurrentCloudInWorld_ndt, transWorldCurrent.matrix()); 
        transWorldCurrent =ndt.getFinalTransformation();    
        //使用点面ICP
        //-----------------拼接点云与法线信息-------------------
        pcl::PointCloud<pcl::PointNormal>::Ptr source_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        cloud_with_normal(currentFeatureCloud, source_with_normals);
        pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        cloud_with_normal(globalFeatureCloud, target_with_normals);
        
        pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
        /*点到面的距离函数构造方法一*/
        pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>::Ptr PointToPlane
        (new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>);
        icp.setTransformationEstimation(PointToPlane);
        icp.setInputSource(source_with_normals);
        icp.setInputTarget(target_with_normals);
        icp.setMaxCorrespondenceDistance(icpMaxCorrespondenceDistance); 
        icp.setMaximumIterations(icpMaximumIterations);
        icp.setTransformationEpsilon(icpTransformationEpsilon);
        icp.setEuclideanFitnessEpsilon(icpEuclideanFitnessEpsilon);
        pcl::PointCloud<pcl::PointNormal>::Ptr transCurrentCloudInWorld(new pcl::PointCloud<pcl::PointNormal>);
        icp.align(*transCurrentCloudInWorld,transWorldCurrent.matrix()); 
         //   initial registration should be strictly 
        if(!initPose){
        if(icp.getFitnessScore() > 0.1)
            return;
        }
        else{
            initPose=true;
        }

        if (icp.hasConverged() == false || icp.getFitnessScore() > icpFitnessScoreThresh) {
                std::cout << "ICP locolization failed    the score is   " << icp.getFitnessScore() << std::endl;
                return ;
        } 
        else 
            transWorldCurrent = icp.getFinalTransformation();             
    }

    if(cube_ass){
        if(currentFeatureCloud->points.size()<150){
            transWorldCurrent=cubetransWorldCurrent;
            std::cout<<"no enough points, use cube localization"<<std::endl;
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
    std::string pose_out = localization_folder+"localization_result.txt";
    std::ofstream out(pose_out.c_str(),std::ios::app);
    out<<time_stamp<<" "<<currentX<<" "<<currentY<<" "<<0<<" "<<cameraPoseQuat.x<<" "<<cameraPoseQuat.y<<" "<<cameraPoseQuat.z<<" "<<cameraPoseQuat.w<<std::endl; 
    out.close();    
	pubCurrentPose.publish(odomAftMapped);

    // std::cout<<"now robot is in x "<<currentX<< " y "<< currentY<< "  z  "<<currentZ<<std::endl;
    // std::cout<<" now robot is in  roll "<<currentRoll<<"  pitch "<<currentPitch<<"  yaw  "<<currentYaw<<std::endl;
   
   
    //   transform  point cloud from current  coordinate to world coordinate
    *currentFeatureCloudInWorld =*transformPointCloud(currentFeatureCloud, transWorldCurrent);
    ros::Time end = ros::Time::now();
    //std::cout<<"localization time: "<<end-start<<std::endl;
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
    Eigen::Quaterniond q;
    q.x() = init_rx;   
    q.y() = init_ry;
    q.z() = init_rz;
    q.w() = init_rw;
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);  
    if(useplane){
        transWorldCurrent=pcl::getTransformation(init_x,init_y,init_z,euler[2],euler[1],euler[0]);
    }
     
      
    systemInitial= true;
}



// load prior global map 
void loadMap(){
         globalFeatureCloud->clear();
         std::cout<<"load map begin ******************"<<std::endl;
         pcl::io::loadPCDFile(localization_folder+"map.pcd", *globalFeatureCloud);
         std::cout<<"load map over ******************"<<std::endl;
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
    nh.param<std::string> ("/data_folder", data_folder, "/home/zjl/Desktop/parking/avp_data/");
    nh.param<std::string> ("/localization_folder", localization_folder, "/home/zjl/Desktop/parking/avp_data/");
    nh.param<int>("invalidColorThresh", invalidColorThresh, 60);
    nh.param<bool>("useICP", useICP, true);
    nh.param<double>("icpMaxCorrespondenceDistance", icpMaxCorrespondenceDistance, 20);
    nh.param<int>("icpMaximumIterations", icpMaximumIterations, 100);
    nh.param<double>("icpTransformationEpsilon", icpTransformationEpsilon, 1e-10);
    nh.param<double>("icpEuclideanFitnessEpsilon", icpEuclideanFitnessEpsilon, 0.001);
    nh.param<double>("icpFitnessScoreThresh", icpFitnessScoreThresh, 0.3);
    nh.param<bool>("useNDT", useNDT, true);
    nh.param<double>("ndtTransformationEpsilon", ndtTransformationEpsilon, 1e-10);
    nh.param<double>("ndtResolution", ndtResolution, 0.1);
    nh.param<double>("ndtFitnessScoreThresh", ndtFitnessScoreThresh, 0.001);
    nh.param<bool>("useplane", useplane, true);
    nh.param<bool>("useNDT_ICP", useNDT_ICP, true);
    nh.param<bool>("useNDT_GICP", useNDT_GICP, true);
    nh.param<bool>("useNDT_PICP", useNDT_PICP, true);
    nh.param<bool>("use_sac_ndt", use_sac_ndt, true);
    nh.param<bool>("cube_localization", cube_localization, true);
    nh.param<bool>("cube_ass", cube_ass, false);
    nh.param<double>("init_x", init_x, 0.001);
    nh.param<double>("init_y", init_y, 0.001);
    nh.param<double>("init_z", init_z, 0.001);
    nh.param<double>("init_rx", init_rx, 0.001);
    nh.param<double>("init_ry", init_ry, 0.001);
    nh.param<double>("init_rz", init_rz, 0.001);
    nh.param<double>("init_rw", init_rw, 0.001);
    nh.param<int>("start_frame", start_frame, 99);
    nh.param<int>("end_frame", end_frame, 3570);
    ros::Subscriber subcameraCloud = nh.subscribe<sensor_msgs::PointCloud2>("/cameraCloudFrame", 100, cameraCloudHandler);
    ros::Subscriber subInitpose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialposeGive", 1,  initposeHandler,  ros::TransportHints().tcpNoDelay());
    
    pubCurrentFeature = nh.advertise<sensor_msgs::PointCloud2>("/currentFeature", 100);
    pubCurrentFeatureInWorld= nh.advertise<sensor_msgs::PointCloud2>("/currentFeatureInWorld", 100);
    pubGlobalFeature=nh.advertise<sensor_msgs::PointCloud2>("/globalFeatureMap", 100);
    pubCurrentPose = nh.advertise<nav_msgs::Odometry>("/currentPose", 100);

    pub_cube = nh.advertise<visualization_msgs::MarkerArray>("/cube_map", 1);
    if(uselidar){
        readposes();
    }
    //readposes();
    readcubemap();
    if(cube_ass) 
    {
        std::string pred_3d_obj_txt = data_folder + "whole_cube/";
        ReadALLOjbecttxt(pred_3d_obj_txt,0,end_frame);
    }
    // load prior global map
    loadMap();
    
    // initialize pose and system
    systemInit();

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        rate.sleep();
    }

    std::cout<<"hello slam"<<std::endl;
    return 0;
}



























