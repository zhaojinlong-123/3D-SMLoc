#include <iostream>
#include <string> 
#include <fstream>
#include <sstream>
#include <ctime>
#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/Geometry> 

#include <ros/ros.h> 
#include <ros/package.h>
#include "matrix_utils.h"
#include "g2o_object.h"
#include "object_landmark.h"
#include "Frame.h"
#include "objectset.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
using namespace std;
using namespace Eigen;

// global variable
std::string data_folder;
std::string save_directory;
int scene;
int run_mode=0;
int start_frame = 0;
int end_frame = 0;
int markerid = 0;
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

geometry_msgs::Pose posenode_to_geomsgs(const g2o::SE3Quat &pose_Twc)
{
    geometry_msgs::Pose pose_msg;    
    Eigen::Vector3d pose_trans = pose_Twc.translation();	
    pose_msg.position.x=pose_trans(0);
    pose_msg.position.y=pose_trans(1);
    pose_msg.position.z=pose_trans(2);
    Eigen::Quaterniond pose_quat = pose_Twc.rotation();
    pose_msg.orientation.x = pose_quat.x();  
    pose_msg.orientation.y = pose_quat.y();
    pose_msg.orientation.z = pose_quat.z();
    pose_msg.orientation.w = pose_quat.w();
    return pose_msg;
}
nav_msgs::Odometry posenode_to_odommsgs(const g2o::SE3Quat &pose_Twc,const std_msgs::Header &img_header)
{
    nav_msgs::Odometry odom_msg;
    odom_msg.pose.pose=posenode_to_geomsgs(pose_Twc);    
    odom_msg.header=img_header;
    return odom_msg;
}

// // one cuboid need front and back markers...
// void cuboid_corner_to_marker(const Matrix38d& cube_corners,visualization_msgs::Marker& marker, int bodyOrfront)
// {
//     Eigen::VectorXd edge_pt_ids;
//     // body edges
//     //按顺序连接各个点，连成一个立方体
//     if (bodyOrfront==0) 
//     { 
// 	    edge_pt_ids.resize(16); 
//         edge_pt_ids<<1,2,3,4,1,5,6,7,8,5,6,2,3,7,8,4;
//         edge_pt_ids.array()-=1;
//     }
//     // front face edges
//     //按顺序连接各个点连成平面
//     else 
//     { 
// 	    edge_pt_ids.resize(5);
//         edge_pt_ids<<1,2,6,5,1;
//         edge_pt_ids.array()-=1;
//     }
//     marker.points.resize(edge_pt_ids.rows());
//     //确定各个点的位置，连成线
//     for (int pt_id=0; pt_id<edge_pt_ids.rows(); pt_id++)
//     {
// 	marker.points[pt_id].x = cube_corners(0, edge_pt_ids(pt_id));
// 	marker.points[pt_id].y = cube_corners(1, edge_pt_ids(pt_id));
// 	marker.points[pt_id].z = cube_corners(2, edge_pt_ids(pt_id));
//     }
// }
// // one cuboid need front and back markers...  rgbcolor is 0-1 based
// visualization_msgs::MarkerArray cuboids_to_marker(std::vector<object_landmark*> obj_landmark, Vector3d rgbcolor, int frame_number) 
// {
//     visualization_msgs::MarkerArray plane_markers;
//     visualization_msgs::Marker marker;
//     marker.header.frame_id="/map";  marker.header.stamp=ros::Time::now();
//     marker.id = frame_number*20; //0
//     marker.type = visualization_msgs::Marker::LINE_STRIP;   marker.action = visualization_msgs::Marker::ADD;
//     //marker.lifetime = ros::Duration(0.2);
//     marker.color.r = rgbcolor(0); marker.color.g = rgbcolor(1); marker.color.b = rgbcolor(2); marker.color.a = 1.0;
//     marker.scale.x = 0.1;
//     for(int i=0; i < obj_landmark.size(); i++)
//     {
//     if (obj_landmark[i]==nullptr)
// 	return plane_markers;

//     g2o::cuboid cube_opti = obj_landmark[i]->cube_vertex->estimate();
//     //cout <<"cube: "<<cube_opti.scale<<"pose: "<<cube_opti.toMinimalVector()<<endl;
//     Eigen::MatrixXd cube_corners = cube_opti.compute3D_BoxCorner();
    
//     for (int ii=0;ii<2;ii++) // each cuboid needs two markers!!! one for all edges, one for front facing edge, could with different color.
//     {
// 	marker.id++;
// 	cuboid_corner_to_marker(cube_corners,marker, ii);
// 	plane_markers.markers.push_back(marker);
//     }
//     }
//     return plane_markers;
// }

visualization_msgs::MarkerArray cuboids_to_marker2(std::vector<object_landmark*> obj_landmark, Vector3d rgbcolor, int frame_number) 
{
    visualization_msgs::MarkerArray plane_markers;
    visualization_msgs::Marker marker;
    marker.header.frame_id="/world";  marker.header.stamp=ros::Time::now();
    marker.id = markerid; //0
    marker.type = visualization_msgs::Marker::CUBE;   marker.action = visualization_msgs::Marker::ADD;
    //marker.lifetime = ros::Duration(0.2);
    marker.color.r = rgbcolor(0); marker.color.g = rgbcolor(1); marker.color.b = rgbcolor(2); marker.color.a = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 3;
    for(int i=0; i < obj_landmark.size(); i++)
    {
    if (obj_landmark[i]==nullptr)
	return plane_markers;

    g2o::cuboid cube_opti = obj_landmark[i]->cube_vertex->estimate();
    //cout <<"cube: "<<cube_opti.scale<<"pose: "<<cube_opti.toMinimalVector()<<endl;
    //Eigen::MatrixXd cube_corners = cube_opti.compute3D_BoxCorner();
    Vector6d position = cube_opti.pose.toXYZPRYVector();
    marker.pose.position.x = position[0];
    marker.pose.position.y = position[1];
    marker.pose.position.z = 1.5; //position[2];
    marker.id++;
    markerid++;
    plane_markers.markers.push_back(marker);
    }
    return plane_markers;
}

void publish_all_poses(std::vector<tracking_frame*> all_frames,std::vector<all_frame_object_landmark*> cube_landmarks_history,
		       std::vector<all_frame_object_landmark*> all_frame_rawcubes, Eigen::MatrixXd& truth_frame_poses)
{
    ros::NodeHandle n;

    // image_transport::ImageTransport it(n);
    // image_transport::Publisher pub = it.advertise("/camera", 1);

    // image_transport::Publisher pub1 = it.advertise("/front", 1);
    // image_transport::Publisher pub2 = it.advertise("/back", 1);
    // image_transport::Publisher pub3 = it.advertise("/left", 1);
    // image_transport::Publisher pub4 = it.advertise("/right", 1);
    // image_transport::Publisher pub5 = it.advertise("/surround", 1);
    // image_transport::Publisher pub6 = it.advertise("/image_color", 1);

    // std::ifstream in("/home/zjl/avpslam_ws/color.txt");
    // //std::ifstream in("/home/zjl/avpslam_ws/11-18/front.txt");
    // std::string line;
    // std_msgs::Header header;

    // std_msgs::Header header1;
    // std_msgs::Header header2;
    // std_msgs::Header header3;
    // std_msgs::Header header4;
    // std_msgs::Header header5;
    // std_msgs::Header header6;




    //估计的和真实的相机位姿（一次性显示）
    //ros::Publisher pub_slam_all_poses = n.advertise<geometry_msgs::PoseArray>("/slam_pose_array", 10);
    //ros::Publisher pub_truth_all_poses = n.advertise<geometry_msgs::PoseArray>("/truth_pose_array", 10);

    //估计的和真实的相机位姿（逐帧显示）
    ros::Publisher pub_slam_odompose = n.advertise<nav_msgs::Odometry>("/slam_odom_pose", 1);
    //ros::Publisher pub_truth_odompose = n.advertise<nav_msgs::Odometry>("/truth_odom_pose", 1);

    //估计的和真实的相机（运动轨迹）
    //ros::Publisher pub_slam_path = n.advertise<nav_msgs::Path>( "/slam_pose_paths", 1);
    ros::Publisher pub_truth_path = n.advertise<nav_msgs::Path>( "/truth_pose_paths", 1);

    //优化后和优化前立方体
    ros::Publisher pub_history_opti_cube = n.advertise<visualization_msgs::MarkerArray>("/cubes_opti_hist", 1); // landmark cube pose after each optimization
    ros::Publisher pub_frame_raw_cube = n.advertise<visualization_msgs::MarkerArray>("/cubes_raw_frame", 1);

    int total_frame_number = all_frames.size();
    cout<< "total_frame_number: " << total_frame_number<<endl;
    //估计的相机位姿
    //geometry_msgs::PoseArray all_pred_pose_array; //存为数组，一次性发布
    std::vector<nav_msgs::Odometry> all_pred_pose_odoms; //存为向量，逐帧发布

    //真实的相机位姿
    //geometry_msgs::PoseArray all_truth_pose_array;
    std::vector<nav_msgs::Odometry> all_truth_pose_odoms;

    //pose_Header 指明在世界坐标系和时间戳
    std_msgs::Header pose_header;    pose_header.frame_id = "/world";    pose_header.stamp = ros::Time::now();

    //运动轨迹，包含位姿和时间戳
    nav_msgs::Path path_truths,path_preds;
    //path_preds.header = pose_header;    
    path_truths.header = pose_header;   

    for (int i = 0; i < total_frame_number; i++)
    {
	//all_pred_pose_array.poses.push_back(posenode_to_geomsgs(all_frames[i]->cam_pose_Twc));
	all_pred_pose_odoms.push_back(posenode_to_odommsgs(all_frames[i]->cam_pose_Twc,pose_header) );
	
	geometry_msgs::PoseStamped postamp;
	postamp.pose = posenode_to_geomsgs(all_frames[i]->cam_pose_Twc);
	postamp.header = pose_header;
	path_preds.poses.push_back(postamp);
    }

    if (truth_frame_poses.rows()>0)
    {
    //读取真实位姿
	for (int i=0; i < total_frame_number;i++)
	{
	    geometry_msgs::Pose pose_msg;
	    pose_msg.position.x=truth_frame_poses(i,1);    pose_msg.position.y=truth_frame_poses(i,2);    pose_msg.position.z=truth_frame_poses(i,3);
	    pose_msg.orientation.x = truth_frame_poses(i,4);	pose_msg.orientation.y = truth_frame_poses(i,5);
	    pose_msg.orientation.z = truth_frame_poses(i,6);	pose_msg.orientation.w = truth_frame_poses(i,7);
	    //all_truth_pose_array.poses.push_back(pose_msg);
	    // nav_msgs::Odometry odom_msg;odom_msg.pose.pose=pose_msg;
	    // odom_msg.header = pose_header;
	    // all_truth_pose_odoms.push_back(odom_msg);
	    
	    geometry_msgs::PoseStamped postamp;
	    postamp.pose = pose_msg;
	    postamp.header = pose_header;
	    path_truths.poses.push_back(postamp);
	}
    }
    //all_pred_pose_array.header.stamp=ros::Time::now();    all_pred_pose_array.header.frame_id="/world";
    //all_truth_pose_array.header.stamp=ros::Time::now();    all_truth_pose_array.header.frame_id="/world";


    ros::Rate loop_rate(10);  //5    
    int frame_number = -1;
    // int ii = 99;
    //int ii = 0;
    while ( n.ok() )
    {
    frame_number++;
    // std::ostringstream stringStream;
    // stringStream << "/home/zjl/Desktop/parking/avp_data/whole_traj/" << ii << ".png";
    // //stringStream << "/home/zjl/Desktop/parking/avp_data/11-18/" << ii << ".png";
    // cv::Mat image = cv::imread(stringStream.str(), CV_LOAD_IMAGE_COLOR);
    // if (image.empty()) {
    //     ROS_ERROR_STREAM("Failed to capture image!");
    //     ros::shutdown();
    // }

    // std::ostringstream stringStream1;
    // stringStream1 << "/media/zjl/My Passport/AVP-dataset/2022-3-24-11-57-11/front/" << num2str(ii) << ".png";
    // cv::Mat image1 = cv::imread(stringStream1.str(), CV_LOAD_IMAGE_COLOR);
    // if (image1.empty()) {
    //     ROS_ERROR_STREAM("Failed to capture image1!");
    //     ros::shutdown();
    // }
    // std::ostringstream stringStream2;
    // stringStream2 << "/media/zjl/My Passport/AVP-dataset/2022-3-24-11-57-11/back/" << num2str(ii) << ".png";
    // cv::Mat image2 = cv::imread(stringStream2.str(), CV_LOAD_IMAGE_COLOR);
    // if (image2.empty()) {
    //     ROS_ERROR_STREAM("Failed to capture image2!");
    //     ros::shutdown();
    // }
    // std::ostringstream stringStream3;
    // stringStream3 << "/media/zjl/My Passport/AVP-dataset/2022-3-24-11-57-11/left/" << num2str(ii) << ".png";
    // cv::Mat image3 = cv::imread(stringStream3.str(), CV_LOAD_IMAGE_COLOR);
    // if (image3.empty()) {
    //     ROS_ERROR_STREAM("Failed to capture image3!");
    //     ros::shutdown();
    // }
    // std::ostringstream stringStream4;
    // stringStream4 << "/media/zjl/My Passport/AVP-dataset/2022-3-24-11-57-11/right/" << num2str(ii) << ".png";
    // cv::Mat image4 = cv::imread(stringStream4.str(), CV_LOAD_IMAGE_COLOR);
    // if (image4.empty()) {
    //     ROS_ERROR_STREAM("Failed to capture image4!");
    //     ros::shutdown();
    // }

    // std::ostringstream stringStream5;
    // stringStream5 << "/media/zjl/My Passport/AVP-dataset/2022-3-24-11-57-11/surround/" << num2str(ii) << ".png";
    // cv::Mat image5 = cv::imread(stringStream5.str(), CV_LOAD_IMAGE_COLOR);
    // if (image5.empty()) {
    //     ROS_ERROR_STREAM("Failed to capture image5!");
    //     ros::shutdown();
    // }
    // std::ostringstream stringStream6;
    // stringStream6 << "/media/zjl/My Passport/AVP-dataset/2022-3-24-11-57-11/image_color/" << num2str(ii) << ".png";
    // cv::Mat image6 = cv::imread(stringStream6.str(), CV_LOAD_IMAGE_COLOR);
    // if (image6.empty()) {
    //     ROS_ERROR_STREAM("Failed to capture image6!");
    //     ros::shutdown();
    // }

    // std::getline(in,line);
    // header.frame_id="camera0_link";
    // header.stamp=string2Time(line);
    // header.seq = 1;
    // //cv::resize(image,image,cv::Size(480,640));
    // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    // pub.publish(msg);

    // header1.frame_id="camera0_link";
    // header1.stamp=string2Time(line);
    // header1.seq = 1;
    // //cv::resize(image,image,cv::Size(480,640));
    // sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(header1, "bgr8", image1).toImageMsg();
    // pub1.publish(msg1);

    // header2.frame_id="camera0_link";
    // header2.stamp=string2Time(line);
    // header2.seq = 1;
    // //cv::resize(image,image,cv::Size(480,640));
    // sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(header2, "bgr8", image2).toImageMsg();
    // pub2.publish(msg2);

    // header3.frame_id="camera0_link";
    // header3.stamp=string2Time(line);
    // header3.seq = 1;
    // //cv::resize(image,image,cv::Size(480,640));
    // sensor_msgs::ImagePtr msg3 = cv_bridge::CvImage(header3, "bgr8", image3).toImageMsg();
    // pub3.publish(msg3);

    // header4.frame_id="camera0_link";
    // header4.stamp=string2Time(line);
    // header4.seq = 1;
    // //cv::resize(image,image,cv::Size(480,640));
    // sensor_msgs::ImagePtr msg4 = cv_bridge::CvImage(header4, "bgr8", image4).toImageMsg();
    // pub4.publish(msg4);

    // header5.frame_id="camera0_link";
    // header5.stamp=string2Time(line);
    // header5.seq = 1;
    // //cv::resize(image,image,cv::Size(480,640));
    // sensor_msgs::ImagePtr msg5 = cv_bridge::CvImage(header5, "bgr8", image5).toImageMsg();
    // pub5.publish(msg5);

    // header6.frame_id="camera0_link";
    // header6.stamp=string2Time(line);
    // header6.seq = 1;
    // //cv::resize(image,image,cv::Size(480,640));
    // sensor_msgs::ImagePtr msg6 = cv_bridge::CvImage(header6, "bgr8", image6).toImageMsg();
    // pub6.publish(msg6);      

    // ii=ii+1;


    if (1) // directly show final results
	{
	    //pub_slam_all_poses.publish(all_pred_pose_array);	pub_truth_all_poses.publish(all_truth_pose_array);
	    //pub_slam_path.publish(path_preds);	
        pub_truth_path.publish(path_truths);
	}

	if (frame_number<total_frame_number)
	{
	    // publish cuboid landmarks, after each frame's g2o optimization
	    if (cube_landmarks_history[frame_number]!=nullptr)
        {
            //cout<<"frame: "<<frame_number<<" cube size: "<<cube_landmarks_history[frame_number]->observed_cuboids.size()<<endl;
            pub_history_opti_cube.publish(cuboids_to_marker2(cube_landmarks_history[frame_number]->observed_cuboids,Vector3d(1,1,1),frame_number));
            
        }
	    // publish raw detected cube in each frame, before optimization
	    if (all_frame_rawcubes[frame_number]!=nullptr)
        {
            // for(int k=0;k<all_frame_rawcubes[frame_number]->observed_cuboids.size();k++)
            // {
            // cout<<"frame: "<<frame_number<<"cube: "<<all_frame_rawcubes[frame_number]->observed_cuboids.size()<<" "<<all_frame_rawcubes[frame_number]->observed_cuboids[k]->cube_vertex->estimate().toMinimalVector()<<endl;
            // }
            pub_frame_raw_cube.publish(cuboids_to_marker2(all_frame_rawcubes[frame_number]->observed_cuboids,Vector3d(0,0,1),frame_number));
        }

	    // publish camera pose estimation of this frame
	    pub_slam_odompose.publish(all_pred_pose_odoms[frame_number]);
	    //pub_truth_odompose.publish(all_truth_pose_odoms[frame_number]);
	    
// 	    std::cout<<"Frame position x/y:   "<<frame_number<<"        "<<all_pred_pose_odoms[frame_number].pose.pose.position.x<<"  "<<
// 			  all_pred_pose_odoms[frame_number].pose.pose.position.y <<std::endl;
	}
	
	if (frame_number==int(all_pred_pose_odoms.size()))
	{
	    cout<<"Finish all visulialization!"<<endl;
	}
	
	ros::spinOnce();
	loop_rate.sleep();        
    }

}


void incremental_build_graph(Eigen::MatrixXd& truth_poses, Eigen::MatrixXd& init_poses)
{
    int total_frame_number = end_frame-start_frame;
    //g2o图优化
    //构造一个graph求解器
    g2o::SparseOptimizer graph;
    //使用线性方程求解器linearSolver
    g2o::BlockSolverX::LinearSolverType* linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    //使用稀疏矩阵块求解器solver_ptr
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
    //使用梯度下降法求解
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    //设置求解器
    graph.setAlgorithm(solver);
    //是否打开调试输出
    graph.setVerbose(true);

    g2o::SE3Quat fixed_init_cam_pose_Twc(init_poses.row(0).tail<7>()); //初始帧位姿

    //存储每帧优化前后cube位姿
    std::vector<all_frame_object_landmark*> cube_pose_opti_history(total_frame_number); //landmark pose after each frame's optimization
    std::vector<all_frame_object_landmark*> cube_pose_raw_detected_history(total_frame_number); //raw detected cuboid frame each frame. before optimization

    int offline_cube_obs_row_id = 0;
    //存储每一帧信息
    std::vector<tracking_frame*> all_frames(total_frame_number);    
    g2o::VertexCuboid* vCube; //存储物体到世界坐标系位姿
    int associate_id = 0; //关联物体Id
    //关联后所有物体集合
    std::vector<object_landmark*> all_objects;
    int cube_num = total_frame_number;
    //process each frame incrementally
    for (int frame_id = 0; frame_id<total_frame_number; frame_id++)
    {
        g2o::SE3Quat curr_cam_pose_Twc;  //from cam to world
        g2o::SE3Quat odom_val; //从上一帧到当前帧位姿

        if (frame_id==0)
            curr_cam_pose_Twc = fixed_init_cam_pose_Twc;
        else
        {
            // g2o::SE3Quat curr_pose_Tcw = all_frames[frame_id]->cam_pose_Tcw;
            // if(frame_id>1)
            // {
            // g2o::SE3Quat prev_pose_Tcw = all_frames[frame_id-1]->cam_pose_Tcw;
            // odom_val = curr_pose_Tcw*prev_pose_Tcw.inverse();
            // }
            g2o::SE3Quat odom_init_cam_pose_Twc(init_poses.row(frame_id).tail<7>());
            g2o::SE3Quat pre_cam_pose_Twc(init_poses.row(frame_id-1).tail<7>());
            curr_cam_pose_Twc = odom_init_cam_pose_Twc; //odom估计的初始位姿
            odom_val = curr_cam_pose_Twc.inverse()*pre_cam_pose_Twc; //从上一帧到当前帧位姿变换
        }

        //将当前帧信息存放到all_frame中
        tracking_frame* currframe = new tracking_frame();
        currframe->frame_seq_id = frame_id;
        all_frames[frame_id] = currframe;

        bool has_detected_cuboid = false;
        g2o::cuboid cube_local_meas; double proposal_error;

        //read cubes
        Eigen::MatrixXd pred_frame_objects;
        pred_frame_objects = all_offline_object_cubes[frame_id];
        if(pred_frame_objects.rows()!=0)
            has_detected_cuboid = true;
        if (has_detected_cuboid)
        {
            for (int i=0; i<pred_frame_objects.rows();i++)
            {
                cuboid *raw_cuboid = new cuboid();
                raw_cuboid->pos = pred_frame_objects.row(i).head(3);
                raw_cuboid->rotY = pred_frame_objects(i, 3);
                raw_cuboid->scale = Vector3d(pred_frame_objects(i, 4), pred_frame_objects(i, 5), pred_frame_objects(i, 6));
                //raw_cuboid->rect_detect_2d = pred_frame_objects.row(i).segment<4>(7);
                //raw_cuboid->object_id = pred_frame_objects(i, 11);
                raw_cuboid->object_type_id = pred_frame_objects(i, 7);
                raw_cuboid->prob = pred_frame_objects(i, 8);
                //all_obj2d_bbox.push_back(raw_cuboid->rect_detect_2d);
                //all_box_confidence.push_back(pred_frame_objects(i, 11));

                g2o::cuboid cube_ground_value;
                Vector9d cube_pose;
                cube_pose << raw_cuboid->pos[0], raw_cuboid->pos[1], raw_cuboid->pos[2], 0, 0, raw_cuboid->rotY,
				raw_cuboid->scale[0], raw_cuboid->scale[1], raw_cuboid->scale[2]; // xyz roll pitch yaw scale
                //滤除15m以外的方块
                double distance_cube = sqrt(pow(raw_cuboid->pos[0],2)+pow(raw_cuboid->pos[1],2)+pow(raw_cuboid->pos[2],2));
                if(distance_cube>15){
                    continue;
                }
                //将9自由度位姿转成g2o::cuboid格式, 当前帧坐标系下
                cube_ground_value.fromMinimalVector(cube_pose);
                //相机位姿，cam to world
                Eigen::VectorXd cam_pose_vec = init_poses.row(frame_id);
                g2o::SE3Quat cam_val_Twc(cam_pose_vec.segment<7>(1)); // time x y z qx qy qz qw
                // transform a global cuboid to local cuboid
                cube_local_meas = cube_ground_value;//.transform_to(cam_val_Twc); // measurement is in local camera frame
                //cout<<"cube lcoal: "<<cube_local_meas.toMinimalVector()<<endl;
                proposal_error = pred_frame_objects(i, 8);
                object_landmark* localcuboid = new object_landmark();
                localcuboid->cube_meas = cube_local_meas;
                //置信度
                localcuboid->meas_quality = proposal_error;
                //初始化track id
                localcuboid->track_id = 0;
                currframe->observed_cuboids.push_back(localcuboid);

                //ObjectSet temp;
                //temp.push_back(raw_cuboid);
                //all_obj_cubes.push_back(temp);
            }
        }

        //数据关联
        bool data_associate = 1;
        bool associate_with_quality = 0;
        cout<<"associate id: "<<associate_id<<endl;
        if(has_detected_cuboid && data_associate)
        {
            for(int m=0;m<currframe->observed_cuboids.size();m++)
            {
                //相机位姿，cam to world
                Eigen::VectorXd cam_pose_vec = init_poses.row(frame_id);
                g2o::SE3Quat cam_val_Twc(cam_pose_vec.segment<7>(1)); // time x y z qx qy qz qw
                g2o::cuboid currt_cube = currframe->observed_cuboids[m]->cube_meas;
                g2o::cuboid global_currt_cube = currt_cube.transform_from(cam_val_Twc);
                object_landmark* globalcuboid = new object_landmark();
                globalcuboid->cube_meas = global_currt_cube;
                globalcuboid->meas_quality = currframe->observed_cuboids[m]->meas_quality;
                globalcuboid->track_id = currframe->observed_cuboids[m]->track_id;
                //第0帧，直接分配id
                if(frame_id==0)
                {
                    currframe->observed_cuboids[m]->track_id = associate_id;
                    all_objects.push_back(globalcuboid);
                    associate_id++;
                }
                //小于20帧，与之前所有帧物体匹配
                bool has_associate = false;
                if(frame_id>0&&frame_id<20)
                { 
                    for(int f=0;f<frame_id;f=f+1){
                        if(all_frames[f]->observed_cuboids.size()!=0)
                        {
                        for (int h=0;h<all_frames[f]->observed_cuboids.size();h++){
                            Vector9d res;
                            Vector6d diff;
                            g2o::cuboid pre_cube = all_frames[f]->observed_cuboids[h]->cube_meas;
                            Eigen::VectorXd pre_cam_pose_vec = init_poses.row(f);
                            g2o::SE3Quat pre_cam_val_Twc(pre_cam_pose_vec.segment<7>(1));
                            g2o::cuboid global_pre_cube = pre_cube.transform_from(pre_cam_val_Twc);
                            g2o::cuboid tmp_global_currt_cube = global_currt_cube;
                            g2o::SE3Quat pose_diff = tmp_global_currt_cube.pose.inverse()*global_pre_cube.pose;
                            diff = pose_diff.toXYZPRYVector();
                            //res.head<6>() = pose_diff.log(); //treat as se3 log error. could also just use yaw error
                            //res.tail<3>() = currframe->observed_cuboids[m]->cube_meas.scale - all_frames[f]->observed_cuboids[h]->cube_meas.scale;
                            double squre = diff.head<3>().array().abs().maxCoeff();
                            if(squre<2){
                                if(associate_with_quality){
                                    double curr_quality = currframe->observed_cuboids[m]->meas_quality;
                                    double pre_quality = all_frames[f]->observed_cuboids[h]->meas_quality;
                                    if(curr_quality>pre_quality){
                                        //g2o::cuboid tmp2_global_currt_cube = global_currt_cube;
                                        int track_id = all_frames[f]->observed_cuboids[h]->track_id;
                                        currframe->observed_cuboids[m]->track_id = all_frames[f]->observed_cuboids[h]->track_id;
                                        //all_frames[f]->observed_cuboids[h]->cube_meas = tmp2_global_currt_cube.transform_to(cam_val_Twc);
                                        //all_frames[f]->observed_cuboids[h]->meas_quality = curr_quality;
                                        all_objects[track_id] = globalcuboid;
                                    }
                                    else{
                                        currframe->observed_cuboids[m]->track_id = all_frames[f]->observed_cuboids[h]->track_id;
                                        currframe->observed_cuboids[m]->cube_meas = global_pre_cube.transform_to(cam_val_Twc);
                                        currframe->observed_cuboids[m]->meas_quality = pre_quality;
                                    }
                                }
                                //currframe->observed_cuboids[m] = all_frames[f]->observed_cuboids[h];
                                if(!associate_with_quality){
                                    currframe->observed_cuboids[m]->track_id = all_frames[f]->observed_cuboids[h]->track_id;
                                    currframe->observed_cuboids[m]->cube_meas = global_pre_cube.transform_to(cam_val_Twc);
                                }
                                has_associate = true;
                                break;
                                // currframe->observed_cuboids[m]->track_id = all_frames[f]->observed_cuboids[h]->track_id;
                                // currframe->observed_cuboids[m]->cube_meas = all_frames[f]->observed_cuboids[h]->cube_meas;
                                // currframe->observed_cuboids[m]->cube_meas = all_frames[f]->observed_cuboids[h]->cube_meas;
                                // currframe->observed_cuboids[m]->cube_vertex = all_frames[f]->observed_cuboids[h]->cube_vertex;
                            }
                        }
                        }
                        if(has_associate) break;
                    }
                    if(!has_associate)
                    {
                        currframe->observed_cuboids[m]->track_id = associate_id;
                        all_objects.push_back(globalcuboid);
                        associate_id++;
                    }
                }
                //大于20帧，与前20帧匹配
                if(frame_id>=20)
                {
                    for(int f=frame_id-20;f<frame_id;f=f+1){
                        if(all_frames[f]->observed_cuboids.size()!=0)
                        {
                        for (int h=0;h<all_frames[f]->observed_cuboids.size();h++){
                            Vector9d res;
                            Vector6d diff;
                            g2o::cuboid pre_cube = all_frames[f]->observed_cuboids[h]->cube_meas;
                            Eigen::VectorXd pre_cam_pose_vec = init_poses.row(f);
                            g2o::SE3Quat pre_cam_val_Twc(pre_cam_pose_vec.segment<7>(1));
                            g2o::cuboid global_pre_cube = pre_cube.transform_from(pre_cam_val_Twc);
                            g2o::cuboid tmp_global_currt_cube = global_currt_cube;
                            g2o::SE3Quat pose_diff = tmp_global_currt_cube.pose.inverse()*global_pre_cube.pose;
                            diff = pose_diff.toXYZPRYVector();
                            //res.head<6>() = pose_diff.log(); //treat as se3 log error. could also just use yaw error
                            //res.tail<3>() = currframe->observed_cuboids[m]->cube_meas.scale - all_frames[f]->observed_cuboids[h]->cube_meas.scale;
                            double squre = diff.head<3>().array().abs().maxCoeff();
                            if(squre<2){
                                if(associate_with_quality){
                                    double curr_quality = currframe->observed_cuboids[m]->meas_quality;
                                    double pre_quality = all_frames[f]->observed_cuboids[h]->meas_quality;
                                    if(curr_quality>pre_quality){
                                        //g2o::cuboid tmp2_global_currt_cube = global_currt_cube;
                                        int track_id = all_frames[f]->observed_cuboids[h]->track_id;
                                        currframe->observed_cuboids[m]->track_id = all_frames[f]->observed_cuboids[h]->track_id;
                                        //all_frames[f]->observed_cuboids[h]->cube_meas = tmp2_global_currt_cube.transform_to(cam_val_Twc);
                                        //all_frames[f]->observed_cuboids[h]->meas_quality = curr_quality;
                                        all_objects[track_id] = globalcuboid;
                                    }
                                    else{
                                        currframe->observed_cuboids[m]->track_id = all_frames[f]->observed_cuboids[h]->track_id;
                                        currframe->observed_cuboids[m]->cube_meas = global_pre_cube.transform_to(cam_val_Twc);
                                        currframe->observed_cuboids[m]->meas_quality = pre_quality;
                                    }
                                }
                                //currframe->observed_cuboids[m] = all_frames[f]->observed_cuboids[h];
                                if(!associate_with_quality){
                                    currframe->observed_cuboids[m]->track_id = all_frames[f]->observed_cuboids[h]->track_id;
                                    currframe->observed_cuboids[m]->cube_meas = global_pre_cube.transform_to(cam_val_Twc);
                                }
                                has_associate = true;
                                break;
                                // currframe->observed_cuboids[m]->track_id = all_frames[f]->observed_cuboids[h]->track_id;
                                // currframe->observed_cuboids[m]->cube_meas = all_frames[f]->observed_cuboids[h]->cube_meas;
                                // currframe->observed_cuboids[m]->cube_meas = all_frames[f]->observed_cuboids[h]->cube_meas;
                                // currframe->observed_cuboids[m]->cube_vertex = all_frames[f]->observed_cuboids[h]->cube_vertex;
                            }
                        }
                        }
                        if(has_associate) break;
                    }
                    if(!has_associate)
                    {
                        //loop detection
                        //在地图上寻找当前帧最近邻物体，如果小于阈值则对物体进行闭环
                        //当数据没有关联上，且大于一定帧数进行闭环检测
                        bool loop_detection = 1;
                        bool have_loop = 0;
                        if(loop_detection)
                        {
                            //Eigen::MatrixXd pose_after_loop(init_poses.rows(),init_poses.cols());
                            if(frame_id>200){
                                for(int k=0;k<all_objects.size();k++){
                                    g2o::cuboid tmp_global_cube = all_objects[k]->cube_meas;
                                    g2o::cuboid tmp_global_currt_cube = global_currt_cube;
                                    g2o::SE3Quat res = tmp_global_currt_cube.pose.inverse()*tmp_global_cube.pose;
                                    Vector6d diff = res.toXYZPRYVector();
                                    double squre = diff.head<3>().array().abs().maxCoeff();
                                    if(squre<8){
                                        have_loop = 1;
                                        cout<<"has loop!"<<endl;
                                        currframe->observed_cuboids[m]->track_id = all_objects[k]->track_id;
                                        currframe->observed_cuboids[m]->cube_meas = tmp_global_cube.transform_to(cam_val_Twc);
                                        break;
                                    }
                                }
                            }
                        }
                        if(!have_loop){
                            currframe->observed_cuboids[m]->track_id = associate_id;
                            all_objects.push_back(globalcuboid);
                            associate_id++;
                        }

                    }                    
                }
            }
        }




	    // set up g2o camera vertex
	    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();       
        currframe->pose_vertex = vSE3;    
        vSE3->setId(frame_id);
        graph.addVertex(vSE3);
        vSE3->setEstimate(curr_cam_pose_Twc.inverse());
        vSE3->setFixed(frame_id==0);

        // camera vertex, add cam-cam odometry edges
        if (frame_id>0)
        {
            g2o::EdgeSE3Expmap* e = new g2o::EdgeSE3Expmap();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>( all_frames[frame_id-1]->pose_vertex ));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>( all_frames[frame_id]->pose_vertex ));
            e->setMeasurement(odom_val);

            e->setId(frame_id);
            //Matrix6d info = Matrix6d::Identity();
            //e->setInformation(info);
            graph.addEdge(e);
        }

        // set up g2o cube vertex
        for (int i=0; i<currframe->observed_cuboids.size();i++){
            g2o::cuboid init_cuboid_global_pose = currframe->observed_cuboids[i]->cube_meas.transform_from(curr_cam_pose_Twc);
            vCube = new g2o::VertexCuboid();
            currframe->observed_cuboids[i]->cube_vertex = vCube;
            vCube->setEstimate(init_cuboid_global_pose);
            vCube->setId(cube_num);
            vCube->setFixed(false);
            graph.addVertex(vCube);
            cube_num++;   
        }

        // add g2o camera-object measurement edges, if there is
        if(currframe->observed_cuboids.size()>0)
        {
            for (int i=0;i<currframe->observed_cuboids.size();i++)
            {
                object_landmark* cube_landmaek_meas = all_frames[frame_id]->observed_cuboids[i];
                g2o::EdgeSE3Cuboid* e = new g2o::EdgeSE3Cuboid();
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(all_frames[frame_id]->pose_vertex));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(currframe->observed_cuboids[i]->cube_vertex));
                e->setMeasurement(cube_landmaek_meas->cube_meas);
                e->setId(total_frame_number+frame_id);
                //Matrix9d info = Matrix9d::Identity();
                //e->setInformation(info);
                graph.addEdge(e);
            }
        }
        //graph.initializeOptimization();
        //graph.optimize(5); // do optimization! 
        // for (int j=0;j<frame_id;j++)
        // {
        //     all_frames[j]->cam_pose_Tcw = all_frames[j]->pose_vertex->estimate();
        //     all_frames[j]->cam_pose_Twc = all_frames[j]->cam_pose_Tcw.inverse();
        // }
        all_frames[frame_id]->cam_pose_Tcw = curr_cam_pose_Twc.inverse();
        all_frames[frame_id]->cam_pose_Twc = curr_cam_pose_Twc;
        //cout<<"pose: "<<all_frames[frame_id]->cam_pose_Tcw.log()<<endl;
        //std::vector<object_landmark*> cubes_pose;
        all_frame_object_landmark* curr_cubes_poses = new all_frame_object_landmark();
        if(all_frames[frame_id]->observed_cuboids.size()>0)
        {
            for(int k=0;k<all_frames[frame_id]->observed_cuboids.size();k++)
            {
                object_landmark* cube_landmark_meas = all_frames[frame_id]->observed_cuboids[k];
                g2o::cuboid local_cube = cube_landmark_meas->cube_meas;
                //cout<<"frame id: "<<frame_id<<"pose: "<<all_frames[frame_id]->cam_pose_Twc.toXYZPRYVector()<<endl;
                //cout<<"frame id: "<<frame_id<<"local_cube: "<<local_cube.toMinimalVector()<<endl;
                g2o::cuboid global_cube = local_cube.transform_from(all_frames[frame_id]->cam_pose_Twc);
                //cout<<"frame id: "<<frame_id<<"global_cube: "<<global_cube.toMinimalVector()<<endl;
                object_landmark* tempcuboids2 = new object_landmark();
                tempcuboids2->cube_vertex = new g2o::VertexCuboid();
                tempcuboids2->cube_vertex->setEstimate(global_cube);
                curr_cubes_poses->observed_cuboids.push_back(tempcuboids2);
            }
            cube_pose_raw_detected_history[frame_id] = curr_cubes_poses;
        }
        else
        {
            cube_pose_raw_detected_history[frame_id] = nullptr;
            //cube_pose_opti_history[frame_id] = nullptr;
        }

    }
    cout<<"all object size: "<<all_objects.size()<<endl;
    std::string object_out=data_folder+"all_objects.txt";
    std::ofstream out(object_out.c_str(),std::ios::app);
    for (int i = 0; i<all_objects.size(); i++){
        Vector9d cube_pose_save; 
        cube_pose_save = all_objects[i]->cube_meas.toMinimalVector();

        out<<cube_pose_save(0)<<" "<<cube_pose_save(1)<<" "<<cube_pose_save(2)<<" "<<\
        cube_pose_save(3)<<" "<<cube_pose_save(4)<<" "<<cube_pose_save(5)<<" "<<\
        cube_pose_save(6)<<" "<<cube_pose_save(7)<<" "<<cube_pose_save(8)<<std::endl;
    }
    out.close();
    cout<<"start optimization!"<<endl;
    //graph.initializeOptimization();
    //graph.optimize(30); // do optimization! 
    cout<<"finish all optimization!"<<endl;

    for (int frame_id = 0; frame_id<total_frame_number; frame_id++)
    {
        all_frame_object_landmark* opt_curr_cubes_poses = new all_frame_object_landmark();
        if(all_frames[frame_id]->observed_cuboids.size()>0)
        {
            for(int k=0;k<all_frames[frame_id]->observed_cuboids.size();k++)
            {
                object_landmark* current_landmark = new object_landmark();
                current_landmark->cube_vertex = new g2o::VertexCuboid();
                current_landmark->cube_vertex->setEstimate(all_frames[frame_id]->observed_cuboids[k]->cube_vertex->estimate());
                opt_curr_cubes_poses->observed_cuboids.push_back(current_landmark);
            }
            cube_pose_opti_history[frame_id] = opt_curr_cubes_poses;
        }
        else
        {
            cube_pose_opti_history[frame_id] = nullptr;
        }
    }
    publish_all_poses(all_frames, cube_pose_opti_history, cube_pose_raw_detected_history,truth_poses);

}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "object_map_node");
    ros::NodeHandle nh;
    nh.param<std::string> ("/data_folder", data_folder, "/home/zjl/");
    nh.param<std::string>("save_directory", save_directory, "/home/zjl/avpslam_ws/src/AVP-SLAM-PLUS/avp_slam_plus/data/");
    nh.param<int>("scene", scene, 1);
    nh.param<int>("run_mode", run_mode, 1);
    nh.param<int>("start_frame", start_frame, 99);
    nh.param<int>("end_frame", end_frame, 3570);
    //read pose truth
    cout<<""<<endl;
    cout<<"data_folder: " <<data_folder<<endl;
    std::string pose_truth_txt;
    std::string pose_init_txt;
    if(run_mode==0){
        pose_truth_txt = save_directory+"odomzjl.txt";//"TUM_odmetry_interpolation.txt";
        pose_init_txt = save_directory+"odomzjl.txt";//"TUM_odmetry_interpolation.txt";        
    }
    else{
        pose_truth_txt = data_folder+"TUM_odmetry_interpolation.txt";//"TUM_odmetry_interpolation.txt";
        pose_init_txt = data_folder+"TUM_odmetry_interpolation.txt";//"TUM_odmetry_interpolation.txt";
    }

    //std:string pose_truth_txt = data_folder+"lego-loam-world-11-18.txt";
    //std::string pose_init_txt = data_folder+"lego-loam-world-11-18.txt";
    //3d cuboid txts:  each row:  [cuboid_center(3), yaw, cuboid_scale(3), [x1 y1 w h]], prob
    std::string pred_3d_obj_txt = data_folder + "whole_cube/";
    //std::string pred_3d_obj_txt = data_folder + "cube/";
    Eigen::MatrixXd truth_poses(100,8);
    Eigen::MatrixXd init_poses(100,8);
    if(run_mode==0){
        read_pose_truth_txt(pose_truth_txt, truth_poses);
        read_pose_truth_txt(pose_init_txt, init_poses);
    }
    else{
        read_pose_truth_txt2(pose_truth_txt, truth_poses,start_frame,end_frame,scene);
        read_pose_truth_txt2(pose_init_txt, init_poses,start_frame,end_frame,scene);
    }

    if (truth_poses.rows()!=init_poses.rows())
        cout<<"dismatch size of pose!"<<endl;
    std::cout<<"pose size: "<<truth_poses.rows()<<endl;

    for (int i=0; i<truth_poses.rows(); i++){
        truth_poses(i,3) = 0;
    }
    for (int i=0; i<init_poses.rows(); i++){
        init_poses(i,3) = 0;
    }

    ReadALLOjbecttxt(pred_3d_obj_txt,start_frame,end_frame);
    //cout<<"cube info:" << all_offline_object_cubes.size()<<" "<<all_offline_object_cubes[0]<<endl;
    incremental_build_graph(truth_poses, init_poses);
    return 0;
}

