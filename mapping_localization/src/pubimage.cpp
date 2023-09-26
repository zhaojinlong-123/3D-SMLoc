#include <ros/ros.h>
#include <stdlib.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
std::string data_folder;
int scene;
int start_frame = 99;
int end_frame = 3570;
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

int main(int argc, char** argv)
{
        ros::init(argc, argv, "image_publisher");
        ros::NodeHandle nh;
        nh.param<int>("start_frame", start_frame, 99);
        nh.param<int>("end_frame", end_frame, 3570);
        nh.param<int>("scene", scene, 1);
        nh.param<std::string> ("/data_folder", data_folder, "/home/zjl/");
        image_transport::ImageTransport it(nh);
        image_transport::Publisher pub = it.advertise("/camera", 1);
        std::string colorpath = data_folder+"color.txt";
        std::ifstream in(colorpath.c_str());
        //std::ifstream in("/home/zjl/Desktop/parking/avp_data/2022-3-12-11-46-58/color.txt");
        //std::ifstream in("/home/zjl/Desktop/parking/avp_data/relocdata/2022-10-23-15-31-3/front.txt");
        //std::ifstream in("/home/zjl/Desktop/parking/avp_data/relocdata/2022-10-23-12-27-23/velodyne.txt");
        //std::ifstream in("/home/zjl/avpslam_ws/11-18/front.txt");
	    std::string line;
        std_msgs::Header header;
        //yt 99 other 0
        int sst=0;
        if(scene==1) sst=99;
        for(int j=sst;j<start_frame;j++){
            std::getline(in,line);
        }
        ros::Rate loop_rate(10);
        for(int i = start_frame; i < end_frame; i = i + 1) //99-600 820-1920 820-1220 99-1220
        {
                if(!nh.ok())
                    break;
                std::ostringstream stringStream;
                if(scene==1){
                    stringStream << data_folder << "whole_traj/" << i << ".png";
                }
                else stringStream << data_folder << "whole_traj/" << num2str(i) << ".png";                
                //stringStream << "/home/zjl/Desktop/parking/avp_data/2022-3-12-11-46-58/seg_result/" << num2str(i) << ".png";
                //stringStream << "/home/zjl/Desktop/parking/avp_data/relocdata/2022-10-23-15-31-3/seg_result/" << num2str(i) << ".png";
                //stringStream << "/home/zjl/Desktop/parking/avp_data/relocdata/2022-10-23-12-27-23/seg_result/" << num2str(i) << ".png";
               // stringStream << "/home/zjl/Desktop/parking/avp_data/11-18/" << i << ".jpg";
                cv::Mat image = cv::imread(stringStream.str(), CV_LOAD_IMAGE_COLOR);
                if (image.empty()) {
                    ROS_ERROR_STREAM("Failed to capture image!");
                    ros::shutdown();
                }            
                std::getline(in,line);
                header.frame_id="camera0_link";
                header.stamp=string2Time(line);
                header.seq = 1;
                //cv::resize(image,image,cv::Size(480,640));
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
                pub.publish(msg);
                ros::spinOnce();
                loop_rate.sleep();
        }
}