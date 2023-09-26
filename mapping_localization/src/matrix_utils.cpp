#include "matrix_utils.h"
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ros/ros.h> 
#include <ros/package.h>
using namespace Eigen;

//声明一个模板，虚拟类型名为T
template <class T>
bool read_pose_truth_txt(const std::string txt_file_name, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &read_number_mat)
{
    if(!std::ifstream(txt_file_name))
    {
        std::cout <<"error!! cannot read txt file "<< txt_file_name << std::endl;
        return false;
    }
    std::ifstream filetxt(txt_file_name.c_str());
    int row_counter = 0;
    std::string line;
    if (read_number_mat.rows() == 0){
        read_number_mat.resize(100,8);
    }
    while (getline(filetxt, line))
    {
        T t;
        if (!line.empty())
        {
            std::stringstream ss(line);
            int colu = 0;
            while (ss >> t)
            {
                read_number_mat(row_counter, colu) = t;
                colu++;
            }
            row_counter++;
            //如果行数不够，就扩大矩阵
            if(row_counter >= read_number_mat.rows())
                read_number_mat.conservativeResize(read_number_mat.rows()*2, read_number_mat.cols());
        }
    }
    filetxt.close();
    //裁剪矩阵多余的行
    read_number_mat.conservativeResize(row_counter,read_number_mat.cols());
    return true;
}

template <class T>
bool read_pose_truth_txt2(const std::string txt_file_name, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &read_number_mat,int start_frame,int end_frame,int scene)
{
    if(!std::ifstream(txt_file_name))
    {
        std::cout <<"error!! cannot read txt file "<< txt_file_name << std::endl;
        return false;
    }
    std::ifstream filetxt(txt_file_name.c_str());
    int row_counter = 0;
    std::string line;
    if (read_number_mat.rows() == 0){
        read_number_mat.resize(100,8);
    }
    int sst=0;
    if(scene==1) sst=99;
    else if(scene==2) sst=2;
    else if(scene==3||scene==4) sst=3;
    for(int j=sst;j<start_frame;j++){
        getline(filetxt,line);
    }
    for(int i = start_frame; i < end_frame; i = i + 1)
    {
        getline(filetxt, line);
        T t;
        if (!line.empty())
        {
            std::stringstream ss(line);
            int colu = 0;
            while (ss >> t)
            {
                read_number_mat(row_counter, colu) = t;
                colu++;
            }
            row_counter++;
            //如果行数不够，就扩大矩阵
            if(row_counter >= read_number_mat.rows())
                read_number_mat.conservativeResize(read_number_mat.rows()*2, read_number_mat.cols());
        }
    }
    filetxt.close();
    //裁剪矩阵多余的行
    read_number_mat.conservativeResize(row_counter,read_number_mat.cols());
    return true;
}

//读取物体3D框
void ReadALLOjbecttxt(const std::string pred_3d_obj_txt,int start_frame,int end_frame)
{
    //int total_img_ind = 13252;
    all_offline_object_cubes.reserve(2000);
    bool set_all_obj_probto_one = false;
    //yt 99 13252 zjg1 2 4860 zjg2 3 6250 yq 3 4300
    for (int img_couter = start_frame; img_couter <end_frame; img_couter=img_couter+1)
    {
        char frame_index_c[256];
        sprintf(frame_index_c, "%06d", img_couter);
        std::string pred_frame_obj_txts;

        pred_frame_obj_txts = pred_3d_obj_txt + frame_index_c;

        //3d cuboid txts:  each row:  [cuboid_center(3), yaw, cuboid_scale(3), [x1 y1 w h]], oject_id, object_type_id, prob
        int data_width = 9;
        Eigen::MatrixXd pred_frame_objects(5,data_width);
        if (read_pose_truth_txt(pred_frame_obj_txts, pred_frame_objects))
        {
            if (set_all_obj_probto_one)
            {
                for (int ii = 0; ii < pred_frame_objects.rows(); ii++)
                    pred_frame_objects(ii, data_width -1) = 1;
            }
            
            all_offline_object_cubes.push_back(pred_frame_objects);

            if(pred_frame_objects.rows()>0)
            {
                for (int ii = 0; ii < pred_frame_objects.rows();ii++)
                    if (pred_frame_objects(ii,data_width-1)<-0.1)
                        ROS_ERROR_STREAM("Read offline Bad object prob  " << pred_frame_objects(ii, data_width - 1) << "   frame  " << img_couter << "  row  " << ii);
            }
        }
        else
        {
            ROS_WARN_STREAM("Totally read object txt num  " << img_couter);
            break;
        }
    }
}

template bool read_pose_truth_txt(const std::string, MatrixXd &);
template bool read_pose_truth_txt2(const std::string, MatrixXd &,int,int,int);
template <class T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> homo_to_real_coord(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_homo_in)
{
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> pts_out(pts_homo_in.rows() - 1, pts_homo_in.cols());
    for (int i = 0; i < pts_homo_in.rows() - 1; i++)
        pts_out.row(i) = pts_homo_in.row(i).array() / pts_homo_in.bottomRows(1).array(); //replicate needs actual number, cannot be M or N

    return pts_out;
}
template MatrixXd homo_to_real_coord<double>(const MatrixXd &);
template MatrixXf homo_to_real_coord<float>(const MatrixXf &);

template <class T>
void homo_to_real_coord(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_homo_in, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_out)
{
    pts_out.resize(pts_homo_in.rows() - 1, pts_homo_in.cols());
    for (int i = 0; i < pts_homo_in.rows() - 1; i++)
        pts_out.row(i) = pts_homo_in.row(i).array() / pts_homo_in.bottomRows(1).array(); //replicate needs actual number, cannot be M or N
}
template void homo_to_real_coord<double>(const MatrixXd &, MatrixXd &);
template void homo_to_real_coord<float>(const MatrixXf &, MatrixXf &);

template <class T> // though vector can be casted into matrix, to make output clear to be vector, it is better to define a new function.
Eigen::Matrix<T, Eigen::Dynamic, 1> homo_to_real_coord_vec(const Eigen::Matrix<T, Eigen::Dynamic, 1> &pts_homo_in)
{
    Eigen::Matrix<T, Eigen::Dynamic, 1> pt_out;
    if (pts_homo_in.rows() == 4)
        pt_out = pts_homo_in.head(3) / pts_homo_in(3);
    else if (pts_homo_in.rows() == 3)
        pt_out = pts_homo_in.head(2) / pts_homo_in(2);

    return pt_out;
}
template VectorXd homo_to_real_coord_vec<double>(const VectorXd &);
template VectorXf homo_to_real_coord_vec<float>(const VectorXf &);

template <class T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> real_to_homo_coord(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_in)
{
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> pts_homo_out;
    int raw_rows = pts_in.rows();
    int raw_cols = pts_in.cols();

    pts_homo_out.resize(raw_rows + 1, raw_cols);
    pts_homo_out << pts_in,
        Matrix<T, 1, Dynamic>::Ones(raw_cols);
    return pts_homo_out;
}
template MatrixXd real_to_homo_coord<double>(const MatrixXd &);
template MatrixXf real_to_homo_coord<float>(const MatrixXf &);

template <class T>
void real_to_homo_coord(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_in, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_homo_out)
{
    int raw_rows = pts_in.rows();
    int raw_cols = pts_in.cols();

    pts_homo_out.resize(raw_rows + 1, raw_cols);
    pts_homo_out << pts_in,
        Matrix<T, 1, Dynamic>::Ones(raw_cols);
}
template void real_to_homo_coord<double>(const MatrixXd &, MatrixXd &);
template void real_to_homo_coord<float>(const MatrixXf &, MatrixXf &);

template <class T> // though vector can be casted into matrix, to make output clear to be vector, it is better to define a new function.
Eigen::Matrix<T, Eigen::Dynamic, 1> real_to_homo_coord_vec(const Eigen::Matrix<T, Eigen::Dynamic, 1> &pts_in)
{
    Eigen::Matrix<T, Eigen::Dynamic, 1> pts_homo_out;
    int raw_rows = pts_in.rows();
    ;

    pts_homo_out.resize(raw_rows + 1);
    pts_homo_out << pts_in,
        1;
    return pts_homo_out;
}
template VectorXd real_to_homo_coord_vec<double>(const VectorXd &);
template VectorXf real_to_homo_coord_vec<float>(const VectorXf &);

template <class T>
Eigen::Quaternion<T> zyx_euler_to_quat(const T &roll, const T &pitch, const T &yaw)
{
    T sy = sin(yaw * 0.5);
    T cy = cos(yaw * 0.5);
    T sp = sin(pitch * 0.5);
    T cp = cos(pitch * 0.5);
    T sr = sin(roll * 0.5);
    T cr = cos(roll * 0.5);
    T w = cr * cp * cy + sr * sp * sy;
    T x = sr * cp * cy - cr * sp * sy;
    T y = cr * sp * cy + sr * cp * sy;
    T z = cr * cp * sy - sr * sp * cy;
    return Eigen::Quaternion<T>(w, x, y, z);
}
template Eigen::Quaterniond zyx_euler_to_quat<double>(const double &, const double &, const double &);
template Eigen::Quaternionf zyx_euler_to_quat<float>(const float &, const float &, const float &);

template <class T>
void quat_to_euler_zyx(const Eigen::Quaternion<T> &q, T &roll, T &pitch, T &yaw)
{
    T qw = q.w();
    T qx = q.x();
    T qy = q.y();
    T qz = q.z();

    roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
    pitch = asin(2 * (qw * qy - qz * qx));
    yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
}
template void quat_to_euler_zyx<double>(const Eigen::Quaterniond &, double &, double &, double &);
template void quat_to_euler_zyx<float>(const Eigen::Quaternionf &, float &, float &, float &);

template <class T>
void rot_to_euler_zyx(const Eigen::Matrix<T, 3, 3> &R, T &roll, T &pitch, T &yaw)
{
    pitch = asin(-R(2, 0));

    if (abs(pitch - M_PI / 2) < 1.0e-3)
    {
        roll = 0.0;
        yaw = atan2(R(1, 2) - R(0, 1), R(0, 2) + R(1, 1)) + roll;
    }
    else if (abs(pitch + M_PI / 2) < 1.0e-3)
    {
        roll = 0.0;
        yaw = atan2(R(1, 2) - R(0, 1), R(0, 2) + R(1, 1)) - roll;
    }
    else
    {
        roll = atan2(R(2, 1), R(2, 2));
        yaw = atan2(R(1, 0), R(0, 0));
    }
}
template void rot_to_euler_zyx<double>(const Matrix3d &, double &, double &, double &);
template void rot_to_euler_zyx<float>(const Matrix3f &, float &, float &, float &);

template <class T>
Eigen::Matrix<T, 3, 3> euler_zyx_to_rot(const T &roll, const T &pitch, const T &yaw)
{
    T cp = cos(pitch);
    T sp = sin(pitch);
    T sr = sin(roll);
    T cr = cos(roll);
    T sy = sin(yaw);
    T cy = cos(yaw);

    Eigen::Matrix<T, 3, 3> R;
    R << cp * cy, (sr * sp * cy) - (cr * sy), (cr * sp * cy) + (sr * sy),
        cp * sy, (sr * sp * sy) + (cr * cy), (cr * sp * sy) - (sr * cy),
        -sp, sr * cp, cr * cp;
    return R;
}
template Matrix3d euler_zyx_to_rot<double>(const double &, const double &, const double &);
template Matrix3f euler_zyx_to_rot<float>(const float &, const float &, const float &);
