// pose_graph_optimizer.cpp

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_dogleg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

#include <vector>
#include <mutex>
std::string localization_folder="";
int edge_id=0;
int count=0;
class PoseGraphOptimizer {
public:
    PoseGraphOptimizer() {
        ros::NodeHandle nh;
        nh.param<std::string> ("/localization_folder", localization_folder, "/home/zjl/Desktop/parking/avp_data/");
        odom_sub_ = nh.subscribe("/rawPose", 1, &PoseGraphOptimizer::odomCallback, this);
        optimized_odom_pub_ = nh.advertise<nav_msgs::Path>("/optimized_odom", 1);

        // 初始化G2O
        initOptimizer();
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        std::unique_lock<std::mutex> lock(mutex_);
        ros::Time time_stamp = msg->header.stamp;
        std::cout<<"time count: "<<time_stamp<<std::endl;
        const double eps=1e-6;
        double time_s = time_stamp.toSec();
        timeids.push_back(time_s);
        long double time_a = 1648095563.499795000; //1648094417.957613;1648094414.656155
        if(abs(time_s-time_a)<eps){
            newTransWorldCurrent=true;
            //1648095563.499795000 156.716 43.807 0 -0.00787839 -0.0387936 -0.9959 0.0813411
            //55.804052738929556 -40.5243279689733 -0.03108353006049906 0.019579210077955902 0.033882603000608115 0.8231349728401314 0.5664957548244317
            //48.78984679629882 -47.35849128703777 1.6831681533944405 0.01604407185379421 0.004865138899947132 0.8028329072323929 0.5959683223516319
            transWorldCurrent.pose.orientation.w=0.0813411;
            transWorldCurrent.pose.orientation.x=-0.00787839;
            transWorldCurrent.pose.orientation.y=-0.0387936;
            transWorldCurrent.pose.orientation.z=-0.9959;
            transWorldCurrent.pose.position.x=156;
            transWorldCurrent.pose.position.y=43;
            transWorldCurrent.pose.position.z=0;
        }
        if(time_s<=1648095563.499795000){
            // 添加位姿到位姿图
            addPoseToGraph(msg);
            if(pose_ids_.size()>1){
                addEdgeBetweenPoses();
            }

            // 如果有新的真实位姿（transWorldCurrent），添加约束并优化
            if (newTransWorldCurrent) {
                addConstraintAndOptimize(transWorldCurrent);
                newTransWorldCurrent = false;
                optimizer_.initializeOptimization();
                std::cout<<"opti now"<<std::endl;
                optimizer_.optimize(30); // 迭代次数可以根据需求调整  
                std::cout<<"opti finish"<<std::endl;
                publishOptimizedPose();      
            }
            // 优化
            // optimizer_.initializeOptimization();
            // optimizer_.optimize(10); // 迭代次数可以根据需求调整
            // 发布优化后的位姿   
            publishOptimizedPose();  
        }
    }

    void initOptimizer() {
        // TODO: 初始化G2O优化器
        //使用线性方程求解器linearSolver
        g2o::BlockSolverX::LinearSolverType* linearSolver;
        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
        //使用稀疏矩阵块求解器solver_ptr
        g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
        //使用梯度下降法求解
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        //设置求解器
        optimizer_.setAlgorithm(solver);
        //是否打开调试输出
        optimizer_.setVerbose(true);
    }

    void addPoseToGraph(const nav_msgs::Odometry::ConstPtr &msg) {
        // TODO: 将位姿添加到G2O图中
        g2o::VertexSE3Expmap *vertex = new g2o::VertexSE3Expmap();
        int pose_id = optimizer_.vertices().size();
        vertex->setId(pose_id);
        pose_ids_.push_back(pose_id);

        Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                            msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z);
        Eigen::Vector3d t(msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
        g2o::SE3Quat se3(q, t);
        vertex->setEstimate(se3.inverse());

        if (pose_id == 0) {
            vertex->setFixed(true);
        }
        optimizer_.addVertex(vertex);
    }


    void addEdgeBetweenPoses() {
        int pose_id1 = pose_ids_.size() - 2;
        int pose_id2 = pose_ids_.size() - 1;

        g2o::EdgeSE3Expmap *edge = new g2o::EdgeSE3Expmap();
        edge->setId(edge_id);
        edge_id++;
        edge->setVertex(0, optimizer_.vertex(pose_id1));
        edge->setVertex(1, optimizer_.vertex(pose_id2));
        g2o::VertexSE3Expmap *v1 = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer_.vertex(pose_id1));
        g2o::VertexSE3Expmap *v2 = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer_.vertex(pose_id2));

        g2o::SE3Quat relative_pose = v2->estimate()*v1->estimate().inverse(); //v1->estimate().inverse()*v2->estimate();

        edge->setMeasurement(relative_pose);

        Eigen::Matrix<double, 6, 6> information_matrix = Eigen::Matrix<double, 6, 6>::Identity();
        // 根据需求设置信息矩阵的值
        edge->setInformation(information_matrix);
        optimizer_.addEdge(edge);
    }

    void addConstraintAndOptimize(const geometry_msgs::PoseStamped &transWorldCurrent) {
        Eigen::Quaterniond q(transWorldCurrent.pose.orientation.w,
                            transWorldCurrent.pose.orientation.x,
                            transWorldCurrent.pose.orientation.y,
                            transWorldCurrent.pose.orientation.z);
        Eigen::Vector3d t(transWorldCurrent.pose.position.x,
                        transWorldCurrent.pose.position.y,
                        transWorldCurrent.pose.position.z);
        g2o::SE3Quat se3(q, t);
        g2o::VertexSE3Expmap *known_vertex = new g2o::VertexSE3Expmap();
        int known_pose_id = pose_ids_.size();
        known_vertex->setId(known_pose_id);
        known_vertex->setEstimate(se3.inverse());
        known_vertex->setFixed(true);
        optimizer_.addVertex(known_vertex);        

        int pose_id1 = pose_ids_.size() - 1;

        g2o::EdgeSE3Expmap *edge = new g2o::EdgeSE3Expmap();
        edge->setId(edge_id);
        edge_id++;
        edge->setVertex(0, optimizer_.vertex(pose_id1));
        edge->setVertex(1, optimizer_.vertex(known_pose_id));

        g2o::VertexSE3Expmap *v1 = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer_.vertex(pose_id1));
        Eigen::Quaterniond unit_quat(1,0,0,0);
        Eigen::Vector3d zero_vec(0,0,0);
        g2o::SE3Quat relative_pose(unit_quat,zero_vec);  //v1->estimate().inverse()*se3
        edge->setMeasurement(relative_pose);

        Eigen::Matrix<double, 6, 6> information_matrix = Eigen::Matrix<double, 6, 6>::Identity();
        // 根据需求设置信息矩阵的值
        information_matrix*=10;
        edge->setInformation(information_matrix);

        optimizer_.addEdge(edge);
    }

    void publishOptimizedPose() {
        // TODO: 发布优化后的位姿
        nav_msgs::Path path_msg;
        path_msg.header.stamp=ros::Time::now();
        path_msg.header.frame_id="world";
        std::string pose_out = localization_folder+"opti_result.txt";
        std::ofstream out(pose_out.c_str(),std::ios::out);
        for(const auto &vertex:optimizer_.vertices()){
            g2o::VertexSE3Expmap *optimized_vertex = dynamic_cast<g2o::VertexSE3Expmap *>(vertex.second);
            double stamp=0;
            if(vertex.first<timeids.size()){
                stamp = timeids[vertex.first];
            }
            g2o::SE3Quat se3 = optimized_vertex->estimate().inverse();
            Eigen::Vector3d t = se3.translation();
            Eigen::Quaterniond q = se3.rotation();
            geometry_msgs::PoseStamped optimized_odom;
            optimized_odom.header.stamp = ros::Time().fromSec(stamp);
            optimized_odom.header.frame_id = "world";

            optimized_odom.pose.position.x = t.x();
            optimized_odom.pose.position.y = t.y();
            optimized_odom.pose.position.z = t.z();
            optimized_odom.pose.orientation.w = q.w();
            optimized_odom.pose.orientation.x = q.x();
            optimized_odom.pose.orientation.y = q.y();
            optimized_odom.pose.orientation.z = q.z();
            path_msg.poses.push_back(optimized_odom);
            out<<std::to_string(stamp)<<" "<<t.x()<<" "<<t.y()<<" "<<0<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<std::endl;             
        }
        optimized_odom_pub_.publish(path_msg);
        
    }

private:
    ros::Subscriber odom_sub_;
    ros::Publisher optimized_odom_pub_;

    g2o::SparseOptimizer optimizer_;
    std::vector<int> pose_ids_;
    std::vector<double> timeids;
    std::mutex mutex_;
    geometry_msgs::PoseStamped transWorldCurrent;
    bool newTransWorldCurrent = false;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_graph_optimizer");

    PoseGraphOptimizer optimizer;

    ros::spin();
    return 0;
}
