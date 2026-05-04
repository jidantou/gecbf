#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <memory>
#include <fstream>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
// #include "plan_env/grid_map.h"
#include "sphere_detector/sphere_detector.hpp"

namespace GCBF_QP_MIN
{
    struct PartStateType
    {
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
        Eigen::Vector4d qua;
        Eigen::Vector3d ome;
        PartStateType(const Eigen::Vector3d& p = Eigen::Vector3d::Zero(), const Eigen::Vector3d& v = Eigen::Vector3d::Zero(), const Eigen::Vector4d& q = Eigen::Vector4d::Zero(), const Eigen::Vector3d& o = Eigen::Vector3d::Zero()) {}
        Eigen::Vector3d TransposeDot(const Eigen::Matrix<double, 13, 3>& other) const;
        double TransposeDot(const Eigen::Matrix<double, 13, 1>& other) const;
    };

    typedef Eigen::Matrix<double, 13, 1> State_Type;
    
    class Gcbf
    {
    private:
        typedef struct 
        {
            double value;
            double gradient;
        } RTypedd;

        std::unique_ptr<PartStateType[]> _drones_state_ptr;
        std::vector<std::vector<int>> _neighbors;
        std::unique_ptr<sphere_detector::SphereDetector[]> _local_maps;
        std::vector<bool> _haveOdom;

        std::ofstream log_file_;
        bool log_opened_;

        RTypedd f(double x, double R, int beta) const;
        RTypedd sigma(double x, double R, double gamma) const;
        void get_neighbors();
        void compute_dynamic_f(const int drone_id, Eigen::Matrix<double, 13, 1>& f) const;
        void compute_dynamic_g(const int drone_id, Eigen::Matrix<double, 13, 3>& g) const;
        
    public:
        Gcbf(ros::NodeHandle &nh, int drone_num, double sensing_horizon, double obstacle_distances, double drone_distance);
        ~Gcbf();

        // ============================================================================
        // 障碍物避碰函数 h_j^o 及其梯度
        // ============================================================================
        // 输入:
        //   o_j : 障碍物位置 (3D)
        //   p_i : 无人机 i 位置
        //   v_i : 无人机 i 速度
        //   r   : 无人机半径
        //   A   : 最大减速度
        // 输出:
        //   h    : 函数值
        //   dh_dp: 对 p_i 的梯度
        //   dh_dv: 对 v_i 的梯度
        void compute_h_j_obstacle(const Eigen::Vector3d& o_j, const Eigen::Vector3d& p_i, const Eigen::Vector3d& v_i, double& h, Eigen::Vector3d& dh_dp, Eigen::Vector3d& dh_dv) const;

        // ============================================================================
        // 无人机间避碰函数 h_j^d 及其梯度
        // ============================================================================
        // 输入:
        //   p_j, v_j : 另一无人机 j 的位置与速度
        //   p_i, v_i : 本无人机 i 的位置与速度
        //   r, A     : 无人机半径与最大减速度
        //   R, beta, gamma : 函数参数
        // 输出:
        //   h        : 函数值
        //   dh_dp_i, dh_dv_i : 对 i 的梯度
        //   dh_dp_j, dh_dv_j : 对 j 的梯度
        void compute_h_j_drone(const Eigen::Vector3d& p_j, const Eigen::Vector3d& v_j, const Eigen::Vector3d& p_i, const Eigen::Vector3d& v_i, double& h, Eigen::Vector3d& dh_dp_i, Eigen::Vector3d& dh_dv_i, Eigen::Vector3d& dh_dp_j, Eigen::Vector3d& dh_dv_j) const;

        void cal_h(const int drone_id, double& h_value, std::vector<std::pair<int,PartStateType>>& gradents);

        void computeQPCoefficientsAl(Eigen::SparseMatrix<double>& A, Eigen::VectorXd& l);

        void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);

        void closeLogFile();

        // system parameters
        int _DRONE_NUM;
        double _sensing_horizon;
        double _obstacle_size;
        double _drone_size;
        double _alpha0;

        // gcbf h's parameters
        int _M_DRONES;
        double ACC_MAX;
        int BETA;
        double GAMMA;

        // DEBUG
        int debug_count;

    };
    
} // namespace gcbf_qp
