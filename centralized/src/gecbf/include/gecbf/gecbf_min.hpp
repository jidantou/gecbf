#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <unordered_set>
#include <fstream>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include "sphere_detector/sphere_detector.hpp"

namespace GECBF_QP_MIN
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
    
    class Gecbf
    {
    private:
        std::vector<PartStateType> drones_states_;
        std::vector<std::unordered_set<int>> neighbors_;
        std::vector<sphere_detector::SphereDetector> local_maps_;
        std::vector<bool> haveOdom_;

        std::ofstream log_file_;
        bool log_opened_;

        // system parameters
        int drone_num_;
        double sensing_horizon_;
        double obstacle_size_;
        double drone_size_;

        // gecbf's parameters
        int M_DRONES_;
        int BETA_;
        double G_;      // gravity acceleration
        double MASS_;

        std::vector<Eigen::Vector2d> Kbs_;
        double Kb_alpha_, Kb_delta_;

        std::vector<Eigen::Vector3d> curCtrls_;

        void compute_xi_derivatives(const double s, double& xi, double& xi_prime, double& xi_double_prime) const;
        void updateNeighbors(const int drone_id);
        void updateKb(const int drone_id);
        void compute_dynamic_f(const int drone_id, Eigen::Matrix<double, 13, 1>& f) const;
        void compute_dynamic_g(const int drone_id, Eigen::Matrix<double, 13, 3>& g) const;
        
    public:
        Gecbf(ros::NodeHandle &nh, int drone_num, double sensing_horizon, double obstacle_distances, double drone_distance);
        ~Gecbf();

        // ============================================================================
        // 障碍物避碰函数 h_j^o 及其梯度
        // ============================================================================
        // 输入:
        //   o_j : 障碍物位置
        //   p_i : 无人机 i 位置
        //   v_i : 无人机 i 速度
        // 输出:
        //   eta : [h, h_dot]
        //   dh_dot_dp: 对 p_i 的梯度
        //   dh_dot_dv: 对 v_i 的梯度
        void compute_h_j_obstacle(const Eigen::Vector3d& o_j,
                                const Eigen::Vector3d& p_i,
                                const Eigen::Vector3d& v_i,
                                Eigen::Vector2d& eta,
                                Eigen::Vector3d& dh_dot_dp,
                                Eigen::Vector3d& dh_dot_dv) const;

        // ============================================================================
        // 无人机间避碰函数 h_j^d 及其梯度
        // ============================================================================
        // 输入:
        //   p_j, v_j : 另一无人机 j 的位置与速度
        //   p_i, v_i : 本无人机 i 的位置与速度
        // 输出:
        //   eta : [h, h_dot]
        //   dh_dot_dp_i, dh_dot_dv_i : 对 i 的梯度
        //   dh_dot_dp_j, dh_dot_dv_j : 对 j 的梯度
        void compute_h_j_drone(const Eigen::Vector3d& p_j,
                            const Eigen::Vector3d& v_j,
                            const Eigen::Vector3d& p_i,
                            const Eigen::Vector3d& v_i,
                            Eigen::Vector2d& eta,
                            Eigen::Vector3d& dh_dot_dp_i,
                            Eigen::Vector3d& dh_dot_dv_i,
                            Eigen::Vector3d& dh_dot_dp_j,
                            Eigen::Vector3d& dh_dot_dv_j) const;

        void cal_h(const int drone_id, Eigen::Vector2d& eta, std::vector<std::pair<int,PartStateType>>& gradents);

        void computeQPCoefficientsAl(Eigen::SparseMatrix<double>& A, Eigen::VectorXd& l);

        void setCtrls(const std::vector<Eigen::Vector3d>& ref_ctrls);
        void setCtrls(const Eigen::VectorXd& solution);

        void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);

        void closeLogFile();

        // DEBUG
        int debug_count;

    };
    
} // namespace gecbf_qp
