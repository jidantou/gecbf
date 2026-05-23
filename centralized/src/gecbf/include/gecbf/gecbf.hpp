#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <fstream>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include "sphere_detector/sphere_detector.hpp"

namespace GECBF_QP
{
    struct PartStateType
    {
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
        Eigen::Vector4d qua;
        Eigen::Vector3d ome;
        
        PartStateType(const Eigen::Vector3d& p = Eigen::Vector3d::Zero(), const Eigen::Vector3d& v = Eigen::Vector3d::Zero(), const Eigen::Vector4d& q = Eigen::Vector4d::Zero(), const Eigen::Vector3d& o = Eigen::Vector3d::Zero()) : pos(p), vel(v), qua(q), ome(o) {}

        Eigen::RowVector3d TransposeDot(const Eigen::Matrix<double, 13, 3>& other) const;
        double TransposeDot(const Eigen::Matrix<double, 13, 1>& other) const;

        void setZero() {pos.setZero(); vel.setZero(); qua.setZero(); ome.setZero();}
    };

    typedef Eigen::Matrix<double, 13, 1> State_Type;

    enum ConstraintTypeEnum {OBSTACLE, DRONE};

    struct ConstraintInfoType
    {
        ConstraintTypeEnum type;
        int current_drone_id;           // 当前约束所属的无人机id
        Eigen::Vector3d o_j;  // 如果 type == OBSTACLE，则表示障碍物位置；如果 type == DRONE，则无意义
        int other_drone_id;  // 如果 type == DRONE，则表示另一个无人机的 ID；如果 type == OBSTACLE，则无意义
        Eigen::Vector2d Kb;

        ConstraintInfoType() : type(OBSTACLE), current_drone_id(-1), o_j(Eigen::Vector3d::Zero()), other_drone_id(-1), Kb(Eigen::Vector2d::Zero()) {}

        // Constraint pair between drone and obstacle
        ConstraintInfoType(const int drone_id, const Eigen::Vector3d& o, const Eigen::Vector2d& Kb_ = Eigen::Vector2d::Zero()) : type(OBSTACLE), current_drone_id(drone_id), o_j(o), other_drone_id(-1), Kb(Kb_) {}

        // Constraint pair between two drones
        ConstraintInfoType(const int drone_id, const int other_drone_id_, const Eigen::Vector2d& Kb_ = Eigen::Vector2d::Zero()) : type(DRONE), current_drone_id(drone_id), o_j(Eigen::Vector3d::Zero()), other_drone_id(other_drone_id_), Kb(Kb_) {}
    };
    
    class Gecbf
    {
    private:
        std::vector<PartStateType> drones_states_;
        std::vector<std::unordered_map<int, uint64_t>> neighbors_;  // key 是邻居无人机的 ID，value 是对应约束的 key，用于删除约束时调用
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
        double Kb_alpha_, Kb_delta_o_, Kb_delta_d_;

        // 约束信息，key 是约束对应的唯一id，value 是一个包含约束类型、相关参数等信息的结构体
        std::unordered_map<uint64_t, ConstraintInfoType> current_constraints_;
        uint64_t next_constraint_id_ = 0;  // 用于生成唯一的约束id

        std::vector<Eigen::Vector3d> curCtrls_;

        // ============================================================================
        // 障碍物避碰函数 h_j^o 及其梯度
        // ============================================================================
        // 输入:
        //   o_j : 障碍物位置
        //   p_i : 无人机 i 位置
        //   v_i : 无人机 i 速度
        // 输出:
        //   eta : [h, h_dot]
        //   dh_dot: 对 i 的梯度
        void compute_h_j_obstacle(const Eigen::Vector3d& o_j,
                                const PartStateType& state_i,
                                Eigen::Vector2d& eta,
                                PartStateType& dh_dot) const;

        // ============================================================================
        // 无人机间避碰函数 h_j^d 及其梯度
        // ============================================================================
        // 输入:
        //   state_i : 本无人机 i 的状态
        //   state_j : 另一无人机 j 的状态
        // 输出:
        //   eta : [h, h_dot]
        //   dh_dot_i : 对 i 的梯度
        //   dh_dot_j : 对 j 的梯度
        void compute_h_j_drone(const PartStateType& state_i,
                            const PartStateType& state_j,
                            Eigen::Vector2d& eta,
                            PartStateType& dh_dot_i,
                            PartStateType& dh_dot_j) const;

        // void cal_h(const int drone_id, Eigen::Vector2d& eta, std::vector<std::pair<int,PartStateType>>& gradents);
        void compute_xi_derivatives(const double s, double& xi, double& xi_prime, double& xi_double_prime) const;
        void updateNeighbors(const int drone_id);
        void computeKb(ConstraintInfoType& conInfo);
        void compute_dynamic_f(const int drone_id, Eigen::Matrix<double, 13, 1>& f) const;
        void compute_dynamic_g(const int drone_id, Eigen::Matrix<double, 13, 3>& g) const;
        
    public:
        Gecbf(ros::NodeHandle &nh, int drone_num, double sensing_horizon, double obstacle_distances, double drone_distance);
        ~Gecbf();

        // return the number of constraints
        int computeQPCoefficientsAl(Eigen::SparseMatrix<double>& A, Eigen::VectorXd& l);

        void setCtrls(const std::vector<Eigen::Vector3d>& ref_ctrls);
        void setCtrls(const Eigen::VectorXd& solution);

        void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);

        // 当检测到新的障碍物时，调用此函数更新约束信息
        // 返回值为新增约束的key，用于删除时调用
        uint64_t addConstraintsCallback(const int drone_id, const Eigen::Vector3d& obstacle_pos);

        // 当检测到新的邻居无人机时，调用此函数更新约束信息
        // 返回值为新增约束的key，用于删除时调用
        uint64_t addConstraintsCallback(const int drone_id, const int other_drone_id);

        void removeConstraintsCallback(const uint64_t constraint_key);

        void closeLogFile();

        void updateAllKbs();

        // DEBUG
        int debug_count;
        std::vector<ConstraintInfoType> debug_cons_;

    };
    
} // namespace gecbf_qp
