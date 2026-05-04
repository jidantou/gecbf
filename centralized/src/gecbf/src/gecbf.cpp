#include "gecbf/gecbf.hpp"
#include <limits>
#include <cmath>
#include <iomanip>

// DEBUG
#include <sstream>

using namespace GECBF_QP;

Gecbf::Gecbf(ros::NodeHandle &nh, int drone_num = 1, double sensing_horizon = 5.0, double obstacle_distances = 0.05, double drone_distance = 0.1) {
    drone_num_ = drone_num;
    sensing_horizon_ = sensing_horizon;
    obstacle_size_ = obstacle_distances;
    drone_size_ = drone_distance;

    nh.param("gecbf/M_DRONES", M_DRONES_, 5);
    nh.param("gecbf/BETA", BETA_, 50);

    G_ = 9.81;
    MASS_ = 0.98;

    nh.param("gecbf/Kb_alpha", Kb_alpha_, 1.0);
    nh.param("gecbf/Kb_delta_o", Kb_delta_o_, 0.1);
    nh.param("gecbf/Kb_delta_d", Kb_delta_d_, 0.1);

    drones_states_.resize(drone_num);
    neighbors_.resize(drone_num);
    haveOdom_.resize(drone_num, false);

    local_maps_.reserve(drone_num);
    curCtrls_.reserve(drone_num);

    for (int i = 0; i < drone_num; i++) {
        local_maps_.emplace_back(nh, i);
        curCtrls_.emplace_back(0.0, 0.0, MASS_ * G_);
        
        neighbors_[i].emplace(i, std::numeric_limits<uint64_t>::max());
    }

    for (int i = 0; i < drone_num_; i++) {
        local_maps_[i].regObstaclesAddCallbacks([this](const int drone_id, const Eigen::Vector3d& obstacle_pos) {
            return this->addConstraintsCallback(drone_id, obstacle_pos);
        });
        local_maps_[i].regObstaclesRemoveCallbacks([this](const uint64_t constraint_key) {
            this->removeConstraintsCallback(constraint_key);
        });
    }

    log_opened_ = false;

}

Gecbf::~Gecbf() {}

Eigen::RowVector3d PartStateType::TransposeDot(const Eigen::Matrix<double, 13, 3>& other) const {
    // 计算 temp^T * other，其中 temp = [pos; vel; qua; ome]
    // 将 other 按行分块：pos 对应 0-2 行，vel 对应 3-5 行，qua 对应 6-9 行，ome 对应 10-12 行
    Eigen::RowVector3d row_result;
    row_result.noalias() = pos.transpose() * other.topRows<3>()         // 1x3 3x3
                         + vel.transpose() * other.middleRows<3>(3)     // 1x3 3x3
                         + qua.transpose() * other.middleRows<4>(6)     // 1x4 4x3
                         + ome.transpose() * other.bottomRows<3>();     // 1x3 3x3
    return row_result;
}

double PartStateType::TransposeDot(const Eigen::Matrix<double, 13, 1>& other) const {
    return pos.dot(other.head<3>()) +
           vel.dot(other.segment<3>(3)) +
           qua.dot(other.segment<4>(6)) +
           ome.dot(other.tail<3>());
}

void Gecbf::compute_xi_derivatives(const double s, double& xi, double& xi_prime, double& xi_double_prime) const {
    const double R2 = sensing_horizon_ * sensing_horizon_;
    const double u = s / R2;                     // u ∈ [0,1]
    const double u_pow_beta_m2 = std::pow(u, BETA_ - 2);
    const double u_pow_beta_m1 = u_pow_beta_m2 * u;
    const double u_pow_beta     = u_pow_beta_m1 * u;
    const double u_pow_beta_p1  = u_pow_beta * u;

    // ξ(s) = s + (R2−s)[(β+1)(s/R2​)^β−β(s/R2​)^(β+1)]
    xi = s + (R2 - s) * ( (BETA_ + 1) * u_pow_beta - BETA_ * u_pow_beta_p1 );

    // ξ'(s) = 1 + β(β+1) u^{β-1} - (β+1)(2β+1) u^β + β(β+2) u^{β+1}
    const int beta_plus_1 = BETA_ + 1;
    const int two_beta_plus_1 = 2 * BETA_ + 1;

    xi_prime = 1.0
               + BETA_ * beta_plus_1 * u_pow_beta_m1
               - beta_plus_1 * two_beta_plus_1 * u_pow_beta
               + BETA_ * (BETA_ + 2) * u_pow_beta_p1;

    // ξ''(s) = (β(β+1)/R^2) * [ (β-1) u^{β-2} - (2β+1) u^{β-1} + (β+2) u^β ]
    const double factor = BETA_ * beta_plus_1 / R2;
    xi_double_prime = factor * ( (BETA_ - 1) * u_pow_beta_m2
                                 - two_beta_plus_1 * u_pow_beta_m1
                                 + (BETA_ + 2) * u_pow_beta );
}

void Gecbf::compute_h_j_obstacle(const Eigen::Vector3d& o_j,
                                const PartStateType& state_i,
                                Eigen::Vector2d& eta,
                                PartStateType& dh_dot) const{
    const Eigen::Vector3d diff = o_j - state_i.pos;
    const double dist_sq = diff.squaredNorm();

    // h = ||o_j - p_i||^2 - (r_o + r_d)^2
    const double sum_radii = obstacle_size_ + drone_size_;
    const double h = dist_sq - sum_radii * sum_radii;

    // h_dot = -2 (o_j - p_i) · v_i
    const double h_dot = -2.0 * diff.dot(state_i.vel);

    // eta = [h, h_dot]
    eta(0) = h;
    eta(1) = h_dot;

    // Gradient of h_dot w.r.t p_i:  2 * v_i
    // dh_dot_dp = 2.0 * v_i;
    dh_dot.pos = 2.0 * state_i.vel;

    // Gradient of h_dot w.r.t v_i: -2 * (o_j - p_i)
    // dh_dot_dv = -2.0 * diff;
    dh_dot.vel = -2.0 * diff;
}

void Gecbf::compute_h_j_drone(const PartStateType& state_i,
                            const PartStateType& state_j,
                            Eigen::Vector2d& eta,
                            PartStateType& dh_dot_i,
                            PartStateType& dh_dot_j) const {
    const Eigen::Vector3d B = state_j.pos - state_i.pos;          // 相对位置
    const Eigen::Vector3d C = state_j.vel - state_i.vel;          // 相对速度

    const double s = B.squaredNorm();             // s = ||p_j - p_i||^2
    const double s0 = 4.0 * drone_size_ * drone_size_;          // (2 r_d)^2

    // 计算 ξ(s) 和 ξ(s0)
    const double R_sq_ = sensing_horizon_ * sensing_horizon_;
    const double u0 = s0 / R_sq_;
    const double u0_pow_beta = std::pow(u0, BETA_);
    const double u0_pow_beta_p1 = u0_pow_beta * u0;

    // ξ(s0) = s0 + (R^2 - s0) [ (β+1) u0^β - β u0^{β+1} ]
    const double xi_s0 = s0 + (R_sq_ - s0) * ( (BETA_ + 1.0) * u0_pow_beta - BETA_ * u0_pow_beta_p1 );

    // ξ(s) 同理
    const double u = s / R_sq_;
    const double u_pow_beta = std::pow(u, BETA_);
    const double u_pow_beta_p1 = u_pow_beta * u;
    const double xi_s = s + (R_sq_ - s) * ( (BETA_ + 1.0) * u_pow_beta - BETA_ * u_pow_beta_p1 );

    const double h = xi_s - xi_s0;

    // 求 ξ'(s) 和 ξ''(s)
    double xi, xi_prime, xi_double_prime;
    compute_xi_derivatives(s, xi, xi_prime, xi_double_prime);

    // h_dot = 2 * ξ'(s) * (B · C)
    const double B_dot_C = B.dot(C);
    const double h_dot = 2.0 * xi_prime * B_dot_C;

    eta(0) = h;
    eta(1) = h_dot;

    // 计算梯度
    // dh_dot / dp_i = -4 ξ''(s) (B·C) B - 2 ξ'(s) C
    const double coeff1 = 4.0 * xi_double_prime * B_dot_C;
    const double coeff2 = 2.0 * xi_prime;
    // dh_dot_dp_i = -coeff1 * B - coeff2 * C;
    dh_dot_i.pos = -coeff1 * B - coeff2 * C;

    // dh_dot / dv_i = -2 ξ'(s) B
    // dh_dot_dv_i = -coeff2 * B;
    dh_dot_i.vel = -coeff2 * B;

    // dh_dot / dp_j =  4 ξ''(s) (B·C) B + 2 ξ'(s) C
    // dh_dot_dp_j = coeff1 * B + coeff2 * C;
    dh_dot_j.pos = coeff1 * B + coeff2 * C;

    // dh_dot / dv_j =  2 ξ'(s) B
    // dh_dot_dv_j = coeff2 * B;
    dh_dot_j.vel = coeff2 * B;
}

void Gecbf::updateNeighbors(const int drone_id) {
    auto& nb_i = neighbors_[drone_id];
    const auto& p_i = drones_states_[drone_id].pos;
    const double param = 0.8;
    // const double R2 = sensing_horizon_ * sensing_horizon_;
    const double R2 = (param*sensing_horizon_) * (param*sensing_horizon_);  // 实际执行时缩小感知半径以避免数值问题（h太小）

    for (int j = 0; j < drone_num_; j++) {
        if (j == drone_id)
            continue;
        auto it = nb_i.find(j);
        const auto& p_j = drones_states_[j].pos;
        if (it != nb_i.end()) {
            // 已经是邻居了，检查是否还满足条件
            if ((p_i - p_j).squaredNorm() > R2) {
                removeConstraintsCallback(it->second);
                // active_constraints_.erase(it->second);  // 删除对应的约束
                nb_i.erase(it);  // 从邻居列表中删除
                neighbors_[j].erase(drone_id);  // 互相删除
            }
        }
        else {
            // 不是邻居，检查是否满足成为邻居的条件
            if ((p_i - p_j).squaredNorm() <= R2) {
                uint64_t con_id = addConstraintsCallback(drone_id, j);
                // ConstraintInfoType new_constraint(drone_id, j);
                // computeKb(drone_id, new_constraint.Kb);
                // active_constraints_.emplace(next_constraint_id_, new_constraint);  // 添加新的约束

                nb_i.emplace(j, con_id);
                neighbors_[j].emplace(drone_id, con_id);
            }
        }
    }
    
}

void Gecbf::computeKb(ConstraintInfoType& conInfo) {
    double p1, p2, l1, l2, h_dotdot = 0.0;
    Eigen::Vector2d eta;

    if (conInfo.type == OBSTACLE) {
        const int i = conInfo.current_drone_id;
        Eigen::Matrix<double, 13, 1> f;
        Eigen::Matrix<double, 13, 3> g;
        compute_dynamic_f(i, f);
        compute_dynamic_g(i, g);

        PartStateType dh_dot;
        compute_h_j_obstacle(conInfo.o_j, drones_states_[i], eta, dh_dot);

        if (eta(0) < 0) {
            ROS_WARN("%d:  h_j^o < 0! drone_id: %d, h = %f", debug_count, i, eta(0));
        }

        State_Type x_dot = f + g * curCtrls_[i];

        h_dotdot = dh_dot.TransposeDot(x_dot);
    }
    else if (conInfo.type == DRONE) {
        const int i = conInfo.current_drone_id, j = conInfo.other_drone_id;
        Eigen::Matrix<double, 13, 1> f_i, f_j;
        Eigen::Matrix<double, 13, 3> g_i, g_j;
        compute_dynamic_f(i, f_i);
        compute_dynamic_g(i, g_i);
        compute_dynamic_f(j, f_j);
        compute_dynamic_g(j, g_j);

        PartStateType dh_dot_i, dh_dot_j;
        compute_h_j_drone(drones_states_[i], drones_states_[j], eta, dh_dot_i, dh_dot_j);

        if (eta(0) < 0) {
            ROS_WARN("%d:  h_j^d < 0! drone_id: %d, other_id: %d, h = %f", debug_count, i, j, eta(0));
        }

        State_Type x_dot_i = f_i + g_i * curCtrls_[i];
        State_Type x_dot_j = f_j + g_j * curCtrls_[j];

        h_dotdot = dh_dot_i.TransposeDot(x_dot_i) + dh_dot_j.TransposeDot(x_dot_j);
    }

    // if (eta(0) > 1000.0 || eta(0) < -1000.0) {
    //     ROS_WARN("%d:  Drone %d has very large h value: %e", debug_count, conInfo.current_drone_id, eta(0));
    // }
    // if (eta(1) > 1000.0 || eta(1) < -1000.0) {
    //     ROS_WARN("%d:  Drone %d has very large h_dot value: %e", debug_count, conInfo.current_drone_id, eta(1));
    // }

    l1 = - eta(1) / eta(0);
    if (l1 > 0)
        p1 = Kb_alpha_ * l1;
    else
        p1 = conInfo.type == OBSTACLE ? Kb_delta_o_ : Kb_delta_d_;
    
    l2 = - (h_dotdot + p1 * eta(1)) / (eta(1) + p1 * eta(0));
    if (l2 > 0)
        p2 = Kb_alpha_ * l2;
    else
        p2 = conInfo.type == OBSTACLE ? Kb_delta_o_ : Kb_delta_d_;

    if (l1 > 1000.0 || l1 < -1000.0) {
        ROS_WARN("%d:  Drone %d has very large l1 value: %e, eta: [%e,%e]", debug_count, conInfo.current_drone_id, l1, eta(0), eta(1));
    }
    if (l2 > 1000.0 || l2 < -1000.0) {
        ROS_WARN("%d:  Drone %d has very large l2 value: %e, eta: [%e,%e], fraction: [%e,%e]", debug_count, conInfo.current_drone_id, l2, eta(0), eta(1), (eta(1) + p1 * eta(0)), (h_dotdot + p1 * eta(1)));
    }

    conInfo.Kb(0) = p1 * p2;
    conInfo.Kb(1) = p1 + p2;
}

int Gecbf::computeQPCoefficientsAl(Eigen::SparseMatrix<double>& A, Eigen::VectorXd& l) {
    if (!log_opened_) {
        const char* home = std::getenv("HOME");
        if (!home)
        {
            ROS_ERROR("HOME environment variable not set");
            ros::shutdown();
        }
        std::string log_path = std::string(home) + "/Projects/gecbf_log/gecbf_h_values.csv";
        log_file_.open(log_path, std::ios::out);
        if (log_file_.is_open()) {
            ROS_INFO("GECBF h_value log file opened: %s", log_path.c_str());
            log_file_ << "timestamp,constraint_id,constraint_type,drone_id,other_id,h_value,h_dot_value\n";
            log_opened_ = true;
        } else {
            ROS_WARN("Failed to open log file: %s", log_path.c_str());
        }
    }

    // 获取当前时间（秒.纳秒），避免科学计数法造成精度显示不全
    const ros::Time now = ros::Time::now();

    std::vector<Eigen::Triplet<double>> triplets;
    int NUM_CONS = active_constraints_.size();

    if (NUM_CONS == 0) {
        // A.resize(1, drone_num_ * 3);
        // l.resize(1);

        // // 没有约束，添加一个虚拟约束保持问题可行
        // l(0) = -100.0;
        // triplets.emplace_back(0, 0, 0.0);
        // A.setFromTriplets(triplets.begin(), triplets.end());
        // return 1;

        return 0;
    }

    A.resize(NUM_CONS, drone_num_ * 3);
    l.resize(NUM_CONS);

    triplets.reserve(NUM_CONS * 6);

    int row_index = 0;
    for (auto &&con : active_constraints_) {
        const uint64_t constraint_id = con.first;
        const auto& conInfo = con.second;

        if (conInfo.type == OBSTACLE) {
            int i = conInfo.current_drone_id;
            Eigen::Matrix<double, 13, 1> f;
            Eigen::Matrix<double, 13, 3> g;
            compute_dynamic_f(i, f);
            compute_dynamic_g(i, g);

            PartStateType dh_dot;
            Eigen::Vector2d eta;
            compute_h_j_obstacle(conInfo.o_j, drones_states_[i], eta, dh_dot);

            if (eta(0) < 0) {
                ROS_WARN("%d:  h_j^o < 0! drone_id: %d, h = %f", debug_count, i, eta(0));
            }

            if (log_file_.is_open()) {
                log_file_ << now.sec << "." << std::setw(9) << std::setfill('0') << now.nsec << ","
                          << constraint_id << ","
                          << "obstacle" << ","
                          << i << ","
                          << -1 << ","
                          << eta(0) << ","
                          << eta(1) << "\n";
            }

            Eigen::RowVector3d grad_g = dh_dot.TransposeDot(g);
            for (int k = 0; k < 3; k++) {
                triplets.emplace_back(row_index, i * 3 + k, grad_g(k));
            }

            l(row_index) = - dh_dot.TransposeDot(f) - conInfo.Kb.dot(eta);
        }
        else if (conInfo.type == DRONE) {
            int i = conInfo.current_drone_id, j = conInfo.other_drone_id;
            Eigen::Matrix<double, 13, 1> f_i, f_j;
            Eigen::Matrix<double, 13, 3> g_i, g_j;
            compute_dynamic_f(i, f_i);
            compute_dynamic_g(i, g_i);
            compute_dynamic_f(j, f_j);
            compute_dynamic_g(j, g_j);

            PartStateType dh_dot_i, dh_dot_j;
            Eigen::Vector2d eta;
            compute_h_j_drone(drones_states_[i], drones_states_[j], eta, dh_dot_i, dh_dot_j);

            if (eta(0) < 0) {
                ROS_WARN("%d:  h_j^d < 0! drone_id: %d, other_id: %d, h = %f", debug_count, i, j, eta(0));
            }

            if (log_file_.is_open()) {
                log_file_ << now.sec << "." << std::setw(9) << std::setfill('0') << now.nsec << ","
                          << constraint_id << ","
                          << "drone" << ","
                          << i << ","
                          << j << ","
                          << eta(0) << ","
                          << eta(1) << "\n";
            }

            Eigen::RowVector3d grad_g_i = dh_dot_i.TransposeDot(g_i);
            Eigen::RowVector3d grad_g_j = dh_dot_j.TransposeDot(g_j);
            for (int k = 0; k < 3; k++) {
                triplets.emplace_back(row_index, i * 3 + k, grad_g_i(k));
                triplets.emplace_back(row_index, j * 3 + k, grad_g_j(k));
            }

            l(row_index) = - dh_dot_i.TransposeDot(f_i) - dh_dot_j.TransposeDot(f_j) - conInfo.Kb.dot(eta);

        }
        row_index++;
    }

    // if (log_file_.is_open()) {
    //     // Keep data durable during runtime while still batching writes per QP solve.
    //     log_file_.flush();
    // }

    A.setFromTriplets(triplets.begin(), triplets.end());
    
    return NUM_CONS;
}

void Gecbf::compute_dynamic_f(const int drone_id, Eigen::Matrix<double, 13, 1>& f) const {
    // 提取状态变量
    const PartStateType& x = drones_states_[drone_id];
    const double v1 = x.vel(0), v2 = x.vel(1), v3 = x.vel(2);
    
    // 计算 f(x) 各分量
    f.setZero();
    f(0) = v1;
    f(1) = v2;
    f(2) = v3;
    f(3) = 0;
    f(4) = 0;
    f(5) = -G_;
    // Keep attitude and angular velocity static in the controller model.
    // f(6) = 0.0;
    // f(7) = 0.0;
    // f(8) = 0.0;
    // f(9) = 0.0;
    // f(10) = 0.0;
    // f(11) = 0.0;
    // f(12) = 0.0;

}

void Gecbf::compute_dynamic_g(const int drone_id, Eigen::Matrix<double, 13, 3>& g) const {

    // Control input is force vector in world frame.
    g.setZero();
    g(3, 0) = 1.0 / MASS_;
    g(4, 1) = 1.0 / MASS_;
    g(5, 2) = 1.0 / MASS_;
}

void Gecbf::setCtrls(const std::vector<Eigen::Vector3d>& ref_ctrls) {
    for (int i = 0; i < drone_num_; i++)
    {
        curCtrls_[i] = ref_ctrls[i];
    }
}

void Gecbf::setCtrls(const Eigen::VectorXd& solution) {
    for (int i = 0; i < drone_num_; i++)
    {
        // curCtrls_[i](0) = solution(i * 3 + 0);
        // curCtrls_[i](1) = solution(i * 3 + 1);
        // curCtrls_[i](2) = solution(i * 3 + 2);
        curCtrls_[i] = solution.segment<3>(i * 3);
    }
}

void Gecbf::odom_callback(const nav_msgs::Odometry::ConstPtr& odom) {
    // TODO: safe check
    int id = std::stoi(odom->child_frame_id.substr(7));

    drones_states_[id].pos(0) = odom->pose.pose.position.x;
    drones_states_[id].pos(1) = odom->pose.pose.position.y;
    drones_states_[id].pos(2) = odom->pose.pose.position.z;
    drones_states_[id].vel(0) = odom->twist.twist.linear.x;
    drones_states_[id].vel(1) = odom->twist.twist.linear.y;
    drones_states_[id].vel(2) = odom->twist.twist.linear.z;
    drones_states_[id].qua(0) = odom->pose.pose.orientation.w;
    drones_states_[id].qua(1) = odom->pose.pose.orientation.x;
    drones_states_[id].qua(2) = odom->pose.pose.orientation.y;
    drones_states_[id].qua(3) = odom->pose.pose.orientation.z;
    drones_states_[id].ome(0) = odom->twist.twist.angular.x;
    drones_states_[id].ome(1) = odom->twist.twist.angular.y;
    drones_states_[id].ome(2) = odom->twist.twist.angular.z;

    // if (haveOdom_[id] == false) {
    //     updateKb(id);
    //     haveOdom_[id] = true;
    //     return;
    // }

    haveOdom_[id] = true;

    updateNeighbors(id);
}

uint64_t Gecbf::addConstraintsCallback(const int drone_id, const Eigen::Vector3d& obstacle_pos) {
    ConstraintInfoType constraint_info(drone_id, obstacle_pos);
    computeKb(constraint_info);
    uint64_t constraint_key = next_constraint_id_;

    active_constraints_.emplace(constraint_key, constraint_info);
    next_constraint_id_++;
    return constraint_key;
}

uint64_t Gecbf::addConstraintsCallback(const int drone_id, const int other_drone_id) {
    ConstraintInfoType constraint_info(drone_id, other_drone_id);
    computeKb(constraint_info);
    uint64_t constraint_key = next_constraint_id_;

    active_constraints_.emplace(constraint_key, constraint_info);
    next_constraint_id_++;
    return constraint_key;
}

void Gecbf::removeConstraintsCallback(const uint64_t constraint_key) {
    active_constraints_.erase(constraint_key);
}

void Gecbf::closeLogFile() {
    if (log_file_.is_open()) {
        log_file_.flush();
        log_file_.close();
        log_opened_ = false;
        ROS_INFO("Log file closed.");
    }
}
