#include "gecbf/gecbf_min.hpp"
#include <limits>
#include <cmath>

// DEBUG
#include <sstream>

using namespace GECBF_QP_MIN;

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
    nh.param("gecbf/Kb_delta", Kb_delta_, 0.1);

    drones_states_.resize(drone_num);
    local_maps_.resize(drone_num);
    neighbors_.resize(drone_num);
    Kbs_.reserve(drone_num);
    haveOdom_.resize(drone_num, false);
    curCtrls_.reserve(drone_num);

    for (int i = 0; i < drone_num; i++) {
        local_maps_[i].init(nh, i);
        neighbors_[i].insert(i);
        curCtrls_.emplace_back(0.0, 0.0, MASS_ * G_);
        Kbs_.emplace_back(Kb_delta_, Kb_delta_);
    }

    log_opened_ = false;
}

Gecbf::~Gecbf() {}

Eigen::Vector3d PartStateType::TransposeDot(const Eigen::Matrix<double, 13, 3>& other) const {
    // 计算 temp^T * other，其中 temp = [pos; vel; qua; ome]
    // 将 other 按行分块：pos 对应 0-2 行，vel 对应 3-5 行，qua 对应 6-9 行，ome 对应 10-12 行
    Eigen::Matrix<double, 1, 3> row_result;
    row_result.noalias() = pos.transpose() * other.topRows<3>()          // 1x4
                         + vel.transpose() * other.middleRows<3>(3)     // 1x4
                         + qua.transpose() * other.middleRows<4>(6)     // 1x4
                         + ome.transpose() * other.bottomRows<3>();     // 1x4
    return row_result;  // 转为列向量
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
                                const Eigen::Vector3d& p_i,
                                const Eigen::Vector3d& v_i,
                                Eigen::Vector2d& eta,
                                Eigen::Vector3d& dh_dot_dp,
                                Eigen::Vector3d& dh_dot_dv) const{
    const Eigen::Vector3d diff = o_j - p_i;
    const double dist_sq = diff.squaredNorm();

    // h = ||o_j - p_i||^2 - (r_o + r_d)^2
    const double sum_radii = obstacle_size_ + drone_size_;
    const double h = dist_sq - sum_radii * sum_radii;

    // h_dot = -2 (o_j - p_i) · v_i
    const double h_dot = -2.0 * diff.dot(v_i);

    // eta = [h, h_dot]
    eta(0) = h;
    eta(1) = h_dot;

    // Gradient of h_dot w.r.t p_i:  2 * v_i
    dh_dot_dp = 2.0 * v_i;

    // Gradient of h_dot w.r.t v_i: -2 * (o_j - p_i)
    dh_dot_dv = -2.0 * diff;
}

void Gecbf::compute_h_j_drone(const Eigen::Vector3d& p_j,
                            const Eigen::Vector3d& v_j,
                            const Eigen::Vector3d& p_i,
                            const Eigen::Vector3d& v_i,
                            Eigen::Vector2d& eta,
                            Eigen::Vector3d& dh_dot_dp_i,
                            Eigen::Vector3d& dh_dot_dv_i,
                            Eigen::Vector3d& dh_dot_dp_j,
                            Eigen::Vector3d& dh_dot_dv_j) const{
    const Eigen::Vector3d B = p_j - p_i;          // 相对位置
    const Eigen::Vector3d C = v_j - v_i;          // 相对速度

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
    dh_dot_dp_i = -coeff1 * B - coeff2 * C;

    // dh_dot / dv_i = -2 ξ'(s) B
    dh_dot_dv_i = -coeff2 * B;

    // dh_dot / dp_j =  4 ξ''(s) (B·C) B + 2 ξ'(s) C
    dh_dot_dp_j = coeff1 * B + coeff2 * C;

    // dh_dot / dv_j =  2 ξ'(s) B
    dh_dot_dv_j = coeff2 * B;
}

// ---------- 修改后的 cal_h ----------
// pair中first为agent的编号，second为该编号对应的的梯度
void Gecbf::cal_h(const int drone_id,
                Eigen::Vector2d& eta,
                std::vector<std::pair<int,PartStateType>>& gradents) {
    const auto& nb = neighbors_[drone_id];
    gradents.clear();

    const Eigen::Vector3d& p_i = drones_states_[drone_id].pos;
    const Eigen::Vector3d& v_i = drones_states_[drone_id].vel;

    Eigen::Vector2d eta_min{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    enum MinType { NONE, OBSTACLE, DRONE } min_type = NONE;
    int min_nb_j = -1;

    PartStateType min_grad_i, min_grad_j;

    // 1. 障碍物
    const std::vector<Eigen::Vector3d>& obs = local_maps_[drone_id].getVisibleSpheres();
    for (int idx = 0; idx < obs.size(); ++idx) {
        // Eigen::Vector3d o_j = obs[idx];
        Eigen::Vector2d eta_internal;
        Eigen::Vector3d grad_p, grad_v;
        compute_h_j_obstacle(obs[idx], p_i, v_i, eta_internal, grad_p, grad_v);

        // DEBUG
        if (debug_count % 50 == 0) {
            std::stringstream ss;
            ss << drone_id << ":  Obstacle:" << idx << ",  h:" << eta_internal(0) << ",  h_dot:" << eta_internal(1);
            ROS_INFO_STREAM(ss.str());
        }

        if (eta_internal(0) < eta_min(0) || (std::abs(eta_internal(0) - eta_min(0)) < 1e-6 && eta_internal(1) < eta_min(1))) {
            eta_min = eta_internal;
            min_type = OBSTACLE;
            min_grad_i.pos = grad_p;
            min_grad_i.vel = grad_v;
        }
    }

    // 2. 其他无人机
    for (auto &&j : nb) {
        if (j == drone_id)
            continue;

        const Eigen::Vector3d& p_j = drones_states_[j].pos;
        const Eigen::Vector3d& v_j = drones_states_[j].vel;
        Eigen::Vector2d eta_internal;
        Eigen::Vector3d grad_pi, grad_vi, grad_pj, grad_vj;
        compute_h_j_drone(p_j, v_j, p_i, v_i, eta_internal, grad_pi, grad_vi, grad_pj, grad_vj);
        if (eta_internal(0) < eta_min(0) || (std::abs(eta_internal(0) - eta_min(0)) < 1e-6 && eta_internal(1) < eta_min(1))) {
            eta_min = eta_internal;
            min_type = DRONE;
            min_nb_j = j;
            min_grad_i.pos = grad_pi;
            min_grad_i.vel = grad_vi;
            min_grad_j.pos = grad_pj;
            min_grad_j.vel = grad_vj;
        }
    }

    eta = eta_min;
    if (min_type == NONE)
        return;

    // 3. 填入梯度
    if (min_type == OBSTACLE) {
        gradents.emplace_back(drone_id, min_grad_i);
    } else { // DRONE
        gradents.emplace_back(drone_id, min_grad_i);
        gradents.emplace_back(min_nb_j, min_grad_j);
    }
}

void Gecbf::updateNeighbors(const int drone_id) {
    auto& nb = neighbors_[drone_id];
    std::vector<bool> has_changed(drone_num_, false);
    const double R2 = sensing_horizon_ * sensing_horizon_;

    for (int j = 0; j < drone_num_; j++) {
        if (j == drone_id)
            continue;
        if (nb.find(j) != nb.end()) {
            // 已经是邻居了，检查是否还满足条件
            if ((drones_states_[drone_id].pos - drones_states_[j].pos).squaredNorm() > R2) {
                nb.erase(j);
                neighbors_[j].erase(drone_id);
                has_changed[drone_id] = true;
                has_changed[j] = true;
            }
        }
        else {
            // 不是邻居，检查是否满足成为邻居的条件
            if ((drones_states_[drone_id].pos - drones_states_[j].pos).squaredNorm() <= R2)
            {
                nb.insert(j);
                neighbors_[j].insert(drone_id);
                has_changed[drone_id] = true;
                has_changed[j] = true;
            }
        }
    }
    
    for (int k = 0; k < drone_num_; k++) {
        if (has_changed[k])
            updateKb(k);
    }
    
}

void Gecbf::updateKb(const int drone_id) {
    double p1, p2, l1, l2;
    Eigen::Vector2d eta;
    std::vector<std::pair<int,PartStateType>> gradents;
    cal_h(drone_id, eta, gradents);

    double h_dotdot = 0.0;

    for (auto &&gra_j : gradents) {
        Eigen::Matrix<double, 13, 1> f;
        Eigen::Matrix<double, 13, 3> g;
        compute_dynamic_f(gra_j.first, f);
        compute_dynamic_g(gra_j.first, g);
        Eigen::Matrix<double, 13, 1> x_dot = f + g * curCtrls_[gra_j.first];

        h_dotdot += gra_j.second.TransposeDot(x_dot);
    }
    
    l1 = - eta(1) / eta(0);
    if (l1 > 0)
        p1 = Kb_alpha_ * l1;
    else
        p1 = Kb_delta_;
    
    l2 = - (h_dotdot + p1 * eta(1)) / (eta(1) + p1 * eta(0));
    if (l2 > 0)
        p2 = Kb_alpha_ * l2;
    else
        p2 = Kb_delta_;

    Kbs_[drone_id](0) = p1 * p2;
    Kbs_[drone_id](1) = p1 + p2;
}

void Gecbf::computeQPCoefficientsAl(Eigen::SparseMatrix<double>& A, Eigen::VectorXd& l) {
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
            log_file_ << "timestamp,drone_id,h_value,h_dot_value\n";
            log_opened_ = true;
        } else {
            ROS_WARN("Failed to open log file: %s", log_path.c_str());
        }
    }

    // 获取当前时间（秒）
    double current_time = ros::Time::now().toSec();

    std::vector<Eigen::Matrix<double, 13, 1>> f(drone_num_);
    std::vector<Eigen::Matrix<double, 13, 3>> g(drone_num_);
    // Eigen::Matrix<double, 13, 1> f;
    // Eigen::Matrix<double, 13, 4> g;

    // get_neighbors();

    for (int i = 0; i < drone_num_; i++)
    {
        compute_dynamic_f(i, f[i]);
        compute_dynamic_g(i, g[i]);
    }

    // DEBUG
    // ROS_INFO("DEBUG: gecbf_qp_node: compute_Qp_coe: after get_neighbors!");

    A.setZero();
    l.setZero();

    std::vector<Eigen::Triplet<double>> triplets;
    // triplets.reserve(_DRONE_NUM * 8);

    for (int i = 0; i < drone_num_; i++)
    {
        if (!haveOdom_[i]) {
            l(i) = -100.0;
            // triplets.emplace_back(i, 0, 0.0);
            continue;
        }

        Eigen::Vector2d eta;
        std::vector<std::pair<int,PartStateType>> gradents;
        cal_h(i, eta, gradents);

        // ========== 记录 h_value 到日志文件 ==========
        if (log_file_.is_open()) {
            log_file_ << std::fixed << std::setprecision(9) 
                      << current_time << "," << i << "," << eta(0) << "," << eta(1) << "\n";
        }

        // safe check
        if (eta(0) < 0) {
            std::ostringstream oss;
            char s[100];
            snprintf(s, 100, "h_value < 0! %d: h_value: %.3f, gradents:\n", debug_count, eta(0));
            oss << s;
            for (auto &&temp_debug : gradents)
            {
                snprintf(s, 100, "    %d: p1:%.2f, p2:%.2f, p3:%.2f, v1:%.2f, v2:%.2f, v3:%.3f\n", temp_debug.first, temp_debug.second.pos(0), temp_debug.second.pos(1), temp_debug.second.pos(2), temp_debug.second.vel(0), temp_debug.second.vel(1), temp_debug.second.vel(2));
                oss << s;
            }
            ROS_WARN("%s", oss.str().c_str());
            // ROS_INFO("%s", oss.str().c_str());
        }

        // no obstacle or neighbor, set a dummy constraint to keep the optimization problem feasible
        if (gradents.size() == 0) {
            l(i) = -100.0;
            // triplets.emplace_back(i, 0, 0.0);
            continue;
        }

        // DEBUG
        if (debug_count % 50 == 0)
        {
            std::ostringstream oss;
            char s[100];
            snprintf(s, 100, "%d: h_value: %.3f, h_dot_value: %.3f, gradents:", debug_count, eta(0), eta(1));
            oss << s;
            for (auto &&temp_debug : gradents)
            {
                snprintf(s, 100, "    %d: p1:%.2f, p2:%.2f, p3:%.2f, v1:%.2f, v2:%.2f, v3:%.3f", temp_debug.first, temp_debug.second.pos(0), temp_debug.second.pos(1), temp_debug.second.pos(2), temp_debug.second.vel(0), temp_debug.second.vel(1), temp_debug.second.vel(2));
                oss << s;
            }
            ROS_INFO("%s", oss.str().c_str());
        }
        
        for (auto &&j : gradents)
        {
            auto temp = j.second.TransposeDot(g[j.first]);
            for (int k = 0; k < 3; k++)
                // A.insert(i, j.first*4 + k) = temp(k);
                triplets.emplace_back(i, j.first * 3 + k, temp(k));
            l(i) -= j.second.TransposeDot(f[j.first]);
        }
        l(i) -= Kbs_[i].dot(eta);

        // DEBUG
        if (debug_count % 50 == 0)
        {
            std::ostringstream oss;
            char s[100];
            snprintf(s, 100, "%d: l(%d): %.3f, Kb:[%.3f, %.3f], eta:[%.3f, %.3f]", debug_count, i, l(i), Kbs_[i](0), Kbs_[i](1), eta(0), eta(1));
            oss << s;
            ROS_INFO("%s", oss.str().c_str());
        }
    }

    if (triplets.size() == 0) {
        triplets.emplace_back(0, 0, 0.0);
        l(0) = -100.0;
    }

    A.setFromTriplets(triplets.begin(), triplets.end());
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

    if (haveOdom_[id] == false) {
        updateKb(id);
        haveOdom_[id] = true;
        return;
    }

    // haveOdom_[id] = true;

    updateNeighbors(id);
}

void Gecbf::closeLogFile() {
    if (log_file_.is_open()) {
        log_file_.flush();
        log_file_.close();
        ROS_INFO("Log file closed.");
    }
}
