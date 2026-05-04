#include "gcbf_qp_solver/gcbf_min.hpp"
#include <limits>
#include <cmath>

// DEBUG
#include <sstream>

using namespace GCBF_QP_MIN;

Gcbf::Gcbf(ros::NodeHandle &nh, int drone_num = 1, double sensing_horizon = 5.0, double obstacle_distances = 0.05, double drone_distance = 0.1) {
    _DRONE_NUM = drone_num;
    _sensing_horizon = sensing_horizon;
    _obstacle_size = obstacle_distances;
    _drone_size = drone_distance;
    _alpha0 = 1.0;

    // _M_DRONES = 5;
    // // ACC_MAX = 31.75;     // max acceleration calculated from package drone_dynamic
    // ACC_MAX = 10.0;
    // BETA = 50;
    // GAMMA = 1.0;

    nh.param("gcbf/ACC_MAX", ACC_MAX, 10.0);
    nh.param("gcbf/M_DRONES", _M_DRONES, 5);
    nh.param("gcbf/BETA", BETA, 50);
    nh.param("gcbf/GAMMA", GAMMA, 1.0);

    _drones_state_ptr = std::make_unique<PartStateType[]>(drone_num);
    _local_maps = std::make_unique<sphere_detector::SphereDetector[]>(drone_num);
    _neighbors.resize(drone_num);
    _haveOdom.resize(drone_num, false);

    for (int i = 0; i < drone_num; i++)
        _local_maps[i].init(nh, i);

    log_opened_ = false;
}

Gcbf::~Gcbf() {}

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

// ============================================================================
// 辅助函数：f(x), f'(x), sigma(x), sigma'(x)
// ============================================================================

// f(x) = x + (x^beta / R^beta) * (R - x)
// f'(x) = 1 + (beta * x^{beta-1} / R^{beta-1}) - ((beta+1) * x^{beta} / R^{beta})
Gcbf::RTypedd Gcbf::f(double x, double R, int beta) const {
    double xb = std::pow(x, beta);
    double Rb = std::pow(R, beta);
    double xb1 = std::pow(x, beta - 1.0);
    double Rb1 = std::pow(R, beta - 1.0);
    Gcbf::RTypedd res;
    res.value = x + (xb / Rb) * (R - x);
    res.gradient = 1.0 + beta * xb1 / Rb1 - (beta + 1.0) * xb / Rb;

    return res;
}

// sigma(x) 在 [R-gamma, R] 内为三次多项式，外为 1
Gcbf::RTypedd Gcbf::sigma(double x, double R, double gamma) const {
    Gcbf::RTypedd res;
    if (x <= R - gamma) {
        res.value = 1.0;
        res.gradient = 0.0;
        return res;
    }
    else if (x >= R)
    {
        res.value = 0.0;
        res.gradient = 0.0;
        return res;
    }
    
    double g3 = gamma * gamma * gamma;
    double term1 = 2.0 * x * x * x;
    double term2 = 3.0 * (gamma - 2.0 * R) * x * x;
    double term3 = 6.0 * R * (R - gamma) * x;
    double term4 = R * R * (3.0 * gamma - 2.0 * R);
    double gra_poly = x * x + (gamma - 2.0 * R) * x + R * (R - gamma);
    res.value = (term1 + term2 + term3 + term4) / g3;
    res.gradient = (6.0 / g3) * gra_poly;
    return res;
}

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
void Gcbf::compute_h_j_obstacle(const Eigen::Vector3d& o_j,
                        const Eigen::Vector3d& p_i,
                        const Eigen::Vector3d& v_i,
                        double& h,
                        Eigen::Vector3d& dh_dp,
                        Eigen::Vector3d& dh_dv) const {
    Eigen::Vector3d delta = o_j - p_i;
    double rho = delta.norm();
    Eigen::Vector3d d = delta / rho;           // 单位方向向量
    double v_parallel = v_i.dot(d);

    if (v_parallel > 0.0) {
        // h = rho - r - (v_parallel^2)/(2A)
        double vp2 = v_parallel * v_parallel;
        h = rho - _obstacle_size - _drone_size - vp2 / (2.0 * ACC_MAX);

        // 对速度的梯度: dh/dv_i = -(v_parallel/A) * d
        dh_dv = -(v_parallel / ACC_MAX) * d;

        // DEBUG TEST
        dh_dv.z() *= 1.5;

        // 对位置的梯度:
        // dh/dp_i = -d + (v_parallel/(A*rho)) * (v_i - v_parallel*d)
        dh_dp = -d + (v_parallel / (ACC_MAX * rho)) * (v_i - v_parallel * d);
    } else {
        // h = rho - r
        h = rho - _obstacle_size - _drone_size;
        dh_dv.setZero();
        dh_dp = -d;
    }
}

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
void Gcbf::compute_h_j_drone(const Eigen::Vector3d& p_j,
                            const Eigen::Vector3d& v_j,
                            const Eigen::Vector3d& p_i,
                            const Eigen::Vector3d& v_i,
                            double& h,
                            Eigen::Vector3d& dh_dp_i,
                            Eigen::Vector3d& dh_dv_i,
                            Eigen::Vector3d& dh_dp_j,
                            Eigen::Vector3d& dh_dv_j) const {
    Eigen::Vector3d delta = p_j - p_i;
    double rho = delta.norm();
    Eigen::Vector3d d = delta / rho;

    double u = v_i.dot(d);   // v_{i,parallel}
    double w = v_j.dot(d);   // v_{j,parallel}

    // 计算 K 及其对 u,w 的偏导数 K_u, K_w
    double K, Ku, Kw;
    if (u > 0.0 && w < 0.0) {
        K = (u*u + w*w) / (2.0 * ACC_MAX);
        Ku = u / ACC_MAX;
        Kw = w / ACC_MAX;
    } else if ((u > w && w > 0.0) || (w < u && u < 0.0)) {
        K = (u*u - w*w) / (2.0 * ACC_MAX);
        Ku = u / ACC_MAX;
        Kw = -w / ACC_MAX;
    } else {
        K = 0.0;
        Ku = 0.0;
        Kw = 0.0;
    }

    // 计算 f(rho) 和 sigma(rho)
    Gcbf::RTypedd f_res = f(rho, _sensing_horizon, BETA);
    Gcbf::RTypedd s_res = sigma(rho, _sensing_horizon, GAMMA);
    double frho = f_res.value, dfrho = f_res.gradient, 
        srho = s_res.value, dsrho = s_res.gradient;

    // h = f(rho) - 2r - sigma(rho) * K
    h = frho - 2.0 * _drone_size - srho * K;

    // 对速度的梯度
    dh_dv_i = -srho * Ku * d;
    dh_dv_j = -srho * Kw * d;

    // DEBUG TEST
    dh_dv_i.z() *= 5.0;
    dh_dv_j.z() *= 1.0;

    // 公共中间量
    Eigen::Vector3d v_i_perp = v_i - u * d;   // 垂直于 d 的分量
    Eigen::Vector3d v_j_perp = v_j - w * d;

    // 对 p_i 的梯度
    // dh/dp_i = -(df - ds*K)*d + (s/rho)*( Ku*v_i_perp + Kw*v_j_perp )
    double coeff = dfrho - dsrho * K;
    Eigen::Vector3d term1 = -coeff * d;
    Eigen::Vector3d term2 = (srho / rho) * (Ku * v_i_perp + Kw * v_j_perp);
    dh_dp_i = term1 + term2;

    // 对 p_j 的梯度 = -dh_dp_i
    dh_dp_j = -dh_dp_i;
}

// ---------- 修改后的 cal_h ----------
// pair中first为agent的编号，second为该编号对应的的梯度
void Gcbf::cal_h(const int drone_id,
                double& h_value,
                std::vector<std::pair<int,PartStateType>>& gradents) {
    const auto& nb = _neighbors[drone_id];
    int nb_size = nb.size();
    gradents.clear();

    const Eigen::Vector3d& p_i = _drones_state_ptr[drone_id].pos;
    const Eigen::Vector3d& v_i = _drones_state_ptr[drone_id].vel;

    double min_h = std::numeric_limits<double>::infinity();
    enum MinType { NONE, OBSTACLE, DRONE } min_type = NONE;
    int min_nb_j = -1;

    PartStateType min_grad_i, min_grad_j;

    // 1. 障碍物
    const std::vector<Eigen::Vector3d>& obs = _local_maps[drone_id].getVisibleSpheres();
    for (int idx = 0; idx < obs.size(); ++idx) {
        // Eigen::Vector3d o_j = obs[idx];
        double h_val;
        Eigen::Vector3d grad_p, grad_v;
        compute_h_j_obstacle(obs[idx], p_i, v_i, h_val, grad_p, grad_v);
        if (h_val < min_h) {
            min_h = h_val;
            min_type = OBSTACLE;
            min_grad_i.pos = grad_p;
            min_grad_i.vel = grad_v;
        }
    }

    // 2. 其他无人机
    for (int k = 0; k < nb_size; ++k) {
        int j = nb[k];
        if (j == drone_id)
            continue;

        const Eigen::Vector3d& p_j = _drones_state_ptr[j].pos;
        const Eigen::Vector3d& v_j = _drones_state_ptr[j].vel;
        double h_val;
        Eigen::Vector3d grad_pi, grad_vi, grad_pj, grad_vj;
        compute_h_j_drone(p_j, v_j, p_i, v_i, h_val, grad_pi, grad_vi, grad_pj, grad_vj);
        if (h_val < min_h) {
            min_h = h_val;
            min_type = DRONE;
            min_nb_j = j;
            min_grad_i.pos = grad_pi;
            min_grad_i.vel = grad_vi;
            min_grad_j.pos = grad_pj;
            min_grad_j.vel = grad_vj;
        }
    }

    h_value = min_h;
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

void Gcbf::get_neighbors() {
    for (int i = 0; i < _DRONE_NUM; i++)
    {
        _neighbors[i].clear();
        _neighbors[i].emplace_back(i);
    }

    for (int i = 0; i < _DRONE_NUM; i++)
    {
        for (int j = 0; j < _DRONE_NUM; j++)
        {
            if (j == i)
                continue;
            // DEBUG
            // ROS_INFO("DEBUG: gcbf_qp_node: get_neighbors: i: %d, j: %d!", i, j);
            if ((_drones_state_ptr[i].pos - _drones_state_ptr[j].pos).squaredNorm() < _sensing_horizon*_sensing_horizon)
            {
                _neighbors[i].emplace_back(j);
            }

            if (_neighbors[i].size() >= _M_DRONES)
                break;
        }
    }
}

void Gcbf::computeQPCoefficientsAl(Eigen::SparseMatrix<double>& A, Eigen::VectorXd& l) {
    if (!log_opened_) {
        const char* home = std::getenv("HOME");
        if (!home)
        {
            ROS_ERROR("HOME environment variable not set");
            ros::shutdown();
        }
        std::string log_path = std::string(home) + "/Projects/gcbf_log/gcbf_h_values.csv";
        log_file_.open(log_path, std::ios::out);
        if (log_file_.is_open()) {
            ROS_INFO("GCBF h_value log file opened: %s", log_path.c_str());
            log_file_ << "timestamp,drone_id,h_value\n";
            log_opened_ = true;
        } else {
            ROS_WARN("Failed to open log file: %s", log_path.c_str());
        }
    }

    for (int i = 0; i < _DRONE_NUM; ++i) {
        if (!_haveOdom[i]) {
            A.setZero();
            A.insert(0, 0) = 0.0; // 添加一个无效约束，迫使优化器选择一个特定的解
            l.setConstant(-100.0);
            return;
        }
    } 

    // 获取当前时间（秒）
    double current_time = ros::Time::now().toSec();

    std::vector<Eigen::Matrix<double, 13, 1>> f(_DRONE_NUM);
    std::vector<Eigen::Matrix<double, 13, 3>> g(_DRONE_NUM);
    // Eigen::Matrix<double, 13, 1> f;
    // Eigen::Matrix<double, 13, 4> g;

    get_neighbors();

    for (int i = 0; i < _DRONE_NUM; i++)
    {
        compute_dynamic_f(i, f[i]);
        compute_dynamic_g(i, g[i]);
    }

    // DEBUG
    // ROS_INFO("DEBUG: gcbf_qp_node: compute_Qp_coe: after get_neighbors!");

    A.setZero();
    l.setZero();

    std::vector<Eigen::Triplet<double>> triplets;
    // triplets.reserve(_DRONE_NUM * 8);

    for (int i = 0; i < _DRONE_NUM; i++)
    {
        if (!_haveOdom[i]) {
            l(i) = -100.0;
            // triplets.emplace_back(i, 0, 0.0);
            continue;
        }

        double h_value;
        std::vector<std::pair<int,PartStateType>> gradents;
        cal_h(i, h_value, gradents);

        // ========== 记录 h_value 到日志文件 ==========
        if (log_file_.is_open()) {
            log_file_ << std::fixed << std::setprecision(9) 
                      << current_time << "," << i << "," << h_value << "\n";
        }

        if (h_value < 0) {
            std::ostringstream oss;
            char s[100];
            snprintf(s, 100, "h_value < 0! %d: h_value: %.3f, gradents:\n", debug_count, h_value);
            oss << s;
            for (auto &&temp_debug : gradents)
            {
                snprintf(s, 100, "    %d: p1:%.2f, p2:%.2f, p3:%.2f, v1:%.2f, v2:%.2f, v3:%.3f\n", temp_debug.first, temp_debug.second.pos(0), temp_debug.second.pos(1), temp_debug.second.pos(2), temp_debug.second.vel(0), temp_debug.second.vel(1), temp_debug.second.vel(2));
                oss << s;
            }
            ROS_WARN("%s", oss.str().c_str());
            // ROS_INFO("%s", oss.str().c_str());
        }

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
            snprintf(s, 100, "%d: h_value: %.3f, gradents:\n", debug_count, h_value);
            oss << s;
            for (auto &&temp_debug : gradents)
            {
                snprintf(s, 100, "    %d: p1:%.2f, p2:%.2f, p3:%.2f, v1:%.2f, v2:%.2f, v3:%.3f\n", temp_debug.first, temp_debug.second.pos(0), temp_debug.second.pos(1), temp_debug.second.pos(2), temp_debug.second.vel(0), temp_debug.second.vel(1), temp_debug.second.vel(2));
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
        l(i) += - _alpha0 * h_value;
    }

    if (triplets.size() == 0) {
        triplets.emplace_back(0, 0, 0.0);
        l(0) = -100.0;
    }

    A.setFromTriplets(triplets.begin(), triplets.end());
}

void Gcbf::compute_dynamic_f(const int drone_id, Eigen::Matrix<double, 13, 1>& f) const {
    // 提取状态变量
    const PartStateType& x = _drones_state_ptr[drone_id];
    const double v1 = x.vel(0), v2 = x.vel(1), v3 = x.vel(2);
    const double g_gravity = 9.8;
    
    // 计算 f(x) 各分量
    f.setZero();
    f(0) = v1;
    f(1) = v2;
    f(2) = v3;
    f(3) = 0;
    f(4) = 0;
    f(5) = -g_gravity;
    // Keep attitude and angular velocity static in the controller model.
    // f(6) = 0.0;
    // f(7) = 0.0;
    // f(8) = 0.0;
    // f(9) = 0.0;
    // f(10) = 0.0;
    // f(11) = 0.0;
    // f(12) = 0.0;

}

void Gcbf::compute_dynamic_g(const int drone_id, Eigen::Matrix<double, 13, 3>& g) const {
    (void)drone_id;
    const double mass = 0.98;

    // Control input is force vector in world frame.
    g.setZero();
    g(3, 0) = 1.0 / mass;
    g(4, 1) = 1.0 / mass;
    g(5, 2) = 1.0 / mass;
}

void Gcbf::odom_callback(const nav_msgs::Odometry::ConstPtr& odom) {
  // TODO: safe check
  int id = std::stoi(odom->child_frame_id.substr(7));

  _drones_state_ptr[id].pos(0) = odom->pose.pose.position.x;
  _drones_state_ptr[id].pos(1) = odom->pose.pose.position.y;
  _drones_state_ptr[id].pos(2) = odom->pose.pose.position.z;
  _drones_state_ptr[id].vel(0) = odom->twist.twist.linear.x;
  _drones_state_ptr[id].vel(1) = odom->twist.twist.linear.y;
  _drones_state_ptr[id].vel(2) = odom->twist.twist.linear.z;
  _drones_state_ptr[id].qua(0) = odom->pose.pose.orientation.w;
  _drones_state_ptr[id].qua(1) = odom->pose.pose.orientation.x;
  _drones_state_ptr[id].qua(2) = odom->pose.pose.orientation.y;
  _drones_state_ptr[id].qua(3) = odom->pose.pose.orientation.z;
  _drones_state_ptr[id].ome(0) = odom->twist.twist.angular.x;
  _drones_state_ptr[id].ome(1) = odom->twist.twist.angular.y;
  _drones_state_ptr[id].ome(2) = odom->twist.twist.angular.z;

  _haveOdom[id] = true;
}

void Gcbf::closeLogFile() {
    if (log_file_.is_open()) {
        log_file_.flush();
        log_file_.close();
        ROS_INFO("Log file closed.");
    }
}
