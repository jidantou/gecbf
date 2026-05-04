
#include <Eigen/Dense>

// 状态向量顺序: [p1, p2, p3, v1, v2, v3, q0, q1, q2, q3, w1, w2, w3]
// 控制向量顺序: [c, tau1, tau2, tau3]

void compute_f(const Eigen::Matrix<double, 13, 1>& x, Eigen::Matrix<double, 13, 1>& f) {
    // 提取状态变量
    double p1 = x(0), p2 = x(1), p3 = x(2);
    double v1 = x(3), v2 = x(4), v3 = x(5);
    double q0 = x(6), q1 = x(7), q2 = x(8), q3 = x(9);
    double w1 = x(10), w2 = x(11), w3 = x(12);
    
    // 常量
    const double g = g;
    // 惯性矩阵元素（常量，需根据实际系统设置）
    const double J11 = J11, J12 = J12, J13 = J13;
    const double J22 = J22, J23 = J23, J33 = J33;
    // 惯性矩阵逆的元素（预计算）
    const double J_inv11 = (J22*J33 - std::pow(J23, 2))/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    const double J_inv12 = (-J12*J33 + J13*J23)/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    const double J_inv13 = (J12*J23 - J13*J22)/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    const double J_inv21 = (-J12*J33 + J13*J23)/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    const double J_inv22 = (J11*J33 - std::pow(J13, 2))/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    const double J_inv23 = (-J11*J23 + J12*J13)/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    const double J_inv31 = (J12*J23 - J13*J22)/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    const double J_inv32 = (-J11*J23 + J12*J13)/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    const double J_inv33 = (J11*J22 - std::pow(J12, 2))/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    
    // 计算 f(x) 各分量
    f(0) = v1;
    f(1) = v2;
    f(2) = v3;
    f(3) = 0;
    f(4) = 0;
    f(5) = -g;
    f(6) = -1.0/2.0*q1*w1 - 1.0/2.0*q2*w2 - 1.0/2.0*q3*w3;
    f(7) = (1.0/2.0)*q0*w1 + (1.0/2.0)*q2*w3 - 1.0/2.0*q3*w2;
    f(8) = (1.0/2.0)*q0*w2 - 1.0/2.0*q1*w3 + (1.0/2.0)*q3*w1;
    f(9) = (1.0/2.0)*q0*w3 + (1.0/2.0)*q1*w2 - 1.0/2.0*q2*w1;
    f(10) = -(J12*J23 - J13*J22)*(w1*(J12*w1 + J22*w2 + J23*w3) - w2*(J11*w1 + J12*w2 + J13*w3))/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22) - (-J12*J33 + J13*J23)*(-w1*(J13*w1 + J23*w2 + J33*w3) + w3*(J11*w1 + J12*w2 + J13*w3))/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22) - (J22*J33 - std::pow(J23, 2))*(w2*(J13*w1 + J23*w2 + J33*w3) - w3*(J12*w1 + J22*w2 + J23*w3))/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    f(11) = -(-J11*J23 + J12*J13)*(w1*(J12*w1 + J22*w2 + J23*w3) - w2*(J11*w1 + J12*w2 + J13*w3))/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22) - (J11*J33 - std::pow(J13, 2))*(-w1*(J13*w1 + J23*w2 + J33*w3) + w3*(J11*w1 + J12*w2 + J13*w3))/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22) - (-J12*J33 + J13*J23)*(w2*(J13*w1 + J23*w2 + J33*w3) - w3*(J12*w1 + J22*w2 + J23*w3))/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    f(12) = -(J11*J22 - std::pow(J12, 2))*(w1*(J12*w1 + J22*w2 + J23*w3) - w2*(J11*w1 + J12*w2 + J13*w3))/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22) - (-J11*J23 + J12*J13)*(-w1*(J13*w1 + J23*w2 + J33*w3) + w3*(J11*w1 + J12*w2 + J13*w3))/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22) - (J12*J23 - J13*J22)*(w2*(J13*w1 + J23*w2 + J33*w3) - w3*(J12*w1 + J22*w2 + J23*w3))/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);

}

void compute_g(const Eigen::Matrix<double, 13, 1>& x, Eigen::Matrix<double, 13, 4>& g) {
    // 提取状态变量
    double p1 = x(0), p2 = x(1), p3 = x(2);
    double v1 = x(3), v2 = x(4), v3 = x(5);
    double q0 = x(6), q1 = x(7), q2 = x(8), q3 = x(9);
    double w1 = x(10), w2 = x(11), w3 = x(12);
    
    // 常量
    const double g_const = g;
    // 惯性矩阵元素
    const double J11 = J11, J12 = J12, J13 = J13;
    const double J22 = J22, J23 = J23, J33 = J33;
    // 惯性矩阵逆的元素（与 f 中相同）
    const double J_inv11 = (J22*J33 - std::pow(J23, 2))/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    const double J_inv12 = (-J12*J33 + J13*J23)/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    const double J_inv13 = (J12*J23 - J13*J22)/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    const double J_inv21 = (-J12*J33 + J13*J23)/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    const double J_inv22 = (J11*J33 - std::pow(J13, 2))/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    const double J_inv23 = (-J11*J23 + J12*J13)/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    const double J_inv31 = (J12*J23 - J13*J22)/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    const double J_inv32 = (-J11*J23 + J12*J13)/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    const double J_inv33 = (J11*J22 - std::pow(J12, 2))/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    
    // 计算 g(x) 矩阵，按行优先存储（13行4列）
    g(0,0) = 0.0;
    g(0,1) = 0.0;
    g(0,2) = 0.0;
    g(0,3) = 0.0;
    g(1,0) = 0.0;
    g(1,1) = 0.0;
    g(1,2) = 0.0;
    g(1,3) = 0.0;
    g(2,0) = 0.0;
    g(2,1) = 0.0;
    g(2,2) = 0.0;
    g(2,3) = 0.0;
    g(3,0) = 2*q0*q2 + 2*q1*q3;
    g(3,1) = 0.0;
    g(3,2) = 0.0;
    g(3,3) = 0.0;
    g(4,0) = -2*q0*q1 + 2*q2*q3;
    g(4,1) = 0.0;
    g(4,2) = 0.0;
    g(4,3) = 0.0;
    g(5,0) = -2*std::pow(q1, 2) - 2*std::pow(q2, 2) + 1;
    g(5,1) = 0.0;
    g(5,2) = 0.0;
    g(5,3) = 0.0;
    g(6,0) = 0.0;
    g(6,1) = 0.0;
    g(6,2) = 0.0;
    g(6,3) = 0.0;
    g(7,0) = 0.0;
    g(7,1) = 0.0;
    g(7,2) = 0.0;
    g(7,3) = 0.0;
    g(8,0) = 0.0;
    g(8,1) = 0.0;
    g(8,2) = 0.0;
    g(8,3) = 0.0;
    g(9,0) = 0.0;
    g(9,1) = 0.0;
    g(9,2) = 0.0;
    g(9,3) = 0.0;
    g(10,0) = 0.0;
    g(10,1) = (J22*J33 - std::pow(J23, 2))/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    g(10,2) = (-J12*J33 + J13*J23)/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    g(10,3) = (J12*J23 - J13*J22)/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    g(11,0) = 0.0;
    g(11,1) = (-J12*J33 + J13*J23)/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    g(11,2) = (J11*J33 - std::pow(J13, 2))/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    g(11,3) = (-J11*J23 + J12*J13)/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    g(12,0) = 0.0;
    g(12,1) = (J12*J23 - J13*J22)/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    g(12,2) = (-J11*J23 + J12*J13)/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
    g(12,3) = (J11*J22 - std::pow(J12, 2))/(J11*J22*J33 - J11*std::pow(J23, 2) - std::pow(J12, 2)*J33 + 2*J12*J13*J23 - std::pow(J13, 2)*J22);
}
