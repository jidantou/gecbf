#include <array>

// 线性化动力学模型 (悬停点，基于欧拉角)
// 状态维度: 12, 控制维度: 4
// 输入: g (重力), J (惯性矩阵元素)
// 输出: A[12][12] (状态矩阵), B[12][4] (输入矩阵)
void linearized_dynamics(
    double mass, double g, double J11, double J12, double J13,
    double J22, double J23, double J33,
    double A[12][12], double B[12][4]) {

    double x0 = 1.0/m;
    double x1 = g*x0;
    double x2 = J22*J33;
    double x3 = std::pow(J23, 2);
    double x4 = std::pow(J12, 2);
    double x5 = std::pow(J13, 2);
    double x6 = J13*J23;
    double x7 = 1.0/(-J11*x2 + J11*x3 - 2*J12*x6 + J22*x5 + J33*x4);
    double x8 = x7*(J12*J33 - x6);
    double x9 = x7*(-J12*J23 + J13*J22);
    double x10 = x7*(J11*J23 - J12*J13);

    A[0][3] = 1;
    A[1][4] = 1;
    A[2][5] = 1;
    A[3][7] = x1;
    A[4][6] = -x1;
    A[6][9] = 1;
    A[7][10] = 1;
    A[8][11] = 1;

    B[5][0] = x0;
    B[9][1] = x7*(-x2 + x3);
    B[9][2] = x8;
    B[9][3] = x9;
    B[10][1] = x8;
    B[10][2] = x7*(-J11*J33 + x5);
    B[10][3] = x10;
    B[11][1] = x9;
    B[11][2] = x10;
    B[11][3] = x7*(-J11*J22 + x4);
}
