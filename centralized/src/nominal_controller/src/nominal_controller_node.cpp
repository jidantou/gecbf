#include <ros/ros.h>
#include "quadrotor_msgs/Force3.h"
#include <nav_msgs/Odometry.h>
#include <memory>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

typedef Eigen::Matrix<double, 13, 12> A_Type;
typedef Eigen::Matrix<double, 13, 4> B_Type;
// typedef Eigen::Matrix<double, 13, 1> X_Type;    // x,  y,  z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz
typedef Eigen::Matrix<double, 13, 1> X_Type;    // x,  y,  z, vx, vy, vz, phi, theta, psi, wx, wy, wz
typedef Eigen::Vector3d U_Type;

X_Type current_state, goal_state;
// Eigen::Vector4d current_qua, goal_qua;
int drone_id;
double TRA_V_MAX, TRA_A, ctrl_fre;
bool getGoal;

// DEBUG
static int count = 0;

Eigen::Vector3d kp, kd;

// void linearized_dynamics(
// 	double mass, double g, double J11, double J12, double J13,
// 	double J22, double J23, double J33,
// 	A_Type& A, B_Type& B);

// void lqr_control(const A_Type& A,
// 				const B_Type& B,
// 				const Eigen::Matrix<double, 12, 1>& Q_vec,
// 				const Eigen::Matrix<double, 4, 1>& R_vec,
// 				Eigen::Matrix<double, 4, 12>& K);

void pd_control(const X_Type& x, const Eigen::Vector3d& pos_des, const Eigen::Vector3d& vel_des,
				const double g, const double mass, U_Type& u);

void trajectoryGenerater(const X_Type& x, const X_Type x_goal, Eigen::Vector3d& pos_des, Eigen::Vector3d& vel_des, double yaw_des);

void quaternionToRotationMatrix(const Eigen::Vector4d& q, Eigen::Matrix3d& R);

void quaternionToEulerAngle(const Eigen::Vector4d& q, double& phi, double& theta, double& psi);

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
	current_state(0) = odom->pose.pose.position.x;
	current_state(1) = odom->pose.pose.position.y;
	current_state(2) = odom->pose.pose.position.z;
	current_state(3) = odom->twist.twist.linear.x;
	current_state(4) = odom->twist.twist.linear.y;
	current_state(5) = odom->twist.twist.linear.z;
	current_state(6) = odom->pose.pose.orientation.w;
	current_state(7) = odom->pose.pose.orientation.x;
	current_state(8) = odom->pose.pose.orientation.y;
	current_state(9) = odom->pose.pose.orientation.z;
	// current_qua(0) = odom->pose.pose.orientation.w;
	// current_qua(1) = odom->pose.pose.orientation.x;
	// current_qua(2) = odom->pose.pose.orientation.y;
	// current_qua(3) = odom->pose.pose.orientation.z;
	// quaternionToEulerAngle(current_qua, current_state(6), current_state(7), current_state(8));

	current_state(10) = odom->twist.twist.angular.x;
	current_state(11) = odom->twist.twist.angular.y;
	current_state(12) = odom->twist.twist.angular.z;
}

void task_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
	goal_state(0) = pos->pose.position.x;
	goal_state(1) = pos->pose.position.y;
	goal_state(2) = pos->pose.position.z;
	goal_state(3) = 0;
	goal_state(4) = 0;
	goal_state(5) = 0;
	goal_state(6) = pos->pose.orientation.w;
	goal_state(7) = pos->pose.orientation.x;
	goal_state(8) = pos->pose.orientation.y;
	goal_state(9) = pos->pose.orientation.z;
	// current_qua(0) = pos->pose.orientation.w;
	// current_qua(1) = pos->pose.orientation.x;
	// current_qua(2) = pos->pose.orientation.y;
	// current_qua(3) = pos->pose.orientation.z;
	// quaternionToEulerAngle(current_qua, goal_state(6), goal_state(7), goal_state(8));

	goal_state(10) = 0;
	goal_state(11) = 0;
	goal_state(12) = 0;

	getGoal = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nominal_controller");
	ros::NodeHandle nh("~");

	ros::Subscriber task_sub = nh.subscribe("task_pose", 10, task_pose_callback);
	ros::Subscriber odom_sub = nh.subscribe("odom", 50, odom_callback, ros::TransportHints().tcpNoDelay());
	ros::Publisher ref_ctrl_pub = nh.advertise<quadrotor_msgs::Force3>("ref_control", 100);

	// TODO: read drone params from yaml file
	double g = 9.81;
	double mass = 0.98;

	// DEBUG
	// try
	// {
	
	Eigen::Vector3d pos_des, vel_des;
	double yaw_des = 0;

	U_Type u_ref;
	//    	 x,  y,  z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz
	//	  	 x,  y,  z, vx, vy, vz, phi, theta, psi, wx, wy, wz
	// Q_vec << 5, 5, 5, 1, 1, 1,  4,  4,  4, 1, 1, 1;
	//        c, tau_x, tau_y, tau_z
	// R_vec <<  1,  1,  1,  1;

	// linearized_dynamics(mass, g, Ixx, 0.0, 0.0, Iyy, 0.0, Izz, A, B);
	// lqr_control(A, B, Q_vec, R_vec, K);

	nh.param("control_frequence", ctrl_fre, 100.0);
	nh.param("drone_id", drone_id, -1);
	nh.param("trajectory_max_velocity", TRA_V_MAX, 1.0);
	nh.param("trajectory_desired_accelerate", TRA_A, 4.0);

	nh.param("kp_0", kp(0), 5.7);
	nh.param("kp_1", kp(1), 5.7);
	nh.param("kp_2", kp(2), 6.2);
	nh.param("kd_0", kd(0), 3.4);
	nh.param("kd_1", kd(1), 3.4);
	nh.param("kd_2", kd(2), 4.0);
	getGoal = false;

	ros::Rate control_rate(ctrl_fre);
	while (ros::ok())
	{
		ros::spinOnce();

		if (!getGoal)
			goal_state = current_state;

		trajectoryGenerater(current_state, goal_state, pos_des, vel_des, yaw_des);
		// DEBUG
		// if (count++ % 20 == 0) {
		// 	// ROS_INFO("%d: pos: (%.3f,%.3f,%.3f)  qua: (%.3f,%.3f,%.3f,%.3f)", count,
		// 	// 		current_state(0), current_state(1), current_state(2),
		// 	// 		current_state(6), current_state(7), current_state(8), current_state(9));
		// 	// ROS_INFO("%d: pos_goal: (%.3f,%.3f,%.3f)  qua_goal: (%.3f,%.3f,%.3f,%.3f)  u: (%.3f, %.3f, %.3f, %.3f)", count,
		// 	// 		goal_state(0), goal_state(1), goal_state(2),
		// 	// 		goal_state(6), goal_state(7), goal_state(8), goal_state(9),
		// 	// 		u_ref(0), u_ref(1), u_ref(2), u_ref(3));
		// 	ROS_INFO("%d: pos: (%.3f,%.3f,%.3f)  pos_des: (%.3f,%.3f,%.3f)  vel: (%.3f,%.3f,%.3f)  vel_des: (%.3f,%.3f,%.3f)", count,
		// 		current_state(0), current_state(1), current_state(2), pos_des(0), pos_des(1), pos_des(2),
		// 		current_state(3), current_state(4), current_state(5), vel_des(0), vel_des(1), vel_des(2));
		// }

		// u_ref = -K * (current_state - goal_state);
		pd_control(current_state, pos_des, vel_des, g, mass, u_ref);

		quadrotor_msgs::Force3 msg;
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "/drone_" + std::to_string(drone_id);
		msg.force.x = u_ref(0);
		msg.force.y = u_ref(1);
		msg.force.z = u_ref(2);
		ref_ctrl_pub.publish(msg);

		control_rate.sleep();
	}
	
	// }
	// catch(const std::exception& e)
	// {
	// 	ROS_ERROR("nominal controller error! ");
	// 	ROS_ERROR(e.what());
	// 	std::cerr << e.what() << '\n';
	// }
}

// 线性化动力学模型 (悬停点，基于欧拉角)
// 状态维度: 12, 控制维度: 4
// 输入: g (重力), J (惯性矩阵元素)
// 输出: A[12][12] (状态矩阵), B[12][4] (输入矩阵)
void linearized_dynamics(
    double mass, double g, double J11, double J12, double J13,
    double J22, double J23, double J33,
    A_Type& A, B_Type& B) {

    double x0 = 1.0/mass;
    double x1 = g*x0;
    double x2 = J22*J33;
    double x3 = J23*J23;
    double x4 = J12*J12;
    double x5 = J13*J13;
    double x6 = J13*J23;
    double x7 = 1.0/(-J11*x2 + J11*x3 - 2*J12*x6 + J22*x5 + J33*x4);
    double x8 = x7*(J12*J33 - x6);
    double x9 = x7*(-J12*J23 + J13*J22);
    double x10 = x7*(J11*J23 - J12*J13);

    A(0, 3) = 1;
    A(1, 4) = 1;
    A(2, 5) = 1;
    A(3, 7) = x1;
    A(4, 6) = -x1;
    A(6, 9) = 1;
    A(7, 10) = 1;
    A(8, 11) = 1;

    B(5, 0) = x0;
    B(9, 1) = x7*(-x2 + x3);
    B(9, 2) = x8;
    B(9, 3) = x9;
    B(10, 1) = x8;
    B(10, 2) = x7*(-J11*J33 + x5);
    B(10, 3) = x10;
    B(11, 1) = x9;
    B(11, 2) = x10;
    B(11, 3) = x7*(-J11*J22 + x4);
}

/**
 * Compute LQR control input u = -K * (x - x_ref)
 * Solves continuous-time algebraic Riccati equation (CARE):
 *   A^T P + P A - P B R^{-1} B^T P + Q = 0
 * via Hamiltonian matrix method. (Arimoto Potter)
 * https://zhuanlan.zhihu.com/p/1898278805430313007
 * https://doi.org/10.1137/0902010
 *
 * n = 12, m = 4
 * @param A       System matrix (n x n)
 * @param B       Input matrix (n x m)
 * @param Q       State cost matrix (n x n), dense positive semidefinite
 * @param R       Input cost matrix (m x m), dense positive definite
 * @param x       Current state (n x 1)
 * @param x_ref   Reference state (n x 1)
 * @return Control vector u (m x 1)
 */
/*
void lqr_control(const A_Type& A,
                const B_Type& B,
                const Eigen::Matrix<double, 12, 1>& Q_vec,
                const Eigen::Matrix<double, 4, 1>& R_vec,
                Eigen::Matrix<double, 4, 12>& K)
{
	int n = 12;          // state dimension
	int m = 4;          // input dimension

	// Build diagonal Q and R from the vectors
	A_Type Q = Q_vec.asDiagonal();
	// auto Q = Q_vec.asDiagonal();
	// Eigen::Matrix<double, 4, 4> R = R_vec.asDiagonal();
	auto R = R_vec.asDiagonal();

	// Compute R inverse (R is positive definite)
	auto R_inv = R.inverse();

	// Build Hamiltonian matrix H = [A, -B*R^{-1}*B'; -Q, -A']
	Eigen::Matrix<double, 24, 24> H;
	H.topLeftCorner(n, n) = A;
	H.topRightCorner(n, n) = -B * R_inv * B.transpose();
	H.bottomLeftCorner(n, n) = -Q;
	H.bottomRightCorner(n, n) = -A.transpose();

	// Compute eigenvalues and eigenvectors of H
	// TODO: deal with unsafe case
	Eigen::EigenSolver<Eigen::Matrix<double, 24, 24>> solver(H);
	if (solver.info() != Eigen::Success)
		throw std::runtime_error("Eigen decomposition failed for Hamiltonian matrix");

	Eigen::VectorXcd evals = solver.eigenvalues();
	Eigen::MatrixXcd evecs = solver.eigenvectors();

	// Select eigenvectors corresponding to eigenvalues with negative real part
	std::vector<int> stable_idx;
	for (int i = 0; i < 2*n; ++i) {
		if (evals(i).real() < 0.0)
			stable_idx.push_back(i);
	}
	if (stable_idx.size() != n)
		throw std::runtime_error("Number of stable eigenvalues != n; system may not be stabilizable/detectable");

	// Build U1 and U2 from the selected eigenvectors
	Eigen::MatrixXcd U1(n, n), U2(n, n);
	for (int j = 0; j < n; ++j) {
		int idx = stable_idx[j];
		U1.col(j) = evecs.col(idx).head(n);
		U2.col(j) = evecs.col(idx).tail(n);
	}

	// Compute P = U2 * U1^{-1} (real part)
	A_Type P = (U2 * U1.inverse()).real();

	// Compute gain K = R^{-1} * B^T * P
	K = R_inv * B.transpose() * P;

	// Control law
	// u = -K * (x - x_ref);
}
*/

// use the control method in "Minimum Snap Trajectory Generation and Control for Quadrotors"
void pd_control(const X_Type& x, const Eigen::Vector3d& pos_des, const Eigen::Vector3d& vel_des,
				const double g, const double mass, U_Type& u) {
	Eigen::VectorBlock<const X_Type, 3> pos = x.head<3>();
	Eigen::VectorBlock<const X_Type> vel = x.segment(3, 3);

    // Extract position and velocity errors
    Eigen::Vector3d pos_err = pos - pos_des;
    Eigen::Vector3d vel_err = vel - vel_des;

	// DEBUG
	// ROS_INFO("DEBUG: nominal: pd_control: pos_err = %f, %f, %f", pos_err(0), pos_err(1), pos_err(2));
	// ROS_INFO("DEBUG: nominal: pd_control: vel_err = %f, %f, %f", vel_err(0), vel_err(1), vel_err(2));

	Eigen::Vector3d acc_cmd = -(kp.asDiagonal() * pos_err) - (kd.asDiagonal() * vel_err);
	u = mass * acc_cmd;
	u.z() += mass * g;
}

void trajectoryGenerater(const X_Type& x, const X_Type x_goal, Eigen::Vector3d& pos_des, Eigen::Vector3d& vel_des, double yaw_des)
{
	Eigen::VectorBlock<const X_Type, 3> pos = x.head<3>(), ome = x.tail<3>(), pos_goal = x_goal.head<3>(), ome_goal = x_goal.tail<3>();
	Eigen::VectorBlock<const X_Type> vel = x.segment(3, 3), qua = x.segment(6, 4), vel_goal = x_goal.segment(3, 3), qua_goal = x_goal.segment(6, 4);
	Eigen::Vector3d dir = pos_goal - pos;

	double dt = 1/ ctrl_fre;

	if (dir.squaredNorm() > 1.0*1.0)
	{
		dir.normalize();
		if (vel.squaredNorm() < TRA_V_MAX*TRA_V_MAX)
		{
			pos_des = pos + (vel.norm() * dt + 0.5 * TRA_A * dt*dt) * dir;
			vel_des = (vel.norm() + TRA_A * dt) * dir;
			yaw_des = atan2(dir.y(), dir.x());
		}
		else
		{
			pos_des = pos + dt * TRA_V_MAX * dir;
			vel_des = TRA_V_MAX * dir;
			yaw_des = atan2(dir.y(), dir.x());
		}
	}
	else
	{
		pos_des = pos_goal;
		vel_des = vel_goal;
		yaw_des = atan2(2.0 * (qua_goal(1)*qua_goal(2) + qua_goal(0)*qua_goal(3)), 1.0 - 2.0 * (qua_goal(2)*qua_goal(2) + qua_goal(3)*qua_goal(3)));
	}
}

/**
 * 将单位四元数转换为旋转矩阵
 * @param q 四元数，存储顺序为 (w, x, y, z)，且模长为1
 * @param R 输出的 3x3 旋转矩阵
 */
void quaternionToRotationMatrix(const Eigen::Vector4d& q, Eigen::Matrix3d& R) {
    const double w = q(0);
    const double x = q(1);
    const double y = q(2);
    const double z = q(3);

    // 可选：检查输入四元数是否为单位四元数（调试用）
    // const double norm = std::sqrt(w*w + x*x + y*y + z*z);
    // assert(std::abs(norm - 1.0) < 1e-6);

    // 预计算常用项以提升效率
    const double xx = x * x;
    const double yy = y * y;
    const double zz = z * z;
    const double xy = x * y;
    const double xz = x * z;
    const double yz = y * z;
    const double wx = w * x;
    const double wy = w * y;
    const double wz = w * z;

    // 计算旋转矩阵元素
    R(0, 0) = 1.0 - 2.0 * (yy + zz);
    R(0, 1) = 2.0 * (xy - wz);
    R(0, 2) = 2.0 * (xz + wy);
    R(1, 0) = 2.0 * (xy + wz);
    R(1, 1) = 1.0 - 2.0 * (xx + zz);
    R(1, 2) = 2.0 * (yz - wx);
    R(2, 0) = 2.0 * (xz - wy);
    R(2, 1) = 2.0 * (yz + wx);
    R(2, 2) = 1.0 - 2.0 * (xx + yy);
}

// euler_angle: phi, theta, psi
void quaternionToEulerAngle(const Eigen::Vector4d& q, double& phi, double& theta, double& psi) {
	const double R20 = 2.0 * (q(1)*q(3) - q(0)*q(2)),
		R21 = 2.0 * (q(2)*q(3) + q(0)*q(1)), 
		R22 = 1.0 - 2.0 * (q(1)*q(1) + q(2)*q(2)), 
		R10 = 2.0 * (q(1)*q(2) + q(0)*q(3)), 
		R00 = 1.0 - 2.0 * (q(2)*q(2) + q(3)*q(3));
	theta = -asin(R20);
	phi = atan2(R21, R22);
	psi = atan2(R10, R00);
}
