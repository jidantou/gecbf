#include "nominal_controller/pd_controller.h"

#include <ros/ros.h>

namespace nominal_controller {

PdController::PdController(ros::NodeHandle& n, const int drone_id, const double ctrl_fre):  nh_(n), drone_id_(drone_id), ctrl_fre_(ctrl_fre) {
    std::string param_prefix = "nominal_controller/";
    nh_.param(param_prefix + "kp_0", kp_(0), 5.7);
    nh_.param(param_prefix + "kp_1", kp_(1), 5.7);
    nh_.param(param_prefix + "kp_2", kp_(2), 6.2);
    nh_.param(param_prefix + "kd_0", kd_(0), 3.4);
    nh_.param(param_prefix + "kd_1", kd_(1), 3.4);
    nh_.param(param_prefix + "kd_2", kd_(2), 4.0);

	nh_.param(param_prefix + "trajectory_max_velocity", TRA_V_MAX_, 1.0);
	nh_.param(param_prefix + "trajectory_desired_accelerate", TRA_A_, 4.0);

	MIN_BRAKE_DIST_ = TRA_V_MAX_ * TRA_V_MAX_ / (2 * TRA_A_);
	// DEBUG
	// ROS_INFO("DEBUG: nominal: pd_control: MIN_BRAKE_DIST_ = %f", MIN_BRAKE_DIST_);

	task_sub_ = nh_.subscribe("/drone_" + std::to_string(drone_id_) + "_task_pose", 10, &PdController::task_pose_callback, this);
	odom_sub_ = nh_.subscribe("/drone_" + std::to_string(drone_id_) + "_odom", 50, &PdController::odom_callback, this);

    getGoal_ = false;
    
	g_ = 9.81;
	mass_ = 0.98;
}

void PdController::setGains(const Eigen::Vector3d& kp, const Eigen::Vector3d& kd) {
	kp_ = kp;
	kd_ = kd;
}

void PdController::getControl(U_Type& u_ref) const {
    if (!getGoal_)
    {
        u_ref.setZero();
        u_ref.z() += mass_ * g_;
        return;
    }
    
    Eigen::Vector3d pos_des, vel_des;
    trajectoryGenerater(pos_des, vel_des);
    // Extract position and velocity errors
    Eigen::Vector3d pos_err = current_state_.pos - pos_des;
    Eigen::Vector3d vel_err = current_state_.vel - vel_des;

	// DEBUG
	// ROS_INFO("DEBUG: nominal: pd_control: pos_err = %f, %f, %f", pos_err(0), pos_err(1), pos_err(2));
	// ROS_INFO("DEBUG: nominal: pd_control: vel_err = %f, %f, %f", vel_err(0), vel_err(1), vel_err(2));

	Eigen::Vector3d acc_cmd = -(kp_.asDiagonal() * pos_err) - (kd_.asDiagonal() * vel_err);
	u_ref = mass_ * acc_cmd;
	u_ref.z() += mass_ * g_;
}

void PdController::trajectoryGenerater(Eigen::Vector3d& pos_des, Eigen::Vector3d& vel_des) const {
    const Eigen::Vector3d &pos = current_state_.pos, &pos_goal = goal_state_.pos;
    const Eigen::Vector3d &vel = current_state_.vel, &vel_goal = goal_state_.vel;
    // Eigen::Vector4d &qua = current_state_.qua, &qua_goal = goal_state_.qua;
	Eigen::Vector3d dir = pos_goal - pos;

	double dt = 1 / ctrl_fre_;
	double dist = dir.norm();

	dir.normalize();

	if (dist < 0.05)										// close to the goal
	// if (dist < -1)										// close to the goal
	{
		pos_des = pos_goal;
		vel_des = vel_goal;
	} 
	// else if (dist < MIN_BRAKE_DIST_)						// need to brake
	else if (dist < 0.5)						// need to brake
	{
		double acc = std::min(TRA_A_, vel.squaredNorm() / (2 * dist));
		pos_des = pos + (vel.norm() * dt - 0.5 * acc * dt*dt) * dir;
		vel_des = (vel.norm() - acc * dt) * dir;
	} 
	else if (vel.squaredNorm() < TRA_V_MAX_*TRA_V_MAX_)	// can accelerate
	{
		pos_des = pos + (vel.norm() * dt + 0.5 * TRA_A_ * dt*dt) * dir;
		vel_des = (vel.norm() + TRA_A_ * dt) * dir;
		// yaw_des = atan2(dir.y(), dir.x());
	}
	else												// cruise at max velocity
	{
		pos_des = pos + dt * TRA_V_MAX_ * dir;
		vel_des = TRA_V_MAX_ * dir;
		// yaw_des = atan2(dir.y(), dir.x());
	}

}

const Eigen::Vector3d& PdController::kp() const {
	return kp_;
}

const Eigen::Vector3d& PdController::kd() const {
	return kd_;
}

void PdController::odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
	
    current_state_.pos(0) = odom->pose.pose.position.x;
	current_state_.pos(1) = odom->pose.pose.position.y;
	current_state_.pos(2) = odom->pose.pose.position.z;
	current_state_.vel(0) = odom->twist.twist.linear.x;
	current_state_.vel(1) = odom->twist.twist.linear.y;
	current_state_.vel(2) = odom->twist.twist.linear.z;
	current_state_.qua(0) = odom->pose.pose.orientation.w;
	current_state_.qua(1) = odom->pose.pose.orientation.x;
	current_state_.qua(2) = odom->pose.pose.orientation.y;
	current_state_.qua(3) = odom->pose.pose.orientation.z;
	// current_qua(0) = odom->pose.pose.orientation.w;
	// current_qua(1) = odom->pose.pose.orientation.x;
	// current_qua(2) = odom->pose.pose.orientation.y;
	// current_qua(3) = odom->pose.pose.orientation.z;
	// quaternionToEulerAngle(current_qua, current_state_(6), current_state_(7), current_state_(8));

	current_state_.ome(0) = odom->twist.twist.angular.x;
	current_state_.ome(1) = odom->twist.twist.angular.y;
	current_state_.ome(2) = odom->twist.twist.angular.z;
}

void PdController::task_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
	goal_state_.pos(0) = pos->pose.position.x;
	goal_state_.pos(1) = pos->pose.position.y;
	goal_state_.pos(2) = pos->pose.position.z;
	goal_state_.vel(0) = 0;
	goal_state_.vel(1) = 0;
	goal_state_.vel(2) = 0;
	goal_state_.qua(0) = pos->pose.orientation.w;
	goal_state_.qua(1) = pos->pose.orientation.x;
	goal_state_.qua(2) = pos->pose.orientation.y;
	goal_state_.qua(3) = pos->pose.orientation.z;
	// current_qua(0) = pos->pose.orientation.w;
	// current_qua(1) = pos->pose.orientation.x;
	// current_qua(2) = pos->pose.orientation.y;
	// current_qua(3) = pos->pose.orientation.z;
	// quaternionToEulerAngle(current_qua, goal_state(6), goal_state(7), goal_state(8));

	goal_state_.ome(0) = 0;
	goal_state_.ome(1) = 0;
	goal_state_.ome(2) = 0;

	getGoal_ = true;
}

}  // namespace nominal_controller
