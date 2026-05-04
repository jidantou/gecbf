#pragma once

#include <ros/node_handle.h>
#include <Eigen/Dense>
#include <string>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

namespace nominal_controller {

typedef Eigen::Matrix<double, 13, 12> A_Type;
typedef Eigen::Matrix<double, 13, 4> B_Type;
typedef Eigen::Matrix<double, 13, 1> X_Type;    // x,  y,  z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz
typedef Eigen::Vector3d U_Type;

struct PartStateType
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector4d qua;
    Eigen::Vector3d ome;
    PartStateType(const Eigen::Vector3d& p = Eigen::Vector3d::Zero(), const Eigen::Vector3d& v = Eigen::Vector3d::Zero(), const Eigen::Vector4d& q = Eigen::Vector4d::Zero(), const Eigen::Vector3d& o = Eigen::Vector3d::Zero()) {}
};

class PdController {
public:
	PdController(ros::NodeHandle& n, const int drone_id, const double ctrl_fre = 200.0);

	void setGains(const Eigen::Vector3d& kp, const Eigen::Vector3d& kd);

    void getControl(U_Type& u_ref) const;

    void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
    void task_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pos);

	const Eigen::Vector3d& kp() const;
	const Eigen::Vector3d& kd() const;

private:
    void trajectoryGenerater(Eigen::Vector3d& pos_des, Eigen::Vector3d& vel_des) const;

	Eigen::Vector3d kp_;
	Eigen::Vector3d kd_;

    PartStateType current_state_, goal_state_;
    ros::NodeHandle& nh_;

    ros::Subscriber task_sub_;
    ros::Subscriber odom_sub_;

    int drone_id_;
    double TRA_V_MAX_, TRA_A_;
    bool getGoal_;

	double g_;
	double mass_;

    double ctrl_fre_;
};

}  // namespace nominal_controller
