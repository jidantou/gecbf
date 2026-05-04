#include <ros/ros.h>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/Force3.h>
#include <Eigen/Eigen>
#include <OsqpEigen/OsqpEigen.h>
#include "gcbf_qp_solver/gcbf.hpp"

#include "nominal_controller/pd_controller.h"

#include <sstream>
#include <iomanip>

// DEBUG
static int count_debug = 0;

// std::unique_ptr<Eigen::Vector3d[]> ref_ctrls_ptr, control_cmds_ptr;

// void ref_ctrl_callback(const quadrotor_msgs::Force3::ConstPtr& ref_ctrl)
// {
// 	// TODO: safe check
// 	int id = std::stoi(ref_ctrl->header.frame_id.substr(7));

// 	ref_ctrls_ptr[id](0) = ref_ctrl->force.x;
// 	ref_ctrls_ptr[id](1) = ref_ctrl->force.y;
// 	ref_ctrls_ptr[id](2) = ref_ctrl->force.z;
// }

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gcbf_qp_node");
	ros::NodeHandle nh("~");

	int drone_num = 1, NUM_VAR;		// NUM_CONSTRAINT;
	double obstacle_distances, drone_distance, sensing_horizon;

	nh.param("drone_num", drone_num, 1);
	nh.param("obstacle_distances", obstacle_distances, 0.05);
	nh.param("drone_distance", drone_distance, 0.1);
	nh.param("sensing_horizon", sensing_horizon, 5.0);

	GCBF_QP::Gcbf gcbf_solution(nh, drone_num, sensing_horizon, obstacle_distances, drone_distance);

	// DEBUG
	gcbf_solution.debug_count = 0;
	
	std::unique_ptr<ros::Subscriber[]> ref_ctrl_subs(new ros::Subscriber[drone_num]), odom_subs(new ros::Subscriber[drone_num]);
	std::unique_ptr<ros::Publisher[]> control_cmd_pubs(new ros::Publisher[drone_num]);

	// ref_ctrls_ptr = std::make_unique<Eigen::Vector3d[]>(drone_num);
	// control_cmds_ptr = std::make_unique<Eigen::Vector3d[]>(drone_num);

	// // initialize
	// for (int i = 0; i < drone_num; i++)
	// {
	// 	ref_ctrls_ptr[i] << 0.0, 0.0, 0.98 * 9.81;
	// }

	for (int i = 0; i < drone_num; i++)
	{
		odom_subs[i] = nh.subscribe("/drone_" + std::to_string(i) + "_odom", 100, &GCBF_QP::Gcbf::odom_callback, &gcbf_solution);
		// ref_ctrl_subs[i] = nh.subscribe("/drone_" + std::to_string(i) + "_ref_control", 100, ref_ctrl_callback);
		control_cmd_pubs[i] = nh.advertise<quadrotor_msgs::Force3>("/drone_" + std::to_string(i) + "_control_cmd", 100);
	}

	double ctrl_fre;
	nh.param("control_frequence", ctrl_fre, 200.0);
	ros::Rate control_rate(ctrl_fre);

	quadrotor_msgs::Force3 msg;

	std::vector<nominal_controller::PdController> pd_controllers;
	pd_controllers.reserve(drone_num);
	for (int i = 0; i < drone_num; i++)
		pd_controllers.emplace_back(nh, i, ctrl_fre);

	// TODO: try qpOASES
	// OSQP solver
	NUM_VAR = 3 * drone_num;
	// NUM_CONSTRAINT = drone_num;
	int num_constraints;
	
	Eigen::VectorXd q(NUM_VAR);
	// Eigen::SparseMatrix<double> P(NUM_VAR, NUM_VAR), A(NUM_CONSTRAINT, NUM_VAR);
	Eigen::SparseMatrix<double> P(NUM_VAR, NUM_VAR), A;
	// Eigen::MatrixXd P(NUM_VAR, NUM_VAR), A(NUM_CONSTRAINT, NUM_VAR);
	// Eigen::VectorXd l(NUM_CONSTRAINT), u(NUM_CONSTRAINT);
	Eigen::VectorXd l, u;

	P.setIdentity();
	// u.setConstant(OsqpEigen::INFTY);

	OsqpEigen::Solver solver;
	solver.settings()->setWarmStart(true);
	solver.settings()->setAbsoluteTolerance(1e-3);   // relax tolerance
	solver.settings()->setRelativeTolerance(1e-3);
	solver.settings()->setMaxIteration(200);         // limit iterations
	solver.settings()->setVerbosity(false);          // suppress output

	solver.data()->setNumberOfVariables(NUM_VAR);
	// solver.data()->setNumberOfConstraints(NUM_CONSTRAINT);
	solver.data()->setHessianMatrix(P);
	// solver.data()->setGradient(q);
	// solver.data()->setLinearConstraintsMatrix(A);
	// solver.data()->setLowerBound(l);
	// solver.data()->setUpperBound(u);

	// if (!solver.initSolver()) {
	// 	ROS_ERROR("Failed to initialize OSQP solver");
	// 	return -1;
	// }

	while (ros::ok())
	{
		ros::spinOnce();

		solver.clearSolver();
		solver.data()->clearLinearConstraintsMatrix();

		for (int i = 0; i < drone_num; i++)
		{
			Eigen::Vector3d ref_ctrl;
      		pd_controllers[i].getControl(ref_ctrl);
			q.segment(i * 3, 3) = -ref_ctrl;
		}
		// 获取当前时刻的约束矩阵 A 和 l，以及约束个数
		num_constraints = gcbf_solution.computeQPCoefficientsAl(A, l);
		u = Eigen::VectorXd::Constant(num_constraints, OsqpEigen::INFTY);
		
		// DEBUG
		// if (count_debug % 50 == 0)
		// {
		// 	ROS_INFO("%d:  num_con:%d  q: [%.3f,%.3f,%.3f,%.3f]  A: [%.3f,%.3f,%.3f,%.3f]  l: [%.3f]", count_debug, num_constraints,
		// 		q(0), q(1), q(2), q(3), A.coeff(0,0), A.coeff(0,1), A.coeff(0,2), A.coeff(0,3), l(0));
		// }
		// DEBUG
		if (count_debug % 50 == 0)
		{
			ROS_INFO("%d:  num_con:%d  q: [%.3f,%.3f,%.3f]", count_debug, num_constraints,
					q(0), q(1), q(2));
			// 打印 A 和 l 的所有值
			for (int i = 0; i < num_constraints; ++i)
			{
				std::stringstream row_ss;
				row_ss << std::fixed << std::setprecision(2);  // 设置全局格式
				row_ss << "    A[" << i << "]: [";
				for (int j = 0; j < NUM_VAR; ++j)
				{
					row_ss << std::setw(5) << A.coeff(i, j);  // 宽度10，右对齐
					if (j < NUM_VAR - 1) row_ss << ", ";
				}
				row_ss << "]  l[" << i << "]: " << std::setw(10) << l(i);
				ROS_INFO("%s", row_ss.str().c_str());
			}
		}
		// ROS_INFO("    drone_num:%d, msg:0x%p, control_cmd_pubs:0x%p", drone_num, &msg, &control_cmd_pubs[0]);

		// solver.updateGradient(q);
		// solver.updateLinearConstraintsMatrix(A);
		// solver.updateLowerBound(l);

		solver.data()->setNumberOfConstraints(num_constraints);
		// solver.data()->setHessianMatrix(P);
		solver.data()->setGradient(q);
		solver.data()->setLinearConstraintsMatrix(A);
		solver.data()->setLowerBound(l);
		solver.data()->setUpperBound(u);

		if (!solver.initSolver()) {
			ROS_ERROR("Failed to initialize OSQP solver");
			// publish reference command
			for (int i = 0; i < drone_num; i++)
			{
				msg.header.frame_id = "/drone_" + std::to_string(i);
				msg.header.stamp = ros::Time::now();
				// msg.force.x = ref_ctrls_ptr[i](0);
				// msg.force.y = ref_ctrls_ptr[i](1);
				// msg.force.z = ref_ctrls_ptr[i](2);
				msg.force.x = -q(i * 3 +0);
				msg.force.y = -q(i * 3 +1);
				msg.force.z = -q(i * 3 +2);

				control_cmd_pubs[i].publish(msg);
			}
			control_rate.sleep();
			// DEBUG
			count_debug++;
			gcbf_solution.debug_count++;
			continue;
		}

		if (solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError) {
			// DEBUG
			// ROS_INFO("DEBUG: gcbf_qp_node: solve QP success!");

			Eigen::VectorXd solution = solver.getSolution();

			// DEBUG
			if (count_debug % 50 == 0)
			{
				ROS_INFO("%d:  solution: [%.3f,%.3f,%.3f]", count_debug, solution(0), solution(1), solution(2));
			}
			// ROS_INFO("    drone_num:%d, msg:0x%p, control_cmd_pubs:0x%p", drone_num, &msg, &control_cmd_pubs[0]);

			for (int i = 0; i < drone_num; i++)
			{
				msg.header.frame_id = "/drone_" + std::to_string(i);
				msg.header.stamp = ros::Time::now();
				msg.force.x = solution(3 * i + 0);
				msg.force.y = solution(3 * i + 1);
				msg.force.z = solution(3 * i + 2);

				control_cmd_pubs[i].publish(msg);
			}
		} else {
			// DEBUG
			ROS_INFO("DEBUG: gcbf_qp_node: solve QP failed!");

			ROS_WARN("OSQP solve failed, use ref ctrl");

			// publish reference command
			for (int i = 0; i < drone_num; i++)
			{
				msg.header.frame_id = "/drone_" + std::to_string(i);
				msg.header.stamp = ros::Time::now();
				// msg.force.x = ref_ctrls_ptr[i](0);
				// msg.force.y = ref_ctrls_ptr[i](1);
				// msg.force.z = ref_ctrls_ptr[i](2);
				msg.force.x = -q(i * 3 +0);
				msg.force.y = -q(i * 3 +1);
				msg.force.z = -q(i * 3 +2);

				control_cmd_pubs[i].publish(msg);
			}
		}
		
		control_rate.sleep();
		// DEBUG
		count_debug++;
		gcbf_solution.debug_count++;
	}
	gcbf_solution.closeLogFile();
	return 0;
}
