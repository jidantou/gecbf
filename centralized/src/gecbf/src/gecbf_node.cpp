#include <ros/ros.h>
#include <memory>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/Force3.h>
#include <Eigen/Eigen>
#include <OsqpEigen/OsqpEigen.h>
#include "gecbf/gecbf.hpp"
#include "nominal_controller/pd_controller.h"

// DEBUG
#include <sstream>

// DEBUG
static int count_debug = 0;

void correction(ros::NodeHandle& nh, const int drone_num, Eigen::VectorXd& solution, const std::vector<Eigen::Vector3d>& ref_ctrls, const Eigen::SparseMatrix<double>& A, const Eigen::VectorXd& l);

// DEBUG
std::vector<GECBF_QP::PartStateType> states;
void node_odom_callback(const nav_msgs::Odometry::ConstPtr& odom) {
    // TODO: safe check
    int id = std::stoi(odom->child_frame_id.substr(7));

    states[id].pos(0) = odom->pose.pose.position.x;
    states[id].pos(1) = odom->pose.pose.position.y;
    states[id].pos(2) = odom->pose.pose.position.z;
    states[id].vel(0) = odom->twist.twist.linear.x;
    states[id].vel(1) = odom->twist.twist.linear.y;
    states[id].vel(2) = odom->twist.twist.linear.z;
    states[id].qua(0) = odom->pose.pose.orientation.w;
    states[id].qua(1) = odom->pose.pose.orientation.x;
    states[id].qua(2) = odom->pose.pose.orientation.y;
    states[id].qua(3) = odom->pose.pose.orientation.z;
    states[id].ome(0) = odom->twist.twist.angular.x;
    states[id].ome(1) = odom->twist.twist.angular.y;
    states[id].ome(2) = odom->twist.twist.angular.z;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gecbf_qp_node");
    ros::NodeHandle nh("~");

    int drone_num = 1, NUM_VAR, NUM_CONSTRAINT;
    double obstacle_distances, drone_distance, sensing_horizon;

    nh.param("drone_num", drone_num, 1);
    nh.param("obstacle_distances", obstacle_distances, 0.05);
    nh.param("drone_distance", drone_distance, 0.1);
    nh.param("sensing_horizon", sensing_horizon, 5.0);

    GECBF_QP::Gecbf gecbf_solution(nh, drone_num, sensing_horizon, obstacle_distances, drone_distance);

    // DEBUG
    gecbf_solution.debug_count = 0;

    std::vector<ros::Subscriber> odom_subs(drone_num);
    std::vector<ros::Publisher> control_cmd_pubs(drone_num);
    std::vector<Eigen::Vector3d> ref_ctrls(drone_num);

    for (int i = 0; i < drone_num; i++) {
        odom_subs[i] = nh.subscribe("/drone_" + std::to_string(i) + "_odom", 100, &GECBF_QP::Gecbf::odom_callback, &gecbf_solution);
        control_cmd_pubs[i] = nh.advertise<quadrotor_msgs::Force3>("/drone_" + std::to_string(i) + "_control_cmd", 100);
    }

    // DEBUG
    states.resize(drone_num);
    std::vector<ros::Subscriber> debug_odom_subs(drone_num);
    for (int i = 0; i < drone_num; i++) {
        debug_odom_subs[i] = nh.subscribe("/drone_" + std::to_string(i) + "_odom", 100, &node_odom_callback);
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
    int num_constraints;

    Eigen::VectorXd q(NUM_VAR);
    Eigen::SparseMatrix<double> P(NUM_VAR, NUM_VAR), A;
    Eigen::VectorXd l, u;

    P.setIdentity();

    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    // solver.settings()->setAbsoluteTolerance(1e-3);   // relax tolerance
    // solver.settings()->setRelativeTolerance(1e-3);
    // solver.settings()->setMaxIteration(200);         // limit iterations
    solver.settings()->setVerbosity(false);          // suppress output

    solver.data()->setNumberOfVariables(NUM_VAR);
    solver.data()->setHessianMatrix(P);

    while (ros::ok()) {
        ros::spinOnce();

		solver.clearSolver();
		solver.data()->clearLinearConstraintsMatrix();

        for (int i = 0; i < drone_num; i++) {
            pd_controllers[i].getControl(ref_ctrls[i]);
            q.segment(i * 3, 3) = -ref_ctrls[i];
        }
        num_constraints = gecbf_solution.computeQPCoefficientsAl(A, l);

        if (num_constraints == 0) {
            // 没有约束，直接发布参考控制命令

            // DEBUG
            if (count_debug % 100 == 0) {
                ROS_INFO("%d:  no constraints, ref_ctrl: [%.3f,%.3f,%.3f]", count_debug, ref_ctrls[0](0), ref_ctrls[0](1), ref_ctrls[0](2));
            }

            for (int i = 0; i < drone_num; i++) {
                msg.header.frame_id = "/drone_" + std::to_string(i);
                msg.header.stamp = ros::Time::now();
                msg.force.x = ref_ctrls[i](0);
                msg.force.y = ref_ctrls[i](1);
                msg.force.z = ref_ctrls[i](2);

                control_cmd_pubs[i].publish(msg);
            }
            control_rate.sleep();
            // DEBUG
            count_debug++;
            gecbf_solution.debug_count++;
            continue;
        }

        u = Eigen::VectorXd::Constant(num_constraints, OsqpEigen::INFTY);

        // DEBUG
        if (count_debug % 100 == 0) {
            //   ROS_INFO("%d:  q: [%.3f,%.3f,%.3f]  A: [%.3f,%.3f,%.3f]  l: [%.3f]", count_debug,
            //     q(0), q(1), q(2), A.coeff(0,0), A.coeff(0,1), A.coeff(0,2), l(0));
            std::stringstream ss;
            ss << std::scientific << std::setprecision(3);

            // 第一行：输出 q 向量
            ss << count_debug << ":  num_cons: " << num_constraints << ", q: [";
            for (int i = 0; i < q.size(); ++i) {
                ss << q(i) << (i < q.size() - 1 ? ", " : "");
            }
            ss << "]\n";

            // 输出 "A = "（不换行）
            ss << "A = ";
            int rows = A.rows();
            int cols = A.cols();
            for (int i = 0; i < rows; ++i) {
                // 非第一行先换行并输出对齐空格（使 '[' 与第一行的 '[' 对齐）
                if (i > 0) {
                    ss << "\n     ";   // 5个空格，与 "A = " 后的第一个空格对齐
                }
                // 输出矩阵的第 i 行
                ss << " [ ";
                for (int j = 0; j < cols; ++j) {
                    ss << A.coeff(i, j);
                    if (j < cols - 1) ss << "  ";
                }
                ss << " ]";
                // 输出 l 的对应元素（右侧列向量形式）
                if (i == 0) {
                    ss << "   l = [ " << l(i);
                } else if (i == rows - 1) {
                    ss << "        " << l(i) << " ]";
                } else {
                    ss << "        " << l(i);
                }
            }

            ROS_INFO_STREAM(ss.str());
        }
        // ROS_INFO("    drone_num:%d, msg:0x%p, control_cmd_pubs:0x%p", drone_num, &msg, &control_cmd_pubs[0]);

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
				msg.force.x = ref_ctrls[i](0);
				msg.force.y = ref_ctrls[i](1);
				msg.force.z = ref_ctrls[i](2);

				control_cmd_pubs[i].publish(msg);
			}
			control_rate.sleep();
			// DEBUG
			count_debug++;
			gecbf_solution.debug_count++;
			continue;
		}

        if (solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError) {
            // DEBUG
            // ROS_INFO("DEBUG: gecbf_qp_node: solve QP success!");

            Eigen::VectorXd solution = solver.getSolution();

            // DEBUG
            if (count_debug % 100 == 0) {
                // ROS_INFO("%d:  solution: [%.3f,%.3f,%.3f]", count_debug, solution(0), solution(1), solution(2));
                std::stringstream ss;
                ss << std::scientific << std::setprecision(3);

                // 第一行：输出 q 向量
                ss << count_debug << ":  solution: [";
                for (int i = 0; i < solution.size(); ++i) {
                    ss << solution(i) << (i < solution.size() - 1 ? ", " : "");
                }
                ss << "]";
                ROS_INFO_STREAM(ss.str());

                std::stringstream ss_2;

                ss_2 << count_debug << ":  cos of f and f_des: [";
                ss_2 << std::scientific << std::setprecision(3);
                for (int i = 0; i < drone_num; i++) {
                    Eigen::Vector3d f = solution.segment(i * 3, 3), f_des = ref_ctrls[i];
                    f.z() -= 9.81 * 0.98;
                    f_des.z() -= 9.81 * 0.98;
                    double value = f.dot(f_des);
                    double cos_value = value / (f.norm() * f_des.norm());
                    ss_2 << cos_value << (i < drone_num - 1 ? ", " : "");
                }
                ss_2 << "]\n";
                ROS_INFO_STREAM(ss_2.str());
            }
            
            // DEBUG
            // for (int i = 0; i < drone_num; i++) {
            //     Eigen::Vector3d f = solution.segment(i * 3, 3), f_des = ref_ctrls[i];
            //     f.z() -= 9.81 * 0.98;
            //     f_des.z() -= 9.81 * 0.98;
            //     double value = f.dot(f_des);
            //     double cos_value = value / (f.norm() * f_des.norm());
            //     if (cos_value < -0.95 && states[i].vel.norm() < 0.01) {
            //         ROS_INFO("%d:  Drone %d slow down and stop, cos of f and f_des: %.3f", count_debug, i, cos_value);
            //     }
            // }
            
            correction(nh, drone_num, solution, ref_ctrls, A, l);

            for (int i = 0; i < drone_num; i++) {
                msg.header.frame_id = "/drone_" + std::to_string(i);
                msg.header.stamp = ros::Time::now();
                msg.force.x = solution(3 * i + 0);
                msg.force.y = solution(3 * i + 1);
                msg.force.z = solution(3 * i + 2);

                control_cmd_pubs[i].publish(msg);
            }
            gecbf_solution.setCtrls(solution);
        } else {
            // DEBUG
            // ROS_INFO("DEBUG: gecbf_qp_node: solve QP failed!");

            ROS_WARN("OSQP solve failed, use ref ctrl");

            // publish reference command
            for (int i = 0; i < drone_num; i++) {
                msg.header.frame_id = "/drone_" + std::to_string(i);
                msg.header.stamp = ros::Time::now();
                // msg.force.x = ref_ctrls_ptr[i](0);
                // msg.force.y = ref_ctrls_ptr[i](1);
                // msg.force.z = ref_ctrls_ptr[i](2);
                msg.force.x = ref_ctrls[i](0);
                msg.force.y = ref_ctrls[i](1);
                msg.force.z = ref_ctrls[i](2);

                control_cmd_pubs[i].publish(msg);
            }
            gecbf_solution.setCtrls(ref_ctrls);
        }

        control_rate.sleep();
        // DEBUG
        count_debug++;
        gecbf_solution.debug_count++;
    }
    gecbf_solution.closeLogFile();   // 在此处关闭文件
    return 0;

}

void correction(ros::NodeHandle& nh, const int drone_num, Eigen::VectorXd& solution, const std::vector<Eigen::Vector3d>& ref_ctrls, const Eigen::SparseMatrix<double>& A, const Eigen::VectorXd& l) {
    double cos_threshold = -0.95;  // 设定一个阈值，判断是否需要修正
    double vel_threshold = 0.01;  // 设定一个速度阈值，判断无人机是否接近静止
    double gravity_compensation = 9.81 * 0.98;  // 重力补偿值
    double correction_force_magnitude = 0.1;  // 修正力的大小，可以根据需要调整

    nh.param("correction/cos_threshold", cos_threshold, -0.95);
    nh.param("correction/vel_threshold", vel_threshold, 0.01);
    nh.param("correction/correction_force_magnitude", correction_force_magnitude, 0.1);

    Eigen::VectorXd solution_new = solution;
    for (int i = 0; i < ref_ctrls.size(); i++) {
        Eigen::Vector3d f = solution.segment(i * 3, 3), f_ref = ref_ctrls[i];
        f.z() -= 9.81 * 0.98;
        f_ref.z() -= 9.81 * 0.98;
        f.normalize();
        f_ref.normalize();

        double cos_value = f.dot(f_ref);

        if (cos_value < cos_threshold && states[i].vel.norm() < vel_threshold) {
            Eigen::Vector3d x_e = - f_ref.z() * f_ref + Eigen::Vector3d(0, 0, 1);
            x_e.normalize();
            
            Eigen::Vector3d y_e = f_ref.cross(x_e);
            y_e.normalize();

            double corr_coeff = correction_force_magnitude;
            
            bool all_satisfied = true;
            for (int iter = 0; iter < 50; iter++) {  // 限制迭代次数
                double theta = i * (2 * M_PI / drone_num);
                Eigen::Vector3d f_e;

                for (int theta_iter = 0; theta_iter < 10; theta_iter++) {
                    all_satisfied = true;

                    theta = i * (2 * M_PI / drone_num) + theta_iter * ((2 * M_PI / drone_num) / 10);
                    f_e = std::cos(theta) * x_e + std::sin(theta) * y_e;
                    f_e.normalize();

                    solution_new.segment(i * 3, 3) += corr_coeff * f_e;

                    // varify constraint satisfaction
                    Eigen::VectorXd constraint_values = A * solution_new - l;
                    
                    for (int j = 0; j < constraint_values.size(); j++) {
                        if (constraint_values(j) < -1e-3) {
                            all_satisfied = false;
                            break;
                        }
                    }
                    if (all_satisfied) {
                        break;  // 如果所有约束都满足，退出循环
                    }
                    
                    solution_new.segment(i * 3, 3) -= corr_coeff * f_e;
                }

                if (all_satisfied)
                    break;

                corr_coeff *= 0.9;  // 如果不满足，逐渐减小修正力度
            }

            // DEBUG
            // if (iter >= 50) {
            //      ROS_INFO("%d:  Drone %d correction failed, could not find a solution that satisfies constraints after 50 iterations.", count_debug, i);
            // } else {
            //     ROS_INFO("%d:  Drone %d slow down and stop, correction force: [%.3f, %.3f, %.3f]", count_debug, i, solution_new(3 * i + 0) - solution(3 * i + 0), solution_new(3 * i + 1) - solution(3 * i + 1), solution_new(3 * i + 2) - solution(3 * i + 2));
            // }

        }
    }
    solution = solution_new;
}
