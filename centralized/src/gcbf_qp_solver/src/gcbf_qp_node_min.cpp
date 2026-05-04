#include <ros/ros.h>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/Force3.h>
#include <Eigen/Eigen>
#include <OsqpEigen/OsqpEigen.h>
#include "gcbf_qp_solver/gcbf_min.hpp"
#include "nominal_controller/pd_controller.h"

// DEBUG
#include <sstream>

// DEBUG
static int count_debug = 0;

// std::unique_ptr<Eigen::Vector3d[]> ref_ctrls_ptr, control_cmds_ptr;

// void ref_ctrl_callback(const quadrotor_msgs::Force3::ConstPtr& ref_ctrl)
// {
//   // TODO: safe check
//   int id = std::stoi(ref_ctrl->header.frame_id.substr(7));

//   ref_ctrls_ptr[id](0) = ref_ctrl->force.x;
//   ref_ctrls_ptr[id](1) = ref_ctrl->force.y;
//   ref_ctrls_ptr[id](2) = ref_ctrl->force.z;
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gcbf_qp_node");
  ros::NodeHandle nh("~");

  int drone_num = 1, NUM_VAR, NUM_CONSTRAINT;
  double obstacle_distances, drone_distance, sensing_horizon;

  nh.param("drone_num", drone_num, 1);
  nh.param("obstacle_distances", obstacle_distances, 0.05);
  nh.param("drone_distance", drone_distance, 0.1);
  nh.param("sensing_horizon", sensing_horizon, 5.0);

  GCBF_QP_MIN::Gcbf gcbf_solution(nh, drone_num, sensing_horizon, obstacle_distances, drone_distance);

  // DEBUG
  gcbf_solution.debug_count = 0;
  
  std::unique_ptr<ros::Subscriber[]> ref_ctrl_subs(new ros::Subscriber[drone_num]), odom_subs(new ros::Subscriber[drone_num]);
  std::unique_ptr<ros::Publisher[]> control_cmd_pubs(new ros::Publisher[drone_num]);

  // ref_ctrls_ptr = std::make_unique<Eigen::Vector3d[]>(drone_num);
  // control_cmds_ptr = std::make_unique<Eigen::Vector3d[]>(drone_num);

  // initialize
  // for (int i = 0; i < drone_num; i++)
  // {
  //   ref_ctrls_ptr[i] << 0.0, 0.0, 0.98 * 9.81;
  // }

  for (int i = 0; i < drone_num; i++)
  {
    odom_subs[i] = nh.subscribe("/drone_" + std::to_string(i) + "_odom", 100, &GCBF_QP_MIN::Gcbf::odom_callback, &gcbf_solution);
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
  NUM_CONSTRAINT = drone_num;

  Eigen::VectorXd q(NUM_VAR);
  Eigen::SparseMatrix<double> P(NUM_VAR, NUM_VAR), A(NUM_CONSTRAINT, NUM_VAR);
  // Eigen::MatrixXd P(NUM_VAR, NUM_VAR), A(NUM_CONSTRAINT, NUM_VAR);
  Eigen::VectorXd l(NUM_CONSTRAINT), u(NUM_CONSTRAINT);

  P.setIdentity();
  q.setZero();
  u.setConstant(OsqpEigen::INFTY);
  l.setZero();

  OsqpEigen::Solver solver;
  solver.settings()->setWarmStart(true);
  // solver.settings()->setAbsoluteTolerance(1e-3);   // relax tolerance
  // solver.settings()->setRelativeTolerance(1e-3);
  // solver.settings()->setMaxIteration(200);         // limit iterations
  solver.settings()->setVerbosity(false);          // suppress output

  solver.data()->setNumberOfVariables(NUM_VAR);
  solver.data()->setNumberOfConstraints(NUM_CONSTRAINT);
  solver.data()->setHessianMatrix(P);
  solver.data()->setGradient(q);
  solver.data()->setLinearConstraintsMatrix(A);
  solver.data()->setLowerBound(l);
  solver.data()->setUpperBound(u);

  if (!solver.initSolver()) {
    ROS_ERROR("Failed to initialize OSQP solver");
    return -1;
  }

  while (ros::ok())
  {
    ros::spinOnce();

    for (int i = 0; i < drone_num; i++)
    {
      Eigen::Vector3d ref_ctrl;
      pd_controllers[i].getControl(ref_ctrl);
      q.segment(i * 3, 3) = -ref_ctrl;
    }
    gcbf_solution.computeQPCoefficientsAl(A, l);

    // DEBUG
    if (count_debug % 50 == 0)
    {
    //   ROS_INFO("%d:  q: [%.3f,%.3f,%.3f]  A: [%.3f,%.3f,%.3f]  l: [%.3f]", count_debug,
    //     q(0), q(1), q(2), A.coeff(0,0), A.coeff(0,1), A.coeff(0,2), l(0));
        std::stringstream ss;
        ss << std::scientific << std::setprecision(3);

        // 第一行：输出 q 向量
        ss << count_debug << ":  q: [";
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

    solver.updateGradient(q);
    solver.updateLinearConstraintsMatrix(A);
    solver.updateLowerBound(l);

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
  gcbf_solution.closeLogFile();   // 在此处关闭文件
  return 0;
  
}
