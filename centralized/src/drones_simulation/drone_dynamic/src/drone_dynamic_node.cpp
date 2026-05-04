#include <Eigen/Geometry>
#include <drone_dynamic/Quadrotor.hpp>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include "quadrotor_msgs/Force3.h"

#define DISTURBANCE_ON

#ifdef DISTURBANCE_ON
typedef struct _Disturbance
{
  Eigen::Vector3d f;
  Eigen::Vector3d m;
} Disturbance;
#endif

static Eigen::Vector3d command_force;
#ifdef DISTURBANCE_ON
static Disturbance disturbance;
#endif

void stateToOdomMsg(const QuadrotorSimulator::Quadrotor::State& state,
                    nav_msgs::Odometry&                         odom);
void quadToImuMsg(const QuadrotorSimulator::Quadrotor& quad,
                  sensor_msgs::Imu&                    imu);

static void
cmd_callback(const quadrotor_msgs::Force3::ConstPtr& cmd)
{
  command_force(0) = cmd->force.x;
  command_force(1) = cmd->force.y;
  command_force(2) = cmd->force.z;
}

#ifdef DISTURBANCE_ON
static void
force_disturbance_callback(const geometry_msgs::Vector3::ConstPtr& f)
{
  disturbance.f(0) = f->x;
  disturbance.f(1) = f->y;
  disturbance.f(2) = f->z;
}

static void
moment_disturbance_callback(const geometry_msgs::Vector3::ConstPtr& m)
{
  disturbance.m(0) = m->x;
  disturbance.m(1) = m->y;
  disturbance.m(2) = m->z;
}
#endif

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_dynamic_node");

  ros::NodeHandle n("~");

  ros::Publisher  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
  ros::Publisher  imu_pub  = n.advertise<sensor_msgs::Imu>("imu", 10);
  ros::Subscriber cmd_sub =
    n.subscribe("cmd", 100, &cmd_callback, ros::TransportHints().tcpNoDelay());

  #ifdef DISTURBANCE_ON
  ros::Subscriber f_sub =
    n.subscribe("force_disturbance", 100, &force_disturbance_callback,
                ros::TransportHints().tcpNoDelay());
  ros::Subscriber m_sub =
    n.subscribe("moment_disturbance", 100, &moment_disturbance_callback,
                ros::TransportHints().tcpNoDelay());
  #endif

  QuadrotorSimulator::Quadrotor quad;
  double                        _init_x, _init_y, _init_z;
  n.param("simulator/init_state_x", _init_x, 0.0);
  n.param("simulator/init_state_y", _init_y, 0.0);
  n.param("simulator/init_state_z", _init_z, 1.0);

  Eigen::Vector3d position = Eigen::Vector3d(_init_x, _init_y, _init_z);
  quad.setStatePos(position);
  command_force.setZero();
  command_force(2) = quad.getMass() * quad.getGravity();

  double max_force;
  n.param("simulator/max_force", max_force, 1000.0);
  quad.setMaxForce(max_force);
  // DEBUG
  // quad.setMinRPM(0);

  double simulation_rate;
  n.param("rate/simulation", simulation_rate, 1000.0);
  ROS_ASSERT(simulation_rate > 0);

  double odom_rate;
  n.param("rate/odom", odom_rate, 100.0);
  const ros::Duration odom_pub_duration(1 / odom_rate);

  // std::string quad_name;
  // n.param("quadrotor_name", quad_name, std::string("quadrotor"));
  int drone_id;
  n.param("drone_id", drone_id, -1);

  QuadrotorSimulator::Quadrotor::State state = quad.getState();

  ros::Rate    r(simulation_rate);
  const double dt = 1 / simulation_rate;

  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = "/world";
  // odom_msg.child_frame_id  = "/" + quad_name;
  odom_msg.child_frame_id  = "/drone_" + std::to_string(drone_id);

  sensor_msgs::Imu imu;
  imu.header.frame_id = "/simulator";

  ros::Time next_odom_pub_time = ros::Time::now();
  while (n.ok())
  {
    ros::spinOnce();

    quad.setControlForce(command_force);

    #ifdef DISTURBANCE_ON
    quad.setExternalForce(disturbance.f);
    quad.setExternalMoment(disturbance.m);
    #endif
    quad.step(dt);

    ros::Time tnow = ros::Time::now();

    if (tnow >= next_odom_pub_time)
    {
      next_odom_pub_time += odom_pub_duration;
      odom_msg.header.stamp = tnow;
      state                 = quad.getState();
      stateToOdomMsg(state, odom_msg);
      quadToImuMsg(quad, imu);
      odom_pub.publish(odom_msg);
      imu_pub.publish(imu);
    }

    r.sleep();
  }

  return 0;
}

void
stateToOdomMsg(const QuadrotorSimulator::Quadrotor::State& state,
               nav_msgs::Odometry&                         odom)
{
  odom.pose.pose.position.x = state.x(0);
  odom.pose.pose.position.y = state.x(1);
  odom.pose.pose.position.z = state.x(2);

  Eigen::Quaterniond q(state.R);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x = state.v(0);
  odom.twist.twist.linear.y = state.v(1);
  odom.twist.twist.linear.z = state.v(2);

  odom.twist.twist.angular.x = state.omega(0);
  odom.twist.twist.angular.y = state.omega(1);
  odom.twist.twist.angular.z = state.omega(2);
}

void
quadToImuMsg(const QuadrotorSimulator::Quadrotor& quad, sensor_msgs::Imu& imu)
{
  QuadrotorSimulator::Quadrotor::State state = quad.getState();
  Eigen::Quaterniond                   q(state.R);
  imu.orientation.x = q.x();
  imu.orientation.y = q.y();
  imu.orientation.z = q.z();
  imu.orientation.w = q.w();

  imu.angular_velocity.x = state.omega(0);
  imu.angular_velocity.y = state.omega(1);
  imu.angular_velocity.z = state.omega(2);

  imu.linear_acceleration.x = quad.getAcc()[0];
  imu.linear_acceleration.y = quad.getAcc()[1];
  imu.linear_acceleration.z = quad.getAcc()[2];
}
