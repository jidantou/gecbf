#include <ros/ros.h>
#include <memory>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

// mode: 0: preset goal, 1: random goal(TODO)
static int mode = 0;

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
  // TODO
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_publisher_node");
  ros::NodeHandle nh("~");

  int drone_num;
  nh.param("drone_num", drone_num, 1);

  std::unique_ptr<ros::Publisher[]> task_pubs(new ros::Publisher[drone_num]);
  std::unique_ptr<ros::Subscriber[]> odom_subs(new ros::Subscriber[drone_num]);
  std::unique_ptr<geometry_msgs::PoseStamped[]> goals(new geometry_msgs::PoseStamped[drone_num]);
  for (int i = 0; i < drone_num; i++)
  {
    task_pubs[i] = nh.advertise<geometry_msgs::PoseStamped>("/drone_" + std::to_string(i) + "_task_pose", 3);
    odom_subs[i] = nh.subscribe<nav_msgs::Odometry>("/drone_" + std::to_string(i) + "_odom", 10, odom_callback, ros::TransportHints().tcpNoDelay());
  }

  if (mode == 0) {
    // int waypoint_num;
    // nh.param("waypoint_num", waypoint_num, -1);

    for (int i = 0; i < drone_num; i++)
    {
      nh.param<double>("drone_" + std::to_string(i) + "_goal_pos_x", goals[i].pose.position.x, 0);
      nh.param<double>("drone_" + std::to_string(i) + "_goal_pos_y", goals[i].pose.position.y, 0);
      nh.param<double>("drone_" + std::to_string(i) + "_goal_pos_z", goals[i].pose.position.z, 0);
      nh.param<double>("drone_" + std::to_string(i) + "_goal_ori_w", goals[i].pose.orientation.w, 1);
      nh.param<double>("drone_" + std::to_string(i) + "_goal_ori_x", goals[i].pose.orientation.x, 0);
      nh.param<double>("drone_" + std::to_string(i) + "_goal_ori_y", goals[i].pose.orientation.y, 0);
      nh.param<double>("drone_" + std::to_string(i) + "_goal_ori_z", goals[i].pose.orientation.z, 0);
      // goals[i].header.frame_id = "/drone_" + std::to_string(i);
      goals[i].header.frame_id = "world";
    }
  }
  for (int i = 0; i < drone_num; i++)
  {
    goals[i].header.stamp = ros::Time::now();
    task_pubs[i].publish<geometry_msgs::PoseStamped>(goals[i]);
  }

  ros::Rate r(1);
  while (ros::ok())
  {
    ros::spinOnce();
    // TODO: check whether drone arrive goal, if arrival, publish next goal pos
    for (int i = 0; i < drone_num; i++)
    {
      goals[i].header.stamp = ros::Time::now();
      task_pubs[i].publish<geometry_msgs::PoseStamped>(goals[i]);
    }
    r.sleep();
  }
  
}