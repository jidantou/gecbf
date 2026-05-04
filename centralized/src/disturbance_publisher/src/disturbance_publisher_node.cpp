#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <string>

struct AxisParams
{
  double constant;
  double mean;
  double stddev;
  double min;
  double max;
};

class DisturbancePublisher
{
public:
  DisturbancePublisher()
    : nh_("~")
  {
    nh_.param("publish_rate", publish_rate_, 100.0);
    nh_.param("distribution_force", force_distribution_, std::string("normal"));
    nh_.param("distribution_moment", moment_distribution_, std::string("normal"));

    // Force parameters (N)
    loadAxisParams("force", force_x_, force_y_, force_z_);

    // Moment/tau parameters (N*m)
    loadAxisParams("moment", moment_x_, moment_y_, moment_z_);

    int random_seed = -1;
    nh_.param("random_seed", random_seed, -1);
    if (random_seed >= 0)
    {
      rng_.seed(static_cast<unsigned int>(random_seed));
    }
    else
    {
      std::random_device rd;
      rng_.seed(rd());
    }

    force_pub_ = nh_.advertise<geometry_msgs::Vector3>("force_disturbance", 50);
    moment_pub_ = nh_.advertise<geometry_msgs::Vector3>("moment_disturbance", 50);

    ROS_INFO("[disturbance_publisher] Started with force distribution='%s', moment distribution='%s'", force_distribution_.c_str(), moment_distribution_.c_str());
  }

  void spin()
  {
    if (publish_rate_ <= 0.0)
    {
      ROS_ERROR("[disturbance_publisher] publish_rate must be > 0. Current: %.3f", publish_rate_);
      return;
    }

    ros::Rate rate(publish_rate_);
    while (ros::ok())
    {
      geometry_msgs::Vector3 f_msg;
      geometry_msgs::Vector3 m_msg;

      f_msg.x = sample(force_distribution_, force_x_);
      f_msg.y = sample(force_distribution_, force_y_);
      f_msg.z = sample(force_distribution_, force_z_);

      m_msg.x = sample(moment_distribution_, moment_x_);
      m_msg.y = sample(moment_distribution_, moment_y_);
      m_msg.z = sample(moment_distribution_, moment_z_);

      force_pub_.publish(f_msg);
      moment_pub_.publish(m_msg);

      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  static double clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(v, hi));
  }

  void loadAxisParams(const std::string& prefix, AxisParams& x, AxisParams& y,
                      AxisParams& z)
  {
    loadOneAxis(prefix, "x", x);
    loadOneAxis(prefix, "y", y);
    loadOneAxis(prefix, "z", z);
  }

  void loadOneAxis(const std::string& prefix, const std::string& axis,
                   AxisParams& params)
  {
    nh_.param(prefix + "/constant_" + axis, params.constant, 0.0);
    nh_.param(prefix + "/mean_" + axis, params.mean, 0.0);
    nh_.param(prefix + "/stddev_" + axis, params.stddev, 0.0);
    nh_.param(prefix + "/min_" + axis, params.min, -std::numeric_limits<double>::infinity());
    nh_.param(prefix + "/max_" + axis, params.max, std::numeric_limits<double>::infinity());

    if (params.min > params.max)
    {
      ROS_WARN("[disturbance_publisher] %s/%s has min > max. Swapping bounds.",
               prefix.c_str(), axis.c_str());
      std::swap(params.min, params.max);
    }

    if (params.stddev < 0.0)
    {
      ROS_WARN("[disturbance_publisher] %s/%s stddev is negative. Taking abs.",
               prefix.c_str(), axis.c_str());
      params.stddev = std::abs(params.stddev);
    }
  }

  double sample(const std::string& distribution, const AxisParams& params)
  {
    double v = params.constant;

    if (distribution == "constant")
    {
      v = params.constant;
    }
    else if (distribution == "normal")
    {
      std::normal_distribution<double> dist(params.mean, params.stddev);
      v = dist(rng_);
    }
    else if (distribution == "uniform")
    {
      std::uniform_real_distribution<double> dist(params.min, params.max);
      v = dist(rng_);
    }
    else
    {
      ROS_WARN_THROTTLE(2.0,
                        "[disturbance_publisher] Unknown distribution '%s'. Using constant.",
                        distribution.c_str());
      v = params.constant;
    }

    return clamp(v, params.min, params.max);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher force_pub_;
  ros::Publisher moment_pub_;

  double publish_rate_;
  std::string force_distribution_;
  std::string moment_distribution_;

  AxisParams force_x_;
  AxisParams force_y_;
  AxisParams force_z_;

  AxisParams moment_x_;
  AxisParams moment_y_;
  AxisParams moment_z_;

  std::mt19937 rng_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "disturbance_publisher_node");

  DisturbancePublisher node;
  node.spin();

  return 0;
}
