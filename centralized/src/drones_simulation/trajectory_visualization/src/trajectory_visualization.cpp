#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <vector>
#include <algorithm>

class TrajectoryPublisher
{
public:
    TrajectoryPublisher(ros::NodeHandle& nh) : nh_(nh)
    {
        // 获取遗忘时间阈值参数（单位：秒），默认值为5.0秒
        nh_.param<double>("timeout", timeout_, 5.0);
        timeout_duration_ = ros::Duration(timeout_);

        // 订阅里程计话题（通常为"odom"）
        odom_sub_ = nh_.subscribe("odom", 10, &TrajectoryPublisher::odomCallback, this);

        // 发布轨迹话题
        path_pub_ = nh_.advertise<nav_msgs::Path>("trajectory", 10);

        // 定时器：每隔0.5秒检查并发布一次，实现自动遗忘（无新消息时也能移除过期点）
        timer_ = nh_.createTimer(ros::Duration(5.0), &TrajectoryPublisher::timerCallback, this);

        ROS_INFO("Trajectory publisher initialized with timeout = %.2f seconds", timeout_);
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // 将里程计消息转换为PoseStamped并添加到轨迹缓存中
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = msg->header;          // 使用相同的header（包含frame_id和时间戳）
        pose_stamped.pose = msg->pose.pose;

        // 添加到轨迹列表
        path_.push_back(pose_stamped);

        // 每次收到新消息后，立即清理过期点并发布
        publishPath();
    }

    void timerCallback(const ros::TimerEvent&)
    {
        // 定时清理并发布，确保在没有新消息时也能遗忘旧点
        publishPath();
    }

    void publishPath()
    {
        // 移除超过时间阈值的旧轨迹点
        prunePath();

        // 构建Path消息
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        // 设置frame_id：如果有轨迹点，则使用第一个点的frame_id；否则使用默认的"odom"
        if (!path_.empty())
        {
            path_msg.header.frame_id = path_.front().header.frame_id;
        }
        else
        {
            path_msg.header.frame_id = "world";
        }
        path_msg.poses = path_;

        // 发布轨迹
        path_pub_.publish(path_msg);
    }

    void prunePath()
    {
        // timeout < 0 表示不遗忘轨迹点，保留全部历史轨迹
        if (timeout_ < 0.0)
            return;

        if (path_.empty())
            return;

        ros::Time now = ros::Time::now();
        // 移除所有时间戳小于 (当前时间 - timeout_) 的点
        auto it = std::remove_if(path_.begin(), path_.end(),
            [&](const geometry_msgs::PoseStamped& pose)
            {
                return (now - pose.header.stamp) > timeout_duration_;
            });
        path_.erase(it, path_.end());
    }

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher path_pub_;
    ros::Timer timer_;

    std::vector<geometry_msgs::PoseStamped> path_;
    double timeout_;
    ros::Duration timeout_duration_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle nh("~");  // 使用私有命名空间，方便读取参数

    TrajectoryPublisher traj_pub(nh);

    ros::spin();
    return 0;
}