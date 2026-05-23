#include "sphere_detector/sphere_detector.hpp"
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace sphere_detector {

SphereDetector::SphereDetector(ros::NodeHandle& nh, int drone_id, std::function<uint64_t(const int, const Eigen::Vector3d&)> on_added, std::function<void(const uint64_t)> on_removed)
{
    // 从参数服务器读取感知半径（默认 2.0 米）
    nh.param<double>("sphere_detector/perception_radius", perception_radius_, 2.0);
    // perception_radius_ *= 1.1;
    perception_radius_ *= 0.9;

	drone_id_ = drone_id; // 存储无人机编号

    onSphereAdded_ = on_added;
    onSphereRemoved_ = on_removed;

    // 构造里程计话题名称
    std::string odom_topic = "/drone_" + std::to_string(drone_id) + "_odom";

    // 构造可视化发布话题名称（例如：/sphere_detector/drone_1/visible_spheres）
    std::string vis_topic = "/sphere_detector/drone_" + std::to_string(drone_id) + "/visible_spheres";

    // 重置状态
    all_spheres_.clear();
    visible_spheres_.clear();
    has_odom_ = false;
    drone_x_ = drone_y_ = drone_z_ = 0.0;

    // 订阅话题
    sub_center_cloud_ = nh.subscribe("/sphere_generator/center_cloud", 1,
        &SphereDetector::centerCloudCallback, this);
    // sub_center_cloud_ = nh.subscribe("/sphere_generator/sphere_cloud", 1,
    //                                    &SphereDetector::centerCloudCallback, this);
    sub_odom_ = nh.subscribe(odom_topic, 1, &SphereDetector::odomCallback, this);

    // 创建发布者（队列大小1）
    pub_visible_spheres_ = nh.advertise<sensor_msgs::PointCloud2>(vis_topic, 1);

    ROS_INFO("SphereDetector initialized for drone %d, perception radius = %.2f m, publishing to %s",
        drone_id, perception_radius_, vis_topic.c_str());
}

// const std::vector<Eigen::Vector3d>& SphereDetector::getVisibleSpheres() const
// {
//     return visible_spheres_;
// }

void SphereDetector::centerCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (all_spheres_.size() > 0) {
		return; // 已经接收过球心云，暂不更新（如果需要频繁更新，请注释此行）
	}
	
	// 将 ROS 点云转换为 PCL 点云 (PointXYZI)
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    // 清空并重新存储所有球心坐标（忽略半径信息）
    all_spheres_.reserve(pcl_cloud->size());

	for (int i = 0; i < pcl_cloud->size(); ++i) {
        const auto& pt = pcl_cloud->points[i];
		Eigen::Vector3d center(pt.x, pt.y, pt.z);
        all_spheres_.emplace(i, center);
    }

    // ROS_INFO("Received center_cloud with %zu spheres", all_spheres_.size());
    // ROS_INFO("Received sphere_cloud with %zu spheres", all_spheres_.size());

    // 如果已经收到过里程计，立即更新可见球心列表
    if (has_odom_)
        updateVisibleSpheres();
}

void SphereDetector::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 提取无人机位置
    drone_x_ = msg->pose.pose.position.x;
    drone_y_ = msg->pose.pose.position.y;
    drone_z_ = msg->pose.pose.position.z;
    has_odom_ = true;

    // 每收到新的里程计数据，重新筛选可见球心
    updateVisibleSpheres();
}

void SphereDetector::updateVisibleSpheres()
{
    if (!has_odom_ || all_spheres_.empty()) {
        // 即使没有可见球心，也要发布空点云，让RViz清除显示
        publishVisibleSpheres();
        return;
    }

	for (auto &&ob_id_pos : all_spheres_)
	{
        double dx = drone_x_ - ob_id_pos.second.x();
        double dy = drone_y_ - ob_id_pos.second.y();
        double dz = drone_z_ - ob_id_pos.second.z();
        double dist2 = dx * dx + dy * dy + dz * dz;
		const double radius2 = perception_radius_ * perception_radius_;

		auto id_cid = visible_spheres_.find(ob_id_pos.first); 
		if (id_cid != visible_spheres_.end()) {
			if (dist2 > radius2) {
				onSphereRemoved_(id_cid->second);
				visible_spheres_.erase(id_cid);
			}
		} else {
			if (dist2 <= radius2) {
				uint64_t cid = onSphereAdded_(drone_id_, ob_id_pos.second);
				visible_spheres_.emplace(ob_id_pos.first, cid);
			}
		}
	}

    // 发布可视化点云
    publishVisibleSpheres();

    // 可选：输出调试信息（频率高时建议注释）
    // ROS_DEBUG("Visible spheres count: %zu", visible_spheres_.size());
}

void SphereDetector::publishVisibleSpheres()
{
    if (pub_visible_spheres_.getNumSubscribers() == 0)
        return; // 无人订阅，避免不必要的转换

    // 构建PCL点云（使用PointXYZ，不需要强度）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->header.frame_id = "world"; // 与球心点云的坐标系一致
    cloud->points.reserve(visible_spheres_.size());

    for (const auto& ob_id_cid : visible_spheres_) {
        pcl::PointXYZ pt;
        pt.x = all_spheres_.at(ob_id_cid.first).x();
        pt.y = all_spheres_.at(ob_id_cid.first).y();
        pt.z = all_spheres_.at(ob_id_cid.first).z();
        cloud->points.push_back(pt);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    // 转换为ROS消息并发布
    sensor_msgs::PointCloud2 ros_msg;
    pcl::toROSMsg(*cloud, ros_msg);
    ros_msg.header.stamp = ros::Time::now();
    pub_visible_spheres_.publish(ros_msg);
}

} // namespace sphere_detector