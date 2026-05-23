#ifndef SPHERE_DETECTOR_H
#define SPHERE_DETECTOR_H

#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <functional>

namespace sphere_detector {

class SphereDetector {
public:
    /**
     * @brief 构造函数（调用 init 进行实际初始化）
     * @param nh ROS节点句柄（通常为私有句柄，用于读取参数）
     * @param drone_id 无人机编号，用于构建里程计话题名称 "/drone_X_odom"
     */
    SphereDetector(ros::NodeHandle& nh, int drone_id, std::function<uint64_t(const int, const Eigen::Vector3d&)> on_added, std::function<void(const uint64_t)> on_removed);

    /**
     * @brief 获取当前位于感知半径内的球心坐标列表（只读引用）
     * @return const std::vector<Eigen::Vector3d>&
     */
    // const std::vector<Eigen::Vector3d>& getVisibleSpheres() const;

	void regObstaclesAddCallbacks(std::function<uint64_t(const int, const Eigen::Vector3d&)> on_added) {
		onSphereAdded_ = on_added;
	}
	void regObstaclesRemoveCallbacks(std::function<void(const uint64_t)> on_removed) {
		onSphereRemoved_ = on_removed;
	}

private:
    // ROS 相关
    ros::Subscriber sub_center_cloud_;
    ros::Subscriber sub_odom_;
    ros::Publisher pub_visible_spheres_; // 用于RViz可视化的发布者

	int drone_id_; // 无人机编号（从构造函数传入）

    // 参数
    double perception_radius_; // 感知半径（米）

    // 数据存储
	// key: id, value: 球心坐标（静态）
    std::unordered_map<uint64_t, Eigen::Vector3d> all_spheres_;
	// key: 当前可见球心id in all_spheres_, value: id in gecbf's constraint which used when removing constraint
    std::unordered_map<uint64_t, uint64_t> visible_spheres_;

    bool has_odom_; // 是否已收到里程计
    double drone_x_, drone_y_, drone_z_; // 无人机当前位置

	std::function<uint64_t(const int, const Eigen::Vector3d&)> onSphereAdded_; // 球心进入感知范围时的回调函数（参数为球心坐标，返回值为球心id）
	std::function<void(const uint64_t)> onSphereRemoved_; // 球心离开感知范围时的回调函数（参数为球心id）

    // 回调函数
    void centerCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    /**
     * @brief 根据当前无人机位置重新筛选可见球心，并发布可视化点云
     */
    void updateVisibleSpheres();

    /**
     * @brief 发布可见球心点云（用于RViz）
     */
    void publishVisibleSpheres();
};

} // namespace sphere_detector

#endif // SPHERE_DETECTOR_H