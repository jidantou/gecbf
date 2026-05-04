#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <random>
#include <cmath>
#include <vector>
#include <string>
#include <Eigen/Eigen>

using namespace std;

// 随机数生成器
random_device rd;
default_random_engine eng;
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_z;

// 发布者
ros::Publisher sphere_cloud_pub;   // 球体表面点云（用于可视化）
ros::Publisher center_cloud_pub;   // 球心点云（带半径强度）

// 参数
int sphere_num;          // 球体数量
double x_size, y_size, z_size;  // 地图范围
double x_l, x_h, y_l, y_h, z_l_, z_h_;
double radius;           // 球体半径（统一半径）
double resolution;       // 采样分辨率
double pub_rate;         // 发布频率
int max_attempts;        // 每个球体最大随机尝试次数（不重叠时）
bool centerSphere;       // 是否在固定位置(0,0,1.5)额外生成一个球体

// 点云数据
pcl::PointCloud<pcl::PointXYZ> sphere_cloud;      // 球体表面点云
pcl::PointCloud<pcl::PointXYZI> center_cloud;     // 球心点云（强度=半径）

// 存储已生成的球心（用于碰撞检测）
vector<Eigen::Vector3d> centers;

/**
 * @brief 向点云中添加一个球体（表面点 + 球心点）
 */
void addSphereToCloud(double cx, double cy, double cz)
{
    // 生成球体表面点云（经度纬度采样）
    double angle_step = resolution / radius;
    if (angle_step > M_PI / 8) angle_step = M_PI / 8;
    if (angle_step < 0.05) angle_step = 0.05;

    for (double theta = 0; theta <= M_PI; theta += angle_step)
    {
        for (double phi = 0; phi < 2 * M_PI; phi += angle_step)
        {
            double x = cx + radius * sin(theta) * cos(phi);
            double y = cy + radius * sin(theta) * sin(phi);
            double z = cz + radius * cos(theta);

            pcl::PointXYZ pt;
            pt.x = x;
            pt.y = y;
            pt.z = z;
            sphere_cloud.points.push_back(pt);
        }
    }

    // 添加球心点（强度存储半径）
    pcl::PointXYZI center_pt;
    center_pt.x = cx;
    center_pt.y = cy;
    center_pt.z = cz;
    center_pt.intensity = radius;
    center_cloud.points.push_back(center_pt);
}

/**
 * @brief 检查新球心是否与已有球体重叠
 * @param c 新球心坐标
 * @return true 表示不重叠，false 表示重叠
 */
bool isNonOverlapping(const Eigen::Vector3d& c)
{
    double min_distance = 2.0 * radius;  // 球心距必须 >= 2*半径
    for (const auto& exist : centers)
    {
        if ((c - exist).norm() < min_distance)
            return false;
    }
    return true;
}

/**
 * @brief 生成球体表面点云和球心点云（球心随机，优先不重叠）
 *        球心随机生成在边界内，半径为固定值（从参数服务器获取）
 *        如果无法在 max_attempts 次内生成不重叠的球心，则允许重叠并输出警告。
 */
void generateSpheres()
{
    sphere_cloud.points.clear();
    center_cloud.points.clear();
    centers.clear();

    int success_count = 0;   // 实际成功生成的不重叠球体数量（可能小于 sphere_num）

    for (int i = 0; i < sphere_num; ++i)
    {
        Eigen::Vector3d candidate;
        bool found = false;

        // 尝试生成不重叠的球心
        for (int attempt = 0; attempt < max_attempts; ++attempt)
        {
            double cx = rand_x(eng);
            double cy = rand_y(eng);
            double cz = rand_z(eng);

            // 对齐到分辨率网格（与原代码风格一致）
            cx = floor(cx / resolution) * resolution + resolution / 2.0;
            cy = floor(cy / resolution) * resolution + resolution / 2.0;
            cz = floor(cz / resolution) * resolution + resolution / 2.0;

            candidate = Eigen::Vector3d(cx, cy, cz);

            if (isNonOverlapping(candidate))
            {
                found = true;
                break;
            }
        }

        if (!found)
        {
            // 无法找到不重叠的位置：仍使用最后一次生成的候选位置（可能重叠）
            ROS_WARN("Sphere %d: Could not find non-overlapping position after %d attempts. Allowing overlap.", i, max_attempts);
        }
        else
        {
            success_count++;
        }

        // 记录球心（即使重叠也加入，后续生成表面点云）
        centers.push_back(candidate);
        double cx = candidate.x();
        double cy = candidate.y();
        double cz = candidate.z();

        addSphereToCloud(cx, cy, cz);
    }

    // 可选：在固定位置添加一个中心球体
    if (centerSphere)
    {
        const Eigen::Vector3d center_candidate(0.0, 0.0, 1.0);

        if (!isNonOverlapping(center_candidate))
        {
            ROS_WARN("centerSphere enabled: fixed sphere at (0,0,1.5) overlaps with existing spheres.");
        }

        if (center_candidate.x() < x_l || center_candidate.x() > x_h ||
            center_candidate.y() < y_l || center_candidate.y() > y_h ||
            center_candidate.z() < z_l_ || center_candidate.z() > z_h_)
        {
            ROS_WARN("centerSphere enabled: fixed sphere center (0,0,1.5) is outside effective map bounds after radius shrink.");
        }

        centers.push_back(center_candidate);
        addSphereToCloud(center_candidate.x(), center_candidate.y(), center_candidate.z());
    }

    sphere_cloud.width = sphere_cloud.points.size();
    sphere_cloud.height = 1;
    sphere_cloud.is_dense = true;

    center_cloud.width = center_cloud.points.size();
    center_cloud.height = 1;
    center_cloud.is_dense = true;

    const int expected_num = sphere_num + (centerSphere ? 1 : 0);
    ROS_INFO("Map generator: Generated %d spheres (expected %d), radius = %.2f, non-overlapping random count = %d, centerSphere = %s, surface points: %lu",
             (int)centers.size(), expected_num, radius, success_count, centerSphere ? "true" : "false", sphere_cloud.points.size());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sphere_generator");
    ros::NodeHandle n("~");

    unsigned int seed;
    std::string seed_given;
    bool useRandom;

    // 读取参数
    n.param("sphere/num", sphere_num, 10);
    n.param("map/x_size", x_size, 20.0);
    n.param("map/y_size", y_size, 20.0);
    n.param("map/z_size", z_size, 5.0);
    n.param("sphere/radius", radius, 0.5);
    n.param("map/resolution", resolution, 0.1);
    n.param("pub_rate", pub_rate, 10.0);
    n.param("map/useRandom", useRandom, true);
    n.param("map/seed", seed_given, std::to_string(47));
    n.param("sphere/max_attempts", max_attempts, 50);
    n.param("centerSphere", centerSphere, false);

    // 计算边界
    x_l = -x_size / 2.0;
    x_h =  x_size / 2.0;
    y_l = -y_size / 2.0;
    y_h =  y_size / 2.0;
    z_l_ = 0;
    z_h_ = z_size;

    // 考虑半径边界收缩，确保球体完全在区域内
    x_l += radius;
    x_h -= radius;
    y_l += radius;
    y_h -= radius;
    z_l_ += radius;
    z_h_ -= radius;

    // 如果边界无效，调整
    if (x_l >= x_h) { x_l = -x_size/2.0; x_h = x_size/2.0; }
    if (y_l >= y_h) { y_l = -y_size/2.0; y_h = y_size/2.0; }
    if (z_l_ >= z_h_) { z_l_ = 0; z_h_ = z_size; }

    // 初始化随机分布
    rand_x = uniform_real_distribution<double>(x_l, x_h);
    rand_y = uniform_real_distribution<double>(y_l, y_h);
    rand_z = uniform_real_distribution<double>(z_l_, z_h_);

    // 设置随机种子
    if (useRandom)
        seed = rd();
    else
        seed = static_cast<unsigned int>(std::stoul(seed_given));
    // unsigned int seed = 123456789;  // 可固定种子以便复现
    ROS_INFO("Map generator: Random seed = %u", seed);
    eng.seed(seed);

    // 创建发布者
    sphere_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/sphere_generator/sphere_cloud", 1);
    center_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/sphere_generator/center_cloud", 1);

    // 生成球体数据（仅一次，位置固定）
    generateSpheres();

    // 转换为ROS消息
    sensor_msgs::PointCloud2 sphere_msg, center_msg;
    pcl::toROSMsg(sphere_cloud, sphere_msg);
    sphere_msg.header.frame_id = "world";
    pcl::toROSMsg(center_cloud, center_msg);
    center_msg.header.frame_id = "world";

    ros::Rate loop_rate(pub_rate);
    while (ros::ok())
    {
        // 持续发布球心坐标（以及球体点云，便于可视化）
        sphere_msg.header.stamp = ros::Time::now();
        center_msg.header.stamp = ros::Time::now();
        sphere_cloud_pub.publish(sphere_msg);
        center_cloud_pub.publish(center_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}