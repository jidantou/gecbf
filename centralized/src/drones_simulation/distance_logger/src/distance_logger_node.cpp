#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>
#include <boost/bind.hpp>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <cmath>
#include <signal.h>
#include <atomic>

class DistanceLogger
{
public:
    DistanceLogger()
    {
        ros::NodeHandle nh("~");
        // 获取参数
        nh.param("num_drones", num_drones_, 1);
        nh.param("drone_radius", drone_radius_, 0.1);
        nh.param("obstacle_radius", obstacle_radius_, 0.1);
        
        if (num_drones_ < 1)
        {
            ROS_ERROR("Parameter 'num_drones' must be positive!");
            ros::shutdown();
        }
        
        // 日志目录
        const char* home = std::getenv("HOME");
        if (!home)
        {
            ROS_ERROR("HOME environment variable not set");
            ros::shutdown();
        }
        log_dir_ = std::string(home) + "/Projects/gecbf_log/";
        boost::filesystem::create_directories(log_dir_);
        
        // 预分配文件句柄容器（无人机维度）
        log_files_.resize(num_drones_);
        
        // 初始化无人机位置缓存
        drone_positions_.resize(num_drones_);
        drone_position_received_.resize(num_drones_, false);
        
        // 打开无人机对日志文件（数量固定，不依赖障碍物）
        openDronePairFiles();
        
        obstacle_cloud_sub_ = nh.subscribe("/sphere_generator/center_cloud", 1,
                                           &DistanceLogger::obstacleCloudCallback, this);
        
        // 订阅无人机 odom 话题
        drone_odom_subs_.resize(num_drones_);
        for (int i = 0; i < num_drones_; ++i)
        {
            std::string topic = "/drone_" + std::to_string(i) + "_odom";
            drone_odom_subs_[i] = nh.subscribe<nav_msgs::Odometry>(
                topic, 10, boost::bind(&DistanceLogger::droneOdomCallback, this, _1, i));
        }
        
        // 注册信号处理，确保退出前 flush 文件
        signal(SIGINT, signalHandler);
        // 将当前对象指针存入静态变量，供信号处理函数使用
        instance_ = this;
        
        ROS_INFO("DistanceLogger initialized: %d drones, drone_radius=%.2f, obstacle_radius=%.2f",
                 num_drones_, drone_radius_, obstacle_radius_);
    }
    
    ~DistanceLogger()
    {
        flushAndCloseAllFiles();
    }
    
    // 供信号处理调用的 public 方法
    void flushAndCloseAllFiles()
    {
        // 关闭障碍物日志文件
        for (auto& drone_files : log_files_)
        {
            for (auto& file : drone_files)
            {
                if (file.is_open())
                {
                    file.flush();
                    file.close();
                }
            }
        }
        
        // 关闭无人机对日志文件
        for (auto& row : drone_pair_files_)
        {
            for (auto& file : row)
            {
                if (file.is_open())
                {
                    file.flush();
                    file.close();
                }
            }
        }
        
        ROS_INFO("All log files flushed and closed.");
    }
    
private:
    void openDronePairFiles()
    {
        drone_pair_files_.resize(num_drones_);
        for (int i = 0; i < num_drones_; ++i)
        {
            drone_pair_files_[i].resize(num_drones_);
        }
        
        int pair_count = 0;
        for (int i = 0; i < num_drones_; ++i)
        {
            for (int j = i + 1; j < num_drones_; ++j)
            {
                std::string filename = log_dir_ + "drone" + std::to_string(i) +
                                       "_drone" + std::to_string(j) + ".csv";
                bool file_exists = boost::filesystem::exists(filename);
                // 以追加模式打开，保留已有内容
                drone_pair_files_[i][j].open(filename, std::ios::out | std::ios::app);
                if (!drone_pair_files_[i][j].is_open())
                {
                    ROS_ERROR("Cannot open drone pair file %s", filename.c_str());
                    continue;
                }
                if (!file_exists)
                {
                    drone_pair_files_[i][j] << "timestamp,distance\n";
                    drone_pair_files_[i][j].flush();
                }
                pair_count++;
            }
        }
        // ROS_INFO("Opened %d drone pair log files", pair_count);
    }
    
    void obstacleCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        if (obstacles_initialized_)
            return;
        
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*cloud_msg, pcl_cloud);
        
        obstacle_centers_.clear();
        for (const auto& point : pcl_cloud.points)
        {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            obstacle_centers_.push_back(p);
        }
        
        if (obstacle_centers_.empty())
        {
            ROS_WARN("Obstacle cloud is empty!");
            return;
        }
        
        obstacles_initialized_ = true;
        // ROS_INFO("Obstacle cloud received with %zu obstacles", obstacle_centers_.size());
        
        // 为每个无人机-障碍物对打开文件句柄并写入表头
        openAllObstacleLogFiles();
    }
    
    void openAllObstacleLogFiles()
    {
        size_t num_obs = obstacle_centers_.size();
        for (int drone_id = 0; drone_id < num_drones_; ++drone_id)
        {
            log_files_[drone_id].resize(num_obs);
            for (size_t obs_idx = 0; obs_idx < num_obs; ++obs_idx)
            {
                std::string filename = log_dir_ + "drone" + std::to_string(drone_id) +
                                       "_obstacle" + std::to_string(obs_idx) + ".csv";
                bool file_exists = boost::filesystem::exists(filename);
                // 以追加模式打开，保留已有内容
                log_files_[drone_id][obs_idx].open(filename, std::ios::out | std::ios::app);
                if (!log_files_[drone_id][obs_idx].is_open())
                {
                    ROS_ERROR("Cannot open file %s", filename.c_str());
                    continue;
                }
                if (!file_exists)
                {
                    log_files_[drone_id][obs_idx] << "timestamp,distance\n";
                    log_files_[drone_id][obs_idx].flush();  // 确保表头立即写入
                }
            }
        }
        // ROS_INFO("All obstacle log files opened successfully");
    }
    
    double computeSurfaceDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2,
                                  double r1, double r2) const
    {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        double center_dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        return center_dist - (r1 + r2);
    }
    
    void droneOdomCallback(const nav_msgs::OdometryConstPtr& odom_msg, int drone_id)
    {
        double stamp = odom_msg->header.stamp.toSec();
        double drone_x = odom_msg->pose.pose.position.x;
        double drone_y = odom_msg->pose.pose.position.y;
        double drone_z = odom_msg->pose.pose.position.z;
        
        // 更新当前无人机的位置缓存
        drone_positions_[drone_id].x = drone_x;
        drone_positions_[drone_id].y = drone_y;
        drone_positions_[drone_id].z = drone_z;
        drone_position_received_[drone_id] = true;
        
        // --- 记录无人机-障碍物距离（如果障碍物已初始化）---
        if (obstacles_initialized_)
        {
        size_t num_obs = obstacle_centers_.size();
        // 直接使用已打开的文件句柄写入，避免重复open/close
        for (size_t obs_idx = 0; obs_idx < num_obs; ++obs_idx)
        {
            const auto& obs = obstacle_centers_[obs_idx];
                double surface_dist = computeSurfaceDistance(drone_positions_[drone_id], obs,
                                                              drone_radius_, obstacle_radius_);
                std::ofstream& file = log_files_[drone_id][obs_idx];
                if (file.is_open())
                {
                    file << std::fixed << std::setprecision(9) << stamp << "," << surface_dist << "\n";
                }
            }
        }
        
        // --- 记录无人机-无人机距离 ---
        for (int other_id = 0; other_id < num_drones_; ++other_id)
        {
            if (other_id == drone_id) continue;
            if (!drone_position_received_[other_id]) continue;  // 另一无人机位置尚未收到
            
            // 确定无人机对索引 (i < j)
            int i = std::min(drone_id, other_id);
            int j = std::max(drone_id, other_id);
            std::ofstream& file = drone_pair_files_[i][j];
            if (file.is_open())
            {
                double surface_dist = computeSurfaceDistance(drone_positions_[drone_id],
                                                              drone_positions_[other_id],
                                                              drone_radius_, drone_radius_);
                file << std::fixed << std::setprecision(9) << stamp << "," << surface_dist << "\n";
                // 为了平衡性能和可靠性，每100行 flush 一次（可选）
                // static int counter = 0; if (++counter % 100 == 0) file.flush();
            }
        }
    }
    
    // 静态信号处理函数
    static void signalHandler(int sig)
    {
        if (sig == SIGINT && instance_ != nullptr)
        {
            // ROS_INFO("Received SIGINT, flushing logs...");
            instance_->flushAndCloseAllFiles();
            ros::shutdown();
        }
    }
    
    int num_drones_;
    double drone_radius_;
    double obstacle_radius_;
    std::string log_dir_;
    
    ros::Subscriber obstacle_cloud_sub_;
    std::vector<ros::Subscriber> drone_odom_subs_;
    
    bool obstacles_initialized_ = false;
    std::vector<geometry_msgs::Point> obstacle_centers_;
    // 障碍物文件句柄: [drone_id][obstacle_index]
    std::vector<std::vector<std::ofstream>> log_files_;
    
    // 无人机位置缓存
    std::vector<geometry_msgs::Point> drone_positions_;
    std::vector<bool> drone_position_received_;
    // 无人机对文件句柄: 只使用 i<j 的位置，其他为空
    std::vector<std::vector<std::ofstream>> drone_pair_files_;
    
    static DistanceLogger* instance_;
};

// 定义静态成员
DistanceLogger* DistanceLogger::instance_ = nullptr;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "distance_logger");
    DistanceLogger logger;
    ros::spin();
    // spin 正常结束后，logger 析构时也会 flush（但信号已经提前处理）
    return 0;
}