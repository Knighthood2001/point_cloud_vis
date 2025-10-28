#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>  // ROS与PCL格式转换
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>  // PCL可视化工具

// 全局可视化器（避免频繁创建）
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("点云可视化器"));
bool is_first_frame = true;  // 标记第一帧点云（用于初始化）

// 点云回调函数：接收ROS消息并显示
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // 1. 将ROS的PointCloud2转换为PCL点云（根据点云类型选择PointType，这里以带RGB为例）
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);  // 核心转换函数

    // 2. 初始化可视化器（仅第一帧执行）
    if (is_first_frame) {
        viewer->setBackgroundColor(0, 0, 0);  // 黑色背景
        // 添加点云（"cloud"为ID，用于后续更新）
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);  // 使用点云自带RGB
        viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");  // 点大小
        viewer->addCoordinateSystem(1.0);  // 添加坐标系（1米单位）
        viewer->initCameraParameters();  // 初始化相机参数
        is_first_frame = false;
    } else {
        // 非第一帧：更新点云数据（避免重复添加）
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer->updatePointCloud<pcl::PointXYZRGB>(cloud, rgb, "cloud");
    }

    // 3. 刷新可视化窗口
    viewer->spinOnce(100);  // 处理事件，超时100ms
}

int main(int argc, char**argv) {
    // 初始化ROS节点（节点名：point_cloud_visualizer）
    ros::init(argc, argv, "point_cloud_visualizer");
    ros::NodeHandle nh;  // 创建节点句柄

    // 订阅点云话题（替换为你的实际话题名，如"/velodyne_points"）
    // 队列长度1：只处理最新帧，避免缓存堆积
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/points_raw1", 1, cloudCallback);

    // 循环：保持节点运行，直到可视化窗口关闭或ROS退出
    while (ros::ok() && !viewer->wasStopped()) {
        ros::spinOnce();  // 处理回调函数
    }

    return 0;
}