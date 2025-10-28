#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>  // ROS与PCL格式转换
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>                  // 包含pcl::PointXYZ
#include <pcl/visualization/pcl_visualizer.h>  // PCL可视化工具

// 全局PCL可视化器（避免频繁创建销毁）
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PointCloudViewer"));
bool is_first_frame = true;  // 标记第一帧点云（用于初始化可视化器）

// 点云回调函数：接收/points_raw1，只提取xyz并显示
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // 1. 转换ROS消息到PCL点云：用PointXYZ（仅x/y/z），自动忽略intensity/ring/time
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);  // 核心转换：只读取msg中的x/y/z字段，其他字段丢弃

    // 2. 初始化可视化器（仅第一帧执行）
    if (is_first_frame) {
        viewer->setBackgroundColor(0, 0, 0);  // 黑色背景（对比更清晰）
        // 添加点云：用自定义颜色（红色，RGB值255,0,0），不依赖话题字段
        viewer->addPointCloud<pcl::PointXYZ>(
            cloud,
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, 255, 0, 0),
            "xyz_cloud"  // 点云ID（后续更新需用相同ID）
        );
        // 设置点大小（2像素，避免点太密看不清）
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "xyz_cloud");
        // 添加坐标系（1米单位，方便判断点云位置）
        viewer->addCoordinateSystem(1.0);
        // 初始化相机参数（自动调整视角，避免点云不在视野内）
        viewer->initCameraParameters();
        is_first_frame = false;  // 初始化完成，后续帧不再执行
    } else {
        // 3. 非第一帧：更新点云数据（避免重复添加点云对象，提升效率）
        viewer->updatePointCloud<pcl::PointXYZ>(
            cloud,
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, 255, 0, 0),
            "xyz_cloud"
        );
    }

    // 4. 刷新可视化窗口（处理鼠标交互、点云更新）
    viewer->spinOnce(100);  // 100ms超时：避免阻塞ROS回调
}

int main(int argc, char** argv) {
    // 初始化ROS节点（节点名：point_cloud_xyz_visualizer）
    ros::init(argc, argv, "point_cloud_xyz_visualizer");
    ros::NodeHandle nh;  // 创建节点句柄（用于订阅话题）

    // 订阅/points_raw1话题：队列长度1（只处理最新帧，减少延迟）
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/points_raw1", 1, cloudCallback);

    // 循环：保持节点运行，直到可视化窗口关闭或ROS退出
    while (ros::ok() && !viewer->wasStopped()) {
        ros::spinOnce();  // 处理ROS回调（接收点云消息）
    }

    return 0;
}