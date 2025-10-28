#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


// 全局变量：仅声明（不提前初始化，避免ROS初始化顺序错误）
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
    new pcl::visualization::PCLVisualizer("lidar Cloud (Ground Removed)")
);  // 可视化窗口：雷达坐标系下的滤波后点云
bool is_first_frame = true;
tf::TransformListener* tf_listener = nullptr;  // TF监听器指针（main中初始化）


/**
 * @brief 点云坐标系转换函数（支持雷达→world 或 world→雷达）
 * @param input_cloud 输入点云（对应输入坐标系）
 * @param transform 雷达→world的TF变换关系
 * @param is_lidar_to_world true=雷达→world，false=world→雷达
 * @return 转换后的点云（对应目标坐标系）
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
    const tf::StampedTransform& transform,
    bool is_lidar_to_world
) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    output_cloud->header = input_cloud->header;  // 保留原始帧ID（后续会更新）

    // 提取TF变换的旋转四元数和平移向量
    tf::Quaternion q = transform.getRotation();
    tf::Vector3 t = transform.getOrigin();
    tf::Quaternion q_inv = q.inverse();  // 预计算逆旋转（优化world→雷达的速度）

    // 遍历每个点执行坐标变换
    for (const auto& p_in : input_cloud->points) {
        tf::Vector3 p_in_tf(p_in.x, p_in.y, p_in.z);
        tf::Vector3 p_out_tf;

        if (is_lidar_to_world) {
            // 雷达→world：先旋转（tf::quatRotate确保结果是向量），再加平移
            tf::Vector3 p_rotated = tf::quatRotate(q, p_in_tf);
            p_out_tf = p_rotated + t;
            output_cloud->header.frame_id = "world";  // 标记为world坐标系
        } else {
            // world→雷达：先减平移，再用逆四元数旋转
            tf::Vector3 p_translated = p_in_tf - t;
            p_out_tf = tf::quatRotate(q_inv, p_translated);
            output_cloud->header.frame_id = transform.child_frame_id_;  // 标记为雷达坐标系（从TF中获取，避免硬编码）
        }

        // 将变换后的点加入输出点云
        pcl::PointXYZ p_out;
        p_out.x = p_out_tf.x();
        p_out.y = p_out_tf.y();
        p_out.z = p_out_tf.z();
        output_cloud->points.push_back(p_out);
    }

    output_cloud->width = output_cloud->points.size();
    output_cloud->height = 1;  // 无序点云
    return output_cloud;
}


/**
 * @brief 点云回调函数：完成“雷达→world滤波→转回雷达”全流程
 * @param msg 原始雷达坐标系点云（/points_raw1话题）
 */
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // 1. 读取原始雷达坐标系点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar_raw(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_lidar_raw);
    if (cloud_lidar_raw->empty()) {
        ROS_WARN_THROTTLE(1, "Received empty point cloud (1s only once)");
        return;
    }

    // 2. 获取“雷达→world”的TF变换（仅1次，供双变换使用）
    std::string lidar_frame = msg->header.frame_id;  // 雷达坐标系（从点云消息获取，如lidar_link）
    std::string world_frame = "world";             // 目标world坐标系（需与TF树一致）
    tf::StampedTransform lidar_to_world_tf;

    try {
        // 等待TF变换可用（超时0.1s，避免阻塞回调影响实时性）
        tf_listener->waitForTransform(
            world_frame,        // 目标坐标系（world）
            lidar_frame,          // 源坐标系（雷达）
            msg->header.stamp,  // 与点云时间同步（避免延迟导致的偏差）
            ros::Duration(0.1)
        );
        // 读取TF变换关系
        tf_listener->lookupTransform(world_frame, lidar_frame, msg->header.stamp, lidar_to_world_tf);
    } catch (tf::TransformException& ex) {
        ROS_WARN_THROTTLE(1, "TF transform failed (1s only once): %s", ex.what());
        return;
    }

    // 3. 步骤1：雷达→world（用于精准滤波，world的z轴与地面水平）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_world = transformCloud(cloud_lidar_raw, lidar_to_world_tf, true);

    // 4. 步骤2：world坐标系下滤波（地面点z范围固定为[-0.1, 0.1]，精准剔除）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_world_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_filter;
    pass_filter.setInputCloud(cloud_world);
    pass_filter.setFilterFieldName("z");                // 过滤world坐标系的z轴
    pass_filter.setFilterLimits(-0.1, 0.1);             // 地面点z值范围（可根据实际场景微调）
    pass_filter.setFilterLimitsNegative(true);          // 反向过滤：保留非地面点
    pass_filter.filter(*cloud_world_filtered);

    // 5. 步骤3：world→雷达（最终输出雷达坐标系点云，满足后续使用需求）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar_filtered = transformCloud(cloud_world_filtered, lidar_to_world_tf, false);

    // 6. 可视化雷达坐标系下的滤波后点云
    if (is_first_frame) {
        viewer->setBackgroundColor(0, 0, 0);  // 黑色背景（点云对比更清晰）
        
        // 添加滤波后点云（红色，RGB=255,0,0）
        viewer->addPointCloud<pcl::PointXYZ>(
            cloud_lidar_filtered,
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_lidar_filtered, 255, 0, 0),
            "filtered_lidar_cloud"
        );
        
        // 设置点大小（2像素，避免点太密看不清）
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
            2, 
            "filtered_lidar_cloud"
        );
        
        // 添加雷达坐标系（1米单位，方便判断点云位置）
        viewer->addCoordinateSystem(1.0, lidar_frame);
        
        // 初始化雷达参数（自动调整视角，避免点云不在视野内）
        viewer->initCameraParameters();
        
        is_first_frame = false;  // 初始化完成，后续帧不再执行
    } else {
        // 非第一帧：更新点云（避免重复添加，提升效率）
        viewer->updatePointCloud<pcl::PointXYZ>(
            cloud_lidar_filtered,
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_lidar_filtered, 255, 0, 0),
            "filtered_lidar_cloud"
        );
    }

    // 刷新可视化窗口（超时30ms，平衡实时性和交互响应）
    viewer->spinOnce(30);
}


int main(int argc, char** argv) {
    // 1. 第一步：初始化ROS（必须是第一个ROS操作，避免初始化顺序错误）
    ros::init(argc, argv, "lidar_cloud_ground_removal_tf");
    // 2. 创建ROS节点句柄（后续订阅/发布需依赖）
    ros::NodeHandle nh;

    // 3. 初始化TF监听器（必须在ros::init和NodeHandle之后，避免依赖错误）
    tf_listener = new tf::TransformListener();

    // 4. 订阅原始雷达点云话题（/points_raw1，队列长度1=只处理最新帧，优化实时性）
    ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(
        "/points_raw1", 
        1, 
        cloudCallback
    );

    // 5. 主循环：保持节点运行，直到可视化窗口关闭或ROS退出
    while (ros::ok() && !viewer->wasStopped()) {
        ros::spinOnce();  // 处理回调函数（接收点云、执行变换滤波）
    }

    // 6. 释放内存（避免内存泄漏）
    delete tf_listener;
    return 0;
}