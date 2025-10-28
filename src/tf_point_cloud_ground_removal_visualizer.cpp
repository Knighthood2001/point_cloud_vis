// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/filters/passthrough.h>  // 滤波
// #include <tf/transform_listener.h>    // TF监听
// #include <tf/transform_datatypes.h>   // TF变换数据类型


// // 全局变量
// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("World Coordinate PointCloud"));
// bool is_first_frame = true;
// // TF监听器（用于获取坐标系变换，需全局或长期存在，避免频繁创建销毁）
// tf::TransformListener* tf_listener = nullptr;


// // 点云回调函数：接收相机坐标系点云→TF转换到world→滤波→可视化
// void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
//     // 1. 读取原始点云（相机坐标系）
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_camera(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromROSMsg(*msg, *cloud_camera);
//     if (cloud_camera->empty()) {
//         ROS_WARN("Received empty point cloud!");
//         return;
//     }

//     // 2. 获取“相机坐标系→world坐标系”的TF变换（不变）
//     std::string camera_frame = msg->header.frame_id;
//     std::string world_frame = "world";
//     tf::StampedTransform transform;

//     try {
//         tf_listener->waitForTransform(world_frame, camera_frame, msg->header.stamp, ros::Duration(3.0));
//         tf_listener->lookupTransform(world_frame, camera_frame, msg->header.stamp, transform);
//     } catch (tf::TransformException& ex) {
//         ROS_ERROR("TF获取失败: %s", ex.what());
//         return;
//     }

//     // 3. 相机→world坐标变换（核心修改：用tf::quaternionRotate旋转向量）
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZ>);
//     cloud_world->header.frame_id = world_frame;

//     tf::Quaternion q = transform.getRotation();  // 旋转四元数（不变）
//     tf::Vector3 t = transform.getOrigin();       // 平移向量（不变）

//     // 遍历每个点：旋转（用正确函数）→ 平移
//     for (const auto& p_cam : cloud_camera->points) {
//         tf::Vector3 p_cam_tf(p_cam.x, p_cam.y, p_cam.z);
        
//         // 关键修改：将tf::quaternionRotate → tf::quatRotate
//         tf::Vector3 p_rotated_tf = tf::quatRotate(q, p_cam_tf);  
//         // 旋转后加平移：得到world坐标系下的点（不变）
//         tf::Vector3 p_world_tf = p_rotated_tf + t;  

//         // 加入world点云（不变）
//         pcl::PointXYZ p_world;
//         p_world.x = p_world_tf.x();
//         p_world.y = p_world_tf.y();
//         p_world.z = p_world_tf.z();
//         cloud_world->points.push_back(p_world);
//     }
//     cloud_world->width = cloud_world->points.size();
//     cloud_world->height = 1;

//     // 4. 地面点剔除（不变）
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PassThrough<pcl::PointXYZ> pass;
//     pass.setInputCloud(cloud_world);
//     pass.setFilterFieldName("z");
//     pass.setFilterLimits(-0.1, 0.1);
//     pass.setFilterLimitsNegative(true);
//     pass.filter(*cloud_filtered);

//     // 5. 可视化（不变）
//     if (is_first_frame) {
//         viewer->setBackgroundColor(0, 0, 0);
//         viewer->addPointCloud<pcl::PointXYZ>(
//             cloud_filtered,
//             pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_filtered, 0, 255, 0),
//             "filtered_world_cloud"
//         );
//         viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "filtered_world_cloud");
//         viewer->addCoordinateSystem(1.0, world_frame);
//         viewer->initCameraParameters();
//         is_first_frame = false;
//     } else {
//         viewer->updatePointCloud<pcl::PointXYZ>(
//             cloud_filtered,
//             pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_filtered, 0, 255, 0),
//             "filtered_world_cloud"
//         );
//     }

//     viewer->spinOnce(100);
// }


// int main(int argc, char**argv) {
//     ros::init(argc, argv, "tf_point_cloud_processor");
//     ros::NodeHandle nh;

//     // 初始化TF监听器（必须在ros::init之后创建）
//     tf_listener = new tf::TransformListener();

//     // 订阅相机点云话题（/points_raw1）
//     ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/points_raw1", 1, cloudCallback);

//     // 保持节点运行
//     while (ros::ok() && !viewer->wasStopped()) {
//         ros::spinOnce();
//     }

//     // 释放TF监听器内存
//     delete tf_listener;
//     return 0;
// }

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
    new pcl::visualization::PCLVisualizer("Camera Cloud (Ground Removed)")
);  // 可视化窗口：相机坐标系下的滤波后点云
bool is_first_frame = true;
tf::TransformListener* tf_listener = nullptr;  // TF监听器指针（main中初始化）


/**
 * @brief 点云坐标系转换函数（支持相机→world 或 world→相机）
 * @param input_cloud 输入点云（对应输入坐标系）
 * @param transform 相机→world的TF变换关系
 * @param is_cam_to_world true=相机→world，false=world→相机
 * @return 转换后的点云（对应目标坐标系）
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
    const tf::StampedTransform& transform,
    bool is_cam_to_world
) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    output_cloud->header = input_cloud->header;  // 保留原始帧ID（后续会更新）

    // 提取TF变换的旋转四元数和平移向量
    tf::Quaternion q = transform.getRotation();
    tf::Vector3 t = transform.getOrigin();
    tf::Quaternion q_inv = q.inverse();  // 预计算逆旋转（优化world→相机的速度）

    // 遍历每个点执行坐标变换
    for (const auto& p_in : input_cloud->points) {
        tf::Vector3 p_in_tf(p_in.x, p_in.y, p_in.z);
        tf::Vector3 p_out_tf;

        if (is_cam_to_world) {
            // 相机→world：先旋转（tf::quatRotate确保结果是向量），再加平移
            tf::Vector3 p_rotated = tf::quatRotate(q, p_in_tf);
            p_out_tf = p_rotated + t;
            output_cloud->header.frame_id = "world";  // 标记为world坐标系
        } else {
            // world→相机：先减平移，再用逆四元数旋转
            tf::Vector3 p_translated = p_in_tf - t;
            p_out_tf = tf::quatRotate(q_inv, p_translated);
            output_cloud->header.frame_id = transform.child_frame_id_;  // 标记为相机坐标系（从TF中获取，避免硬编码）
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
 * @brief 点云回调函数：完成“相机→world滤波→转回相机”全流程
 * @param msg 原始相机坐标系点云（/points_raw1话题）
 */
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // 1. 读取原始相机坐标系点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cam_raw(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_cam_raw);
    if (cloud_cam_raw->empty()) {
        ROS_WARN_THROTTLE(1, "Received empty point cloud (1s only once)");
        return;
    }

    // 2. 获取“相机→world”的TF变换（仅1次，供双变换使用）
    std::string cam_frame = msg->header.frame_id;  // 相机坐标系（从点云消息获取，如camera_link）
    std::string world_frame = "world";             // 目标world坐标系（需与TF树一致）
    tf::StampedTransform cam_to_world_tf;

    try {
        // 等待TF变换可用（超时0.1s，避免阻塞回调影响实时性）
        tf_listener->waitForTransform(
            world_frame,        // 目标坐标系（world）
            cam_frame,          // 源坐标系（相机）
            msg->header.stamp,  // 与点云时间同步（避免延迟导致的偏差）
            ros::Duration(0.1)
        );
        // 读取TF变换关系
        tf_listener->lookupTransform(world_frame, cam_frame, msg->header.stamp, cam_to_world_tf);
    } catch (tf::TransformException& ex) {
        ROS_WARN_THROTTLE(1, "TF transform failed (1s only once): %s", ex.what());
        return;
    }

    // 3. 步骤1：相机→world（用于精准滤波，world的z轴与地面水平）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_world = transformCloud(cloud_cam_raw, cam_to_world_tf, true);

    // 4. 步骤2：world坐标系下滤波（地面点z范围固定为[-0.1, 0.1]，精准剔除）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_world_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_filter;
    pass_filter.setInputCloud(cloud_world);
    pass_filter.setFilterFieldName("z");                // 过滤world坐标系的z轴
    pass_filter.setFilterLimits(-0.1, 0.1);             // 地面点z值范围（可根据实际场景微调）
    pass_filter.setFilterLimitsNegative(true);          // 反向过滤：保留非地面点
    pass_filter.filter(*cloud_world_filtered);

    // 5. 步骤3：world→相机（最终输出相机坐标系点云，满足后续使用需求）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cam_filtered = transformCloud(cloud_world_filtered, cam_to_world_tf, false);

    // 6. 可视化相机坐标系下的滤波后点云
    if (is_first_frame) {
        viewer->setBackgroundColor(0, 0, 0);  // 黑色背景（点云对比更清晰）
        
        // 添加滤波后点云（红色，RGB=255,0,0）
        viewer->addPointCloud<pcl::PointXYZ>(
            cloud_cam_filtered,
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_cam_filtered, 255, 0, 0),
            "filtered_cam_cloud"
        );
        
        // 设置点大小（2像素，避免点太密看不清）
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
            2, 
            "filtered_cam_cloud"
        );
        
        // 添加相机坐标系（1米单位，方便判断点云位置）
        viewer->addCoordinateSystem(1.0, cam_frame);
        
        // 初始化相机参数（自动调整视角，避免点云不在视野内）
        viewer->initCameraParameters();
        
        is_first_frame = false;  // 初始化完成，后续帧不再执行
    } else {
        // 非第一帧：更新点云（避免重复添加，提升效率）
        viewer->updatePointCloud<pcl::PointXYZ>(
            cloud_cam_filtered,
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_cam_filtered, 255, 0, 0),
            "filtered_cam_cloud"
        );
    }

    // 刷新可视化窗口（超时30ms，平衡实时性和交互响应）
    viewer->spinOnce(30);
}


int main(int argc, char** argv) {
    // 1. 第一步：初始化ROS（必须是第一个ROS操作，避免初始化顺序错误）
    ros::init(argc, argv, "cam_cloud_ground_removal_tf");
    // 2. 创建ROS节点句柄（后续订阅/发布需依赖）
    ros::NodeHandle nh;

    // 3. 初始化TF监听器（必须在ros::init和NodeHandle之后，避免依赖错误）
    tf_listener = new tf::TransformListener();

    // 4. 订阅原始相机点云话题（/points_raw1，队列长度1=只处理最新帧，优化实时性）
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