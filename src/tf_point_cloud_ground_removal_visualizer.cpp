#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>  // 滤波
#include <tf/transform_listener.h>    // TF监听
#include <tf/transform_datatypes.h>   // TF变换数据类型


// 全局变量
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("World Coordinate PointCloud"));
bool is_first_frame = true;
// TF监听器（用于获取坐标系变换，需全局或长期存在，避免频繁创建销毁）
tf::TransformListener* tf_listener = nullptr;


// 点云回调函数：接收相机坐标系点云→TF转换到world→滤波→可视化
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // 1. 读取原始点云（相机坐标系）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_camera(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_camera);
    if (cloud_camera->empty()) {
        ROS_WARN("Received empty point cloud!");
        return;
    }

    // 2. 获取“相机坐标系→world坐标系”的TF变换（不变）
    std::string camera_frame = msg->header.frame_id;
    std::string world_frame = "world";
    tf::StampedTransform transform;

    try {
        tf_listener->waitForTransform(world_frame, camera_frame, msg->header.stamp, ros::Duration(3.0));
        tf_listener->lookupTransform(world_frame, camera_frame, msg->header.stamp, transform);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("TF获取失败: %s", ex.what());
        return;
    }

    // 3. 相机→world坐标变换（核心修改：用tf::quaternionRotate旋转向量）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_world->header.frame_id = world_frame;

    tf::Quaternion q = transform.getRotation();  // 旋转四元数（不变）
    tf::Vector3 t = transform.getOrigin();       // 平移向量（不变）

    // 遍历每个点：旋转（用正确函数）→ 平移
    for (const auto& p_cam : cloud_camera->points) {
        tf::Vector3 p_cam_tf(p_cam.x, p_cam.y, p_cam.z);
        
        // 关键修改：将tf::quaternionRotate → tf::quatRotate
        tf::Vector3 p_rotated_tf = tf::quatRotate(q, p_cam_tf);  
        // 旋转后加平移：得到world坐标系下的点（不变）
        tf::Vector3 p_world_tf = p_rotated_tf + t;  

        // 加入world点云（不变）
        pcl::PointXYZ p_world;
        p_world.x = p_world_tf.x();
        p_world.y = p_world_tf.y();
        p_world.z = p_world_tf.z();
        cloud_world->points.push_back(p_world);
    }
    cloud_world->width = cloud_world->points.size();
    cloud_world->height = 1;

    // 4. 地面点剔除（不变）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_world);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.1, 0.1);
    pass.setFilterLimitsNegative(true);
    pass.filter(*cloud_filtered);

    // 5. 可视化（不变）
    if (is_first_frame) {
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(
            cloud_filtered,
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_filtered, 0, 255, 0),
            "filtered_world_cloud"
        );
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "filtered_world_cloud");
        viewer->addCoordinateSystem(1.0, world_frame);
        viewer->initCameraParameters();
        is_first_frame = false;
    } else {
        viewer->updatePointCloud<pcl::PointXYZ>(
            cloud_filtered,
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_filtered, 0, 255, 0),
            "filtered_world_cloud"
        );
    }

    viewer->spinOnce(100);
}


int main(int argc, char**argv) {
    ros::init(argc, argv, "tf_point_cloud_processor");
    ros::NodeHandle nh;

    // 初始化TF监听器（必须在ros::init之后创建）
    tf_listener = new tf::TransformListener();

    // 订阅相机点云话题（/points_raw1）
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/points_raw1", 1, cloudCallback);

    // 保持节点运行
    while (ros::ok() && !viewer->wasStopped()) {
        ros::spinOnce();
    }

    // 释放TF监听器内存
    delete tf_listener;
    return 0;
}