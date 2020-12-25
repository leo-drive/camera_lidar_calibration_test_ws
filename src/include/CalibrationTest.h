//
// Created by goktug on 24.12.2020.
//

#ifndef CALIBRATION_TEST_CALIBRATIONTEST_H
#define CALIBRATION_TEST_CALIBRATIONTEST_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <tf/transform_listener.h>

#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_eigen/tf2_eigen.h>

#include "PointCloudTypes.h"

class CalibrationTest {
public:
    using Point = pcl::PointXYZI;
    using Cloud = pcl::PointCloud<pcl::PointXYZI>;

    explicit CalibrationTest(ros::NodeHandle &nh);

    ros::NodeHandle& nh_;

    image_transport::ImageTransport it;
    image_transport::Publisher image_pub;

    ros::Publisher frustum_cloud_publisher;
    ros::Publisher frustum_cloud_colored_publisher;

    std::string camera_topic;
    std::string lidar_topic;
    std::string cam_info_topic;

    tf2_ros::Buffer tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;

    Eigen::Matrix3d mat_cam_intrinsic_3x3;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
    sensor_msgs::PointCloud2, sensor_msgs::CameraInfo> ApproxTime;

    typedef message_filters::Subscriber<sensor_msgs::Image> msg_camera;
    typedef message_filters::Subscriber<sensor_msgs::PointCloud2> msg_lidar;
    typedef message_filters::Subscriber<sensor_msgs::CameraInfo> msg_cam_info;

    typedef message_filters::Synchronizer<ApproxTime> msg_synchronizer_;

    std::shared_ptr<msg_synchronizer_> synchronizer_;

    std::shared_ptr<msg_camera> sub_cam_ ;
    std::shared_ptr<msg_lidar> sub_lidar_ ;
    std::shared_ptr<msg_cam_info> sub_cam_info_;

    void Callback(const sensor_msgs::ImageConstPtr &msg_img, const sensor_msgs::PointCloud2ConstPtr &msg_lidar,
                  const sensor_msgs::CameraInfoConstPtr& msg_cam_info);

    static pcltype::PointColored giveColoredPoint(const cv::Mat &image, cv::Point &point_in_image, const pcltype::Point &cloud_point);

    static std::pair<bool, cv::Point> pointInImagePlane(pcltype::Point point, Eigen::Matrix4d mat_point_transformer, cv::Size img_size);








};


#endif //CALIBRATION_TEST_CALIBRATIONTEST_H





