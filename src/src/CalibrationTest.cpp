//
// Created by goktug on 24.12.2020.
//

#include "CalibrationTest.h"
#include "PointCloudTypes.h"


CalibrationTest::CalibrationTest(ros::NodeHandle &nh) :
    nh_(nh),
    camera_topic("/camera/image_raw"),
    lidar_topic("/velodyne_points"),
    cam_info_topic("/camera/camera_info"),
    mat_velo_to_cam_3x4(3,4)
{
    sub_cam_ = std::make_shared<msg_camera>(nh_,camera_topic, 12);
    sub_lidar_ = std::make_shared<msg_lidar>(nh_,lidar_topic, 12);
    sub_cam_info_ = std::make_shared<msg_cam_info>(nh_, cam_info_topic, 12);

    synchronizer_ = std::make_shared<msg_synchronizer_>(ApproxTime(10),*sub_cam_, *sub_lidar_, *sub_cam_info_);
    synchronizer_->registerCallback(boost::bind(&CalibrationTest::Callback,this, _1, _2, _3));

    frustum_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/frustum_cloud", 1);
    frustum_cloud_colored_publisher = nh.advertise<sensor_msgs::PointCloud2>("/frustum_cloud_colored", 1);


    mat_cam_intrinsic_3x3 << 1112.362634, 0.000000, 998.386810, 0.000000, 1101.277314, 566.534762,0.000000, 0.000000, 1.000000;
    mat_velo_to_cam_3x4 <<
    0.0463852390944369, 0.01466058174567638, 1.029219078176173, 0.000955019,
    -0.896812064602486, 0.2389920814805407, -0.04414866208473666, -0.00267233,
    0.07241254358148354, -0.5814859082104717, -0.02069363859114789, -0.000290694;

    Eigen::MatrixXd mat_projector_3x4 = mat_cam_intrinsic_3x3*mat_velo_to_cam_3x4;

    mat_point_transformer.topLeftCorner(3, 4) = mat_projector_3x4;
}

void CalibrationTest::Callback(const sensor_msgs::ImageConstPtr &msg_img, const sensor_msgs::PointCloud2ConstPtr &msg_lidar,
                               const sensor_msgs::CameraInfoConstPtr& msg_cam_info)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat cv_img(cv_ptr->image);
    std::cout << "Image size: " << cv_img.cols << "x"<< cv_img.rows << std::endl;
    std::cout << "Camera frame: " << msg_cam_info->header.frame_id << std::endl;
    cv::Size img_size = cv::Size2d(cv_img.cols, cv_img.rows);

    Cloud::Ptr cloud(new Cloud);
    pcl::fromROSMsg(*msg_lidar, *cloud);

    std::cout << "Cloud size: " << cloud->size() << std::endl;
    std::cout << "Lidar frame: " << msg_lidar->header.frame_id << std::endl;

    Cloud::Ptr frustum_cloud(new Cloud);
    pcltype::CloudColored::Ptr frustum_cloud_colored(new pcltype::CloudColored);
    // Iterate point clouds in order to produce frustum cloud
    for(const auto& point : cloud->points)
    {
        auto pair = CalibrationTest::pointInImagePlane(point, mat_point_transformer, img_size);
        bool point_in_image_plane = pair.first;
        cv::Point point_in_image = pair.second;

        if(point_in_image_plane)
        {
            frustum_cloud->points.push_back(point);
            auto point_colored = CalibrationTest::giveColoredPoint(cv_img, point_in_image, point);
            frustum_cloud_colored->points.push_back(point_colored);
        }
    }
    std::cout << "Frustum cloud size: " << frustum_cloud->size() << std::endl;

    sensor_msgs::PointCloud2 msg_frustum;
    pcl::toROSMsg(*frustum_cloud, msg_frustum);
    msg_frustum.header = msg_lidar->header;
    frustum_cloud_publisher.publish(msg_frustum);

    sensor_msgs::PointCloud2 msg_frustum_colored;
    pcl::toROSMsg(*frustum_cloud_colored, msg_frustum_colored);
    msg_frustum_colored.header = msg_lidar->header;
    frustum_cloud_colored_publisher.publish(msg_frustum_colored);


    std::cout << "************************" << std::endl;




}


std::pair<bool, cv::Point> CalibrationTest::pointInImagePlane(pcltype::Point point, Eigen::Matrix4d mat_point_transformer, cv::Size img_size)
{
    cv::Point point_in_image;
    std::pair<bool, cv::Point> pair;

    double distance = sqrt(pow(point.x, 2) + pow(point.y, 2));

    if (distance <= 0)
    {
        pair.first = false;
        pair.second = point_in_image;
        return pair;
    }

    Eigen::Vector4d pos;
    pos << point.x, point.y, point.z, 1;

    Eigen::Vector4d vec_image_plane_coords = mat_point_transformer * pos;

    //std::cout << vec_image_plane_coords << std::endl;

    if (vec_image_plane_coords(2) <= 0)
    {
        pair.first = false;
        pair.second = point_in_image;
        return pair;
    }

    point_in_image.x = (int) (vec_image_plane_coords(0) / vec_image_plane_coords(2));
    point_in_image.y = (int) (vec_image_plane_coords(1) / vec_image_plane_coords(2));

    if (point_in_image.x < 0
        || point_in_image.y < 0
        || point_in_image.x >= img_size.width
        || point_in_image.y >= img_size.height)
    {
        pair.first = false;
        pair.second = point_in_image;
        return pair;
    }

    pair.first = true;
    pair.second = point_in_image;
    return pair;
}


pcltype::PointColored CalibrationTest::giveColoredPoint(const cv::Mat &image, cv::Point &point_in_image, const pcltype::Point &cloud_point)
{
    const cv::Vec3b& color_of_point = image.at<cv::Vec3b>(point_in_image);
    pcl::PointXYZRGB point_colored = pcl::PointXYZRGB(color_of_point[2],
                                                      color_of_point[1],
                                                      color_of_point[0]);
    point_colored.x = cloud_point.x;
    point_colored.y = cloud_point.y;
    point_colored.z = cloud_point.z;
    return point_colored;
}

/*
void CalibrationTest::ImageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{

    camera_info_msg_.K = {
            1088.733557, 0.000000, 981.725055,
            0.000000, 1108.070580, 562.265356,
            0.000000, 0.000000, 1.000000};

    camera_info_msg_.height = 1280;
    camera_info_msg_.width  = 1920;

    camera_info_msg_.distortion_model = "plumb_bob";

    camera_info_msg_.D = {-0.306301, 0.073200, 0.006803, -0.003658, 0.000000};

    camera_info_msg_.header = image_msg->header;

    camera_info_pub.publish(camera_info_msg_);

    std::cout << "Camera Info is published!\n*************************" << std::endl;

}
*/