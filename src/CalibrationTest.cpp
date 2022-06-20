//
// Created by goktug on 24.12.2020.
//

#include "CalibrationTest.h"
#include "PointCloudTypes.h"
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <pcl/common/transforms.h>

CalibrationTest::CalibrationTest(ros::NodeHandle &nh) :
    nh_(nh),
    camera_topic("/arena_camera_node/image_raw"),
    lidar_topic("/velodyne_points"),
    cam_info_topic("/arena_camera_node/camera_info"),
    it(nh)
{
    tfListener = std::make_shared<tf2_ros::TransformListener>(tfBuffer);

    sub_cam_ = std::make_shared<msg_camera>(nh_,camera_topic, 12);
    sub_lidar_ = std::make_shared<msg_lidar>(nh_,lidar_topic, 12);
    sub_cam_info_ = std::make_shared<msg_cam_info>(nh_, cam_info_topic, 12);

    synchronizer_ = std::make_shared<msg_synchronizer_>(ApproxTime(10),*sub_cam_, *sub_lidar_, *sub_cam_info_);
    synchronizer_->registerCallback(boost::bind(&CalibrationTest::Callback,this, _1, _2, _3));

    frustum_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/frustum_cloud", 1);
    frustum_cloud_colored_publisher = nh.advertise<sensor_msgs::PointCloud2>("/frustum_cloud_colored", 1);
    image_pub = it.advertise("/projected_image", 1);
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

    cv::Mat cv_img; //opencv matrisine çeviriyor
    cv_ptr->image.copyTo(cv_img);
    std::cout << "Image size: " << cv_img.cols << "x"<< cv_img.rows << std::endl;
    std::cout << "Camera frame: " << msg_cam_info->header.frame_id << std::endl;
    cv::Size img_size = cv::Size2d(cv_img.cols, cv_img.rows);

    Cloud::Ptr cloud(new Cloud);
    pcl::fromROSMsg(*msg_lidar, *cloud);

    std::cout << "Cloud size: " << cloud->size() << std::endl;
    std::cout << "Lidar frame: " << msg_lidar->header.frame_id << std::endl;

    project_lidar_points_to_image_plan (
            cv_img,
            image_pub,
            frustum_cloud_colored_publisher,
            15,
            cloud);
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

void CalibrationTest::project_lidar_points_to_image_plan (
                                               const cv::Mat &image,
                                               const image_transport::Publisher& pub_undistorted_image_,
                                               const ros::Publisher& pub_cloud_frustum_colored,
                                               const double& max_distance_cam_seen,
                                               Cloud::Ptr& input_cloud)
{
    cv::Mat undistortedImage;
    // define distort mat : dist = [-0.1621, 0.1176, -0.0065, -0.0045, -0.0403]
    cv::Mat dist = (cv::Mat_<double>(5,1) << -0.1621, 0.1176, -0.0065, -0.0045, -0.0403);
    //define intrinsic mat : intrinsic = [2171.9, 0, 1372.4, 0, 2176.5, 0846.6, 0, 0, 1]
    cv::Mat intrinsic = (cv::Mat_<double>(3,3) << 2171.9, 0, 1372.4, 0, 2176.5, 0846.6, 0, 0, 1);
    cv::undistort(image, undistortedImage,
                    intrinsic, dist);


    // Extrinsic:
    Eigen::Quaterniond q(0.521658, 0.498583, -0.481028, 0.498298);
    Eigen::Matrix3d mat_cam_rotation_3x3 = q.toRotationMatrix();
    Eigen::Matrix4d mat_transf_lid_to_cam = Eigen::Matrix4d::Identity(4,4);
    mat_transf_lid_to_cam.block<3,3>(0,0) = mat_cam_rotation_3x3;
    mat_transf_lid_to_cam.block<3,1>(0,3) = Eigen::Vector3d(0.0390706, -0.207147, -0.084638);



    // Intrinsic:
    Eigen::Matrix3d mat_cam_intrinsic_3x3 = Eigen::MatrixXd::Zero(3,3);
    mat_cam_intrinsic_3x3 <<
            2171.9, 0, 1372.4,
            0, 2176.5, 0846.6,
            0, 0, 1;

    // Mat point transformer:
    Eigen::Matrix4d mat_point_projector = Eigen::MatrixXd::Identity(4,4);
    Eigen::MatrixXd mat(3,4);
    mat = mat_cam_intrinsic_3x3*mat_transf_lid_to_cam.topLeftCorner(3,4);
    mat_point_projector.topLeftCorner(3,4) = mat;

    Cloud::Ptr frustum(new Cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr frustum_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

    cv::Mat undistorted_image_clear;
    undistortedImage.copyTo(undistorted_image_clear);

    size_t count_point_in_img = 0;
    for (const auto &point : input_cloud->points)
    {
        Eigen::Vector4d pos;
        pos << point.x, point.y, point.z, 1;

        Eigen::Vector4d vec_image_plane_coords = mat_point_projector * pos;

        if (vec_image_plane_coords(2) <= 0)
            continue;

        cv::Point point_in_image;

        point_in_image.x = (int) (vec_image_plane_coords(0) / vec_image_plane_coords(2));
        point_in_image.y = (int) (vec_image_plane_coords(1) / vec_image_plane_coords(2));

        if (point_in_image.x < 0
            || point_in_image.y < 0
            || point_in_image.x >= image.cols
            || point_in_image.y >= image.rows)
            continue;

        frustum->points.push_back(point);

        pcl::PointXYZRGB point_c =  CalibrationTest::giveColoredPoint(undistorted_image_clear, point_in_image, point);
        frustum_colored->points.push_back(point_c);

        auto getDistance = [](const Point& p)->float
        {
            float distance = sqrt(pow(p.x,2) + pow(p.y,2));
            return distance;
        };
        float dist = getDistance(point);
        auto color = distance_to_color(dist, max_distance_cam_seen);
        int r = std::get<0>(color);
        int g = std::get<1>(color);
        int b = std::get<2>(color);

        int radiusCircle = 2;
        cv::Scalar colorCircle1(r,g,b);
        int thicknessCircle1 = 2;

        cv::circle(undistortedImage, point_in_image, radiusCircle, colorCircle1, thicknessCircle1);
        count_point_in_img++;
    }

    if (count_point_in_img == 0)
        std::cout << "There is no image in the image plane!" << std::endl;
    pcl::transformPointCloud(*frustum_colored, *frustum_colored, mat_transf_lid_to_cam);
    // Publish frustum colored cloud:
    sensor_msgs::PointCloud2 msg_cloud_frustum_colored;
    pcl::toROSMsg(*frustum_colored, msg_cloud_frustum_colored);
    msg_cloud_frustum_colored.header.frame_id = "camera";
    msg_cloud_frustum_colored.header.stamp = ros::Time::now();
    pub_cloud_frustum_colored.publish(msg_cloud_frustum_colored);

    // Publish undistorted image:
    cv_bridge::CvImage out_msg;
    out_msg.header.frame_id   = "camera";
    out_msg.header.stamp = ros::Time::now();
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image    = undistortedImage;
    pub_undistorted_image_.publish(out_msg.toImageMsg());


}



std::tuple<int, int, int> CalibrationTest::distance_to_color(const double& distance,
                                                  const double& max_distance) {
    double hh, p, q, t, ff;
    long i;
    double v = 0.75;
    double s = 0.75;
    double r, g, b;
    double h = ((max_distance - distance) / max_distance) * 360.0;
    hh = h;
    if (hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long) hh;
    ff = hh - i;
    p = v * (1.0 - s);
    q = v * (1.0 - (s * ff));
    t = v * (1.0 - (s * (1.0 - ff)));

    switch (i) {
        case 0:
            r = v;
            g = t;
            b = p;
            break;
        case 1:
            r = q;
            g = v;
            b = p;
            break;
        case 2:
            r = p;
            g = v;
            b = t;
            break;

        case 3:
            r = p;
            g = q;
            b = v;
            break;
        case 4:
            r = t;
            g = p;
            b = v;
            break;
        case 5:
        default:
            r = v;
            g = p;
            b = q;
            break;
    }
    return std::make_tuple((int) (r * 255), (int) (g * 255), (int) (b * 255));

}