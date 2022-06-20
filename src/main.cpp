//
// Created by goktug on 24.12.2020.
//
#include <ros/ros.h>
#include <ros/spinner.h>
#include "CalibrationTest.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "CameraInfoPublisher");
    ROS_INFO("Camera-LiDAR Calibration test node is started.");
    ros::NodeHandle nh;
    CalibrationTest obj_(nh);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}

