//
// Created by goktug on 25.12.2020.
//

#ifndef CALIBRATION_TEST_POINTCLOUDTYPES_H
#define CALIBRATION_TEST_POINTCLOUDTYPES_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcltype{
    typedef pcl::PointXYZI Point;
    typedef pcl::PointCloud<Point> Cloud;

    typedef pcl::PointXYZRGB PointColored;
    typedef pcl::PointCloud<PointColored> CloudColored;
}


#endif //CALIBRATION_TEST_POINTCLOUDTYPES_H
