#pragma once

#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace demo {

struct CloudPoint {
    Eigen::Vector3d pt3d = Eigen::Vector3d::Zero();
    unsigned char intensity = 0.0;
    double timestamp = 0.0;
};

typedef std::vector<CloudPoint> PointCloud;

bool load_pcd(const std::string& file_path, PointCloud *cloud);

}  // namespace demo
