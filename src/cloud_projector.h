#pragma once

#include <memory>

#include "opencv2/opencv.hpp"

#include "poses_interpolation.h"
#include "point_cloud.h"
#include "camera_model.h"
#include "intrinsics_io.h"
#include "util.h"

namespace demo {

struct ImagePixel {
    float x = -1.0;
    float y = -1.0;
    float depth = -1.0;
};

struct CloudProjectorConfig {
    std::string odometry_pose_file = "";
    std::string lidar2vehicle_extrinsic_file = "";
    std::string cam2vehicle_extrinsic_file = "";
    std::string cam_intrinsic_file = "";
    double rolling_shutter_time = 0.0455625;
};

class CloudProjector {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CloudProjector() {}

    bool init(const CloudProjectorConfig &config);

    bool project_cloud_to_image(
            const double &img_stamp, const cv::Mat &img,
            const double &cloud_stamp, const PointCloud &cloud);

    cv::Mat get_rendered_image();

private:
    bool project_rolling_shutter(
            const double &img_stamp, const cv::Mat &img,
            const double &cloud_stamp, const PointCloud &cloud,
            PointCloud *inliers, PointCloud *outliers, std::vector<ImagePixel> *pixels);

    bool project_with_global_shutter(const PointCloud &cloud,
            const Eigen::Quaterniond &cloud2cam_quat, const Eigen::Vector3d &cloud2cam_trans,
            PointCloud* inliers, PointCloud* outliers, std::vector<ImagePixel> *image_pixels);

    bool project_with_rolling_shutter(const double &cloud_stamp,
            const PointCloud &cloud, const std::vector<ImagePixel> &pixels,
            const double &first_stamp, const double &last_stamp,
            std::vector<ImagePixel> *image_pixels);

    bool filter_cloud(const PointCloud& cloud, const std::vector<ImagePixel> &pixels,
        int width, int height, PointCloud *inliers, PointCloud* outliers,
        std::vector<ImagePixel> *out_pixels);

private:
    bool get_transformation_cloud2cam(const double &cam_stamp, const double &cloud_stamp,
            Eigen::Quaterniond *cloud2cam_quat, Eigen::Vector3d *cloud2cam_trans);

    void transformation_cloud2cam(
            const Eigen::Quaterniond &camera_quat, const Eigen::Vector3d &camera_trans,
            const Eigen::Quaterniond &cloud_quat, const Eigen::Vector3d &cloud_trans,
            Eigen::Quaterniond *cloud2cam_quat, Eigen::Vector3d *cloud2cam_trans);

    bool load_extrinsic(const std::string& file_path, Eigen::Affine3d* extrinsic);

private:
    CloudProjectorConfig _config;

    Eigen::Affine3d _lidar_to_vehicle_extrinsic = Eigen::Affine3d::Identity();
    Eigen::Affine3d _cam_to_vehicle_extrinsic = Eigen::Affine3d::Identity();
    CameraIntrinsics _cam_intrinsic;

    PosesInterpolation _pose_buffer;

    bool _is_paint_visual_img = true;

    CameraModel _camera_model;
    double _shutter_time_per_line = 42.1875 * 1e-6;

private:
    cv::Mat _rendered_image;
};

}  // namespace demo