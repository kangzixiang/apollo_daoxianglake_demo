#include "cloud_projector.h"

#include <fstream>
#include "yaml-cpp/yaml.h"

#include "point_cloud.h"

namespace demo {

bool CloudProjector::init(const CloudProjectorConfig &config) {
    _config = config;

    if (!load_extrinsic(_config.lidar2vehicle_extrinsic_file, &_lidar_to_vehicle_extrinsic)) {
        std::cerr << "Load lidar2vehicle extrinsic failed." << std::endl;
        return false;
    }
    if (!load_extrinsic(_config.cam2vehicle_extrinsic_file, &_cam_to_vehicle_extrinsic)) {
        std::cerr << "Load cam2vehicle extrinsic failed." << std::endl;
        return false;
    }

    if (!read_camera_intrinsics_yaml(_config.cam_intrinsic_file, _cam_intrinsic)) {
        std::cerr << "Load camera intrinsic failed." << std::endl;
        return false;
    }

    _shutter_time_per_line = _config.rolling_shutter_time / _cam_intrinsic.height;

    _pose_buffer.init(_config.odometry_pose_file);

    return true;
}

bool CloudProjector::project_cloud_to_image(
        const double &img_stamp, const cv::Mat &img,
        const double &cloud_stamp, const PointCloud &cloud) {
    PointCloud inliers;
    PointCloud outliers;
    std::vector<ImagePixel> pixels;
    if (!project_rolling_shutter(
            img_stamp, img, cloud_stamp, cloud, &inliers, &outliers, &pixels)) {
        return false;
    }

    // visualizer
    const double max_depth  = 60.0;
    const float ratio = 1.0;
    _rendered_image = img.clone();
    cv::resize(_rendered_image, _rendered_image, cv::Size(), ratio, ratio);
    for (size_t idx = 0; idx < pixels.size(); ++idx) {
        const auto &pix = pixels[idx];
        float depth = pix.depth > max_depth ? max_depth : pix.depth;
        depth = depth / max_depth;
        unsigned char r = 255.0 * JetColormap::red(depth);
        unsigned char g = 255.0 * JetColormap::green(depth);
        unsigned char b = 255.0 * JetColormap::blue(depth);
        cv::circle(_rendered_image, cv::Point(pix.x * ratio, pix.y * ratio), 2, cv::Scalar(b, g, r), -1);
    }

    return true;
}

bool CloudProjector::project_rolling_shutter(
            const double &img_stamp, const cv::Mat &img,
            const double &cloud_stamp, const PointCloud &cloud,
            PointCloud *inliers, PointCloud *outliers, std::vector<ImagePixel> *refined_pixels) {
    int stride = _cam_intrinsic.height;
    double img_last_stamp = img_stamp;
    double img_first_stamp = img_last_stamp - stride * _shutter_time_per_line;

    double img_middle_stamp = (img_first_stamp + img_last_stamp) * 0.5;

    Eigen::Quaterniond cloud2cam_quat;
    Eigen::Vector3d cloud2cam_trans;
    get_transformation_cloud2cam(img_middle_stamp, cloud_stamp, &cloud2cam_quat, &cloud2cam_trans);

    // filter cloud
    PointCloud tmp_cloud;
    std::vector<ImagePixel> tmp_pixels;
    project_with_global_shutter(
            cloud, cloud2cam_quat, cloud2cam_trans,
            &tmp_cloud, outliers, &tmp_pixels);
    std::cerr << "inlier/total: " << tmp_cloud.size() << " / " << cloud.size() << std::endl;

    // project with rolling shutter effect
    std::vector<ImagePixel> tmp_refined_pixels;
    project_with_rolling_shutter(cloud_stamp, tmp_cloud, tmp_pixels, img_first_stamp,
                                 img_last_stamp, &tmp_refined_pixels);

    std::cerr << "inlier/total: " << tmp_cloud.size() << " / " << cloud.size() << std::endl;
    
    filter_cloud(tmp_cloud, tmp_refined_pixels, _cam_intrinsic.width, _cam_intrinsic.height,
            inliers, outliers, refined_pixels);
    return true;
}

cv::Mat CloudProjector::get_rendered_image() {
    return _rendered_image;
}

bool CloudProjector::load_extrinsic(const std::string& file_path, Eigen::Affine3d* extrinsic) {
    YAML::Node config = YAML::LoadFile(file_path);

    if (config["transform"]) {
        if (config["transform"]["translation"]) {
            extrinsic->translation()(0) =
                config["transform"]["translation"]["x"].as<double>();
            extrinsic->translation()(1) =
                config["transform"]["translation"]["y"].as<double>();
            extrinsic->translation()(2) =
                config["transform"]["translation"]["z"].as<double>();

            if (config["transform"]["rotation"]) {
                double qx = config["transform"]["rotation"]["x"].as<double>();
                double qy = config["transform"]["rotation"]["y"].as<double>();
                double qz = config["transform"]["rotation"]["z"].as<double>();
                double qw = config["transform"]["rotation"]["w"].as<double>();
                extrinsic->linear() =
                    Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
                return true;
            }
        }
    }

    return false;
}

bool CloudProjector::project_with_global_shutter(
        const PointCloud &cloud, const Eigen::Quaterniond &cloud2cam_quat,
        const Eigen::Vector3d &cloud2cam_trans, PointCloud* inliers,
        PointCloud* outliers, std::vector<ImagePixel> *image_pixels) {
    uint32_t height = _cam_intrinsic.height;
    uint32_t width = _cam_intrinsic.width;

    for (size_t idx = 0; idx < cloud.size(); ++idx) {
        const auto &pt = cloud[idx];
        const auto &pt3d = pt.pt3d;
        const auto &intensity = pt.intensity;
        const Eigen::Vector3d cam_pt3d =  cloud2cam_quat * pt3d + cloud2cam_trans;
        
        if (cam_pt3d[2] < 0.2) {
            outliers->push_back(pt);
            continue;
        }

        ImagePixel pix;
        pix.depth = cam_pt3d[2];
        _camera_model.proj_3d_to_2d(_cam_intrinsic, cam_pt3d, &pix.x, &pix.y);

        if (pix.x > 0.0 && pix.x < width && pix.y > 0.0 && pix.y < height) {
            inliers->push_back(pt);
            image_pixels->push_back(pix);
        } else {
            outliers->push_back(pt);
        }
    }

    return true;
}

bool CloudProjector::filter_cloud(const PointCloud& cloud,
        const std::vector<ImagePixel> &pixels, int width, int height, 
        PointCloud *inliers, PointCloud* outliers, std::vector<ImagePixel> *out_pixels) {

    for (size_t idx = 0; idx < cloud.size(); ++idx) {
        const auto &pt = cloud[idx];
        const auto &pt3d = pt.pt3d;
        const auto &intensity = pt.intensity;
        const auto &pix = pixels[idx];
        // nearest neighbor
        int row = pix.y + 0.5;
        int col = pix.x + 0.5;
        if (col >= 0 && col < width && row >= 0 && row < height) {
            inliers->push_back(pt);
            out_pixels->push_back(pix);
            
        } else {
            outliers->push_back(pt);
        }
    }
    return true;
}

bool CloudProjector::project_with_rolling_shutter(const double &cloud_stamp,
            const PointCloud &cloud, const std::vector<ImagePixel> &pixels,
            const double &first_stamp, const double &last_stamp,
            std::vector<ImagePixel> *image_pixels) {
    uint32_t height = _cam_intrinsic.height;
    uint32_t width = _cam_intrinsic.width;

    const int sample_size = 256;
    const float ratio = float(sample_size) / height;

    double stamp_stride = (last_stamp - first_stamp) / sample_size;
    
    std::vector<Eigen::Quaterniond,
                Eigen::aligned_allocator<Eigen::Quaterniond>> quat_array(sample_size);
    std::vector<Eigen::Vector3d> trans_array(sample_size);
    for (uint32_t idx = 0; idx < sample_size; ++idx) {
        double cur_stamp = first_stamp + stamp_stride * (idx + 0.5);
        get_transformation_cloud2cam(cur_stamp, cloud_stamp, &quat_array[idx], &trans_array[idx]);
    }

    const int itr_num = 2;
    *image_pixels = pixels;
    for (int itr = 0; itr < itr_num; ++itr) {
        for (size_t idx = 0; idx < cloud.size(); ++idx) {
            const auto &pt = cloud[idx];
            const auto &pt3d = pt.pt3d;
            const auto &intensity = pt.intensity;
            const auto &pix = (*image_pixels)[idx];

            // Use pose from current image line
            int row = pix.y * ratio;
            if (row < 0) {
                row = 0;
            }
            if (row > sample_size - 1) {
                row = sample_size - 1;
            }
            const Eigen::Vector3d cam_pt3d = quat_array[row] * pt3d + trans_array[row];

            ImagePixel refined_pix;
            if (cam_pt3d[2] > 0.2) {
                refined_pix.depth = cam_pt3d[2];
                _camera_model.proj_3d_to_2d(
                        _cam_intrinsic, cam_pt3d, &refined_pix.x, &refined_pix.y);
            }
            (*image_pixels)[idx] = refined_pix;
        }
    }

    return true;
}

bool CloudProjector::get_transformation_cloud2cam(
        const double &cam_stamp, const double &cloud_stamp,
        Eigen::Quaterniond *cloud2cam_quat, Eigen::Vector3d *cloud2cam_trans) {
    Eigen::Affine3d img_pose;
    bool suc = _pose_buffer.query_pose(cam_stamp, _cam_to_vehicle_extrinsic, &img_pose);
    if (suc == false) {
        return false;
    }

    auto img_quat = Eigen::Quaterniond(img_pose.linear());
    Eigen::Vector3d img_trans;
    img_trans[0] = img_pose.translation()(0);
    img_trans[1] = img_pose.translation()(1);
    img_trans[2] = img_pose.translation()(2);

    Eigen::Affine3d cloud_pose;
    suc = _pose_buffer.query_pose(cloud_stamp, _lidar_to_vehicle_extrinsic, &cloud_pose);
    if (suc == false) {
        return false;
    }
    auto cloud_quat = Eigen::Quaterniond(cloud_pose.linear());
    Eigen::Vector3d cloud_trans;
    cloud_trans[0] = cloud_pose.translation()(0);
    cloud_trans[1] = cloud_pose.translation()(1);
    cloud_trans[2] = cloud_pose.translation()(2);


    transformation_cloud2cam(img_quat, img_trans, cloud_quat,
            cloud_trans, cloud2cam_quat, cloud2cam_trans);
    return true;
}

void CloudProjector::transformation_cloud2cam(
        const Eigen::Quaterniond &camera_quat, const Eigen::Vector3d &camera_trans,
        const Eigen::Quaterniond &cloud_quat, const Eigen::Vector3d &cloud_trans,
        Eigen::Quaterniond *cloud2cam_quat, Eigen::Vector3d *cloud2cam_trans) {
    *cloud2cam_quat = camera_quat.inverse() * cloud_quat;
    *cloud2cam_trans = camera_quat.inverse() * (cloud_trans -  camera_trans);
    return;
}

}  // namesapce demo
