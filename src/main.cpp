#include <iomanip>
#include <iostream>
#include <string>

#include <Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "opencv2/opencv.hpp"
#include "yaml-cpp/yaml.h"

#include "timestamp_loader.h"
#include "cloud_projector.h"

#include "rendering_gflags.h"

#include "opencv2/imgcodecs/legacy/constants_c.h"

typedef demo::TimestampDataConfig TimestampDataConfig;
typedef demo::TimestampInfo TimestampInfo;
typedef demo::TimestampDataLoader TimestampDataLoader;
typedef demo::PointCloud PointCloud;
typedef demo::CloudProjectorConfig CloudProjectorConfig;
typedef demo::CloudProjector CloudProjector;

void generate_cloud_projector_config(CloudProjectorConfig *config) {
    config->odometry_pose_file = FLAGS_odometry_pose_file;
    config->lidar2vehicle_extrinsic_file = FLAGS_lidar2vehicle_extrinsic_file;
    config->cam2vehicle_extrinsic_file = FLAGS_cam2vehicle_extrinsic_file;
    config->cam_intrinsic_file = FLAGS_cam_intrinsic_file;
    config->rolling_shutter_time = FLAGS_rolling_shutter_time;
    return;
}

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    // Load image timestamp file
    TimestampDataConfig image_data_config;
    image_data_config.timestamp_file = FLAGS_image_stamp_file;
    TimestampDataLoader image_data_loader;
    if (!image_data_loader.load_data(image_data_config)) {
        std::cerr << "Load image timestamp file failed." << std::endl;
        return -1;
    }

    // Load point cloud timestamp file
    TimestampDataConfig cloud_data_config;
    cloud_data_config.timestamp_file = FLAGS_point_cloud_stamp_file;
    TimestampDataLoader cloud_data_loader;
    if (!cloud_data_loader.load_data(cloud_data_config)) {
        std::cerr << "Load point cloud timestamp file failed." << std::endl;
        return -1;
    }

    // Init point cloud projector
    CloudProjectorConfig cloud_project_config;
    generate_cloud_projector_config(&cloud_project_config);
    CloudProjector cloud_projector;
    if (!cloud_projector.init(cloud_project_config)) {
        std::cerr << "Init CloudProjector failed." << std::endl;
        return -1;
    }
    
    int pair_num = 0;
    int fail_load_image_num = 0;
    int fail_load_cloud_num = 0;
    for (size_t cloud_idx = 0; cloud_idx < cloud_data_loader.timestamp_size(); ++cloud_idx) {
        TimestampInfo cloud_info;

        if (!cloud_data_loader.get_stamp_info(cloud_idx, &cloud_info)) {
            continue;
        }

        // Move timestamp from the end of a LiDAR frame to middle of a LiDAR frame.
        double dst_stamp = cloud_info.timestamp + FLAGS_lidar_align_stamp_offset;
        auto img_idx = image_data_loader.find_nearest_timestamp(dst_stamp);

        TimestampInfo img_info;
        if (!image_data_loader.get_stamp_info(img_idx, &img_info)) {
            continue;
        }

        std::cout << "Do Projection, img name: " << img_info.name
                  << " cloud name: " << cloud_info.name << std::endl;


        std::string img_path = FLAGS_image_folder + "/" + img_info.name + ".jpg";
        cv::Mat img = cv::imread(img_path, CV_LOAD_IMAGE_COLOR);

        std::string cloud_path = FLAGS_point_cloud_folder + "/" + cloud_info.name + ".pcd";
        PointCloud cloud;
        if (!load_pcd(cloud_path, &cloud)) {
            std::cerr << "Load cloud failed." << std::endl;
            continue;
        }

        if (cloud_projector.project_cloud_to_image(
                img_info.timestamp, img, cloud_info.timestamp, cloud)) {
            cv::Mat projected_mat = cloud_projector.get_rendered_image();
            cv::imshow("projection", projected_mat);
            cv::waitKey(20);
        } else {
            std::cerr << "Project cloud to image failed." << std::endl;
        }
    }

    return 0;
}
