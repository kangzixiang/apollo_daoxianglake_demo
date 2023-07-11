#include "rendering_gflags.h"

DEFINE_string(odometry_pose_file, "",
              "odometry pose file path");

DEFINE_string(lidar2vehicle_extrinsic_file, "",
              "lidar to vehicle extrinsic file path");

DEFINE_string(cam2vehicle_extrinsic_file, "",
              "camera to vehicle extrinsic file path");

DEFINE_string(cam_intrinsic_file, "",
              "camera intrinsic file path");

DEFINE_string(point_cloud_stamp_file, "",
              "point cloude timestamp file path");

DEFINE_string(point_cloud_folder, "",
              "point cloud folder");

DEFINE_string(image_stamp_file, "",
              "image timestamp file path");

DEFINE_string(image_folder, "",
              "image folder");

DEFINE_double(rolling_shutter_time, 0.0455625,
              "rolling shutter time");

DEFINE_double(lidar_align_stamp_offset, -0.05, "lidar align stamp offset");
