#pragma once

#include "gflags/gflags.h"

DECLARE_string(odometry_pose_file);
DECLARE_string(lidar2vehicle_extrinsic_file);
DECLARE_string(cam2vehicle_extrinsic_file);
DECLARE_string(cam_intrinsic_file);

DECLARE_string(point_cloud_stamp_file);
DECLARE_string(point_cloud_folder);

DECLARE_string(image_stamp_file);
DECLARE_string(image_folder);

DECLARE_double(rolling_shutter_time);

DECLARE_double(lidar_align_stamp_offset);