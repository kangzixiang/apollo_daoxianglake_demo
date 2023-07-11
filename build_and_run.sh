#!/bin/bash
cd "$(dirname $0)"

echo "Configuring and building Demo ..."

cd src
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2
cd ../..

echo "Run Demo ..."

Root=./demo_data
Date=20190918143332

Camera_type=front
./src/build/project_with_rolling_shutter \
        --odometry_pose_file=$Root/gt_pose_data/$Date/vechicle_poses.txt \
        --lidar2vehicle_extrinsic_file=$Root/params/lidar_extrinsic.yaml \
        --cam2vehicle_extrinsic_file=$Root/params/${Camera_type}_camera_extrinsic.yaml \
        --cam_intrinsic_file=$Root/params/${Camera_type}_camera_intrinsic.yaml \
        --point_cloud_stamp_file=$Root/pcd_data/$Date/pcd_stamps.txt \
        --point_cloud_folder=$Root/pcd_data/$Date/ \
        --image_stamp_file=$Root/image_data/$Date/$Camera_type/image_stamps.txt \
        --image_folder=$Root/image_data/$Date/$Camera_type/

Camera_type=left_back
./src/build/project_with_rolling_shutter \
        --odometry_pose_file=$Root/gt_pose_data/$Date/vechicle_poses.txt \
        --lidar2vehicle_extrinsic_file=$Root/params/lidar_extrinsic.yaml \
        --cam2vehicle_extrinsic_file=$Root/params/${Camera_type}_camera_extrinsic.yaml \
        --cam_intrinsic_file=$Root/params/${Camera_type}_camera_intrinsic.yaml \
        --point_cloud_stamp_file=$Root/pcd_data/$Date/pcd_stamps.txt \
        --point_cloud_folder=$Root/pcd_data/$Date/ \
        --image_stamp_file=$Root/image_data/$Date/$Camera_type/image_stamps.txt \
        --image_folder=$Root/image_data/$Date/$Camera_type/

Camera_type=right_back
./src/build/project_with_rolling_shutter \
        --odometry_pose_file=$Root/gt_pose_data/$Date/vechicle_poses.txt \
        --lidar2vehicle_extrinsic_file=$Root/params/lidar_extrinsic.yaml \
        --cam2vehicle_extrinsic_file=$Root/params/${Camera_type}_camera_extrinsic.yaml \
        --cam_intrinsic_file=$Root/params/${Camera_type}_camera_intrinsic.yaml \
        --point_cloud_stamp_file=$Root/pcd_data/$Date/pcd_stamps.txt \
        --point_cloud_folder=$Root/pcd_data/$Date/ \
        --image_stamp_file=$Root/image_data/$Date/$Camera_type/image_stamps.txt \
        --image_folder=$Root/image_data/$Date/$Camera_type/