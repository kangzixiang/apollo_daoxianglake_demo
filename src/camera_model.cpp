#include "camera_model.h"

namespace demo {

bool CameraModel::check_model(const CameraIntrinsics& intrinsic) {
    return intrinsic.distortion_model == "plumb_bob";
}

void CameraModel::proj_3d_to_2d(const CameraIntrinsics& intrinsic,
        const Eigen::Vector3d &pt3d, float *pix_x, float *pix_y) {
    double inv_z = 1.0 / pt3d[2];
    double px = pt3d[0] * inv_z;
    double py = pt3d[1] * inv_z;

    double distortion_params[4] = {};
    distortion_params[0] = intrinsic.dist_params[0];  // k1
    distortion_params[1] = intrinsic.dist_params[1];  // k2
    distortion_params[2] = intrinsic.dist_params[2];  // p1
    distortion_params[3] = intrinsic.dist_params[3];  // p2
    distort(distortion_params, px, py, &px, &py);

    double fx = intrinsic.cam_mat[0];
    double cx = intrinsic.cam_mat[2];
    double fy = intrinsic.cam_mat[4];
    double cy = intrinsic.cam_mat[5];

    *pix_x = fx * px + cx;
    *pix_y = fy * py + cy;
}

void CameraModel::distortion(const CameraIntrinsics& intrinsic,
        const double &u, const double &v, double* refined_u, double* refined_v) {
    double distortion_params[4] = {};
    distortion_params[0] = intrinsic.dist_params[0];  // k1
    distortion_params[1] = intrinsic.dist_params[1];  // k2
    distortion_params[2] = intrinsic.dist_params[2];  // p1
    distortion_params[3] = intrinsic.dist_params[3];  // p2
    distort(distortion_params, u, v, refined_u, refined_v);
}

}  // namespace demo