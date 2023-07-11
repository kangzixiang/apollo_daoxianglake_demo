#pragma once

#include <stdint.h>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "intrinsics.h"

namespace demo {

class CameraModel {
public:
    bool check_model(const CameraIntrinsics& intrinsic);

    void proj_3d_to_2d(const CameraIntrinsics& intrinsic,
            const Eigen::Vector3d &pt3d, float *pix_x, float *pix_y);

    void distortion(const CameraIntrinsics& intrinsic,
            const double &u, const double &v,
            double* refined_u, double* refined_v);

private:
    inline void distort(const double params[4],
            const double &u, const double &v,
            double* refined_u, double* refined_v);
};

void CameraModel::distort(const double params[4],
        const double &u, const double &v, double* refined_u, double* refined_v) {
    const double& k1 = params[0];
    const double& k2 = params[1];
    const double& p1 = params[2];
    const double& p2 = params[3];

    const double u2 = u * u;
    const double uv = u * v;
    const double v2 = v * v;
    const double r2 = u2 + v2;
    const double radial = k1 * r2 + k2 * r2 * r2;
    double du = u * radial + double(2) * p1 * uv + p2 * (r2 + double(2) * u2);
    double dv = v * radial + double(2) * p2 * uv + p1 * (r2 + double(2) * v2);
    *refined_u = u + du;
    *refined_v = v + dv;
    return;
}

}  // namespace demo