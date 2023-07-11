#pragma once

#include <array>
#include <stdint.h>
#include <string>
#include <vector>

namespace demo {

/**
 * @brief Camera instrinsics struct
 */
struct CameraIntrinsics {
    CameraIntrinsics() : height(0), width(0), distortion_model("plumb_bob") {
        dist_params.resize(5, 0.0);
        cam_mat.fill(0.0);
    }

    uint32_t height;
    uint32_t width;

    std::string distortion_model;
    std::vector<double> dist_params;

    std::array<double, 9> cam_mat;
};

}  // namespace demo
