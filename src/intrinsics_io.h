#pragma once

#include <stdint.h>
#include <string>

#include "intrinsics.h"

namespace demo {

/**
 * @brief Read camera intrinsics from yaml file
 * @param filename      The input path to yaml file
 * @param intrinsics    The output camera intrinsics
 * @return 0 if succeeded
 */
bool read_camera_intrinsics_yaml(const std::string& filename, CameraIntrinsics& intrinsics);

}  // namespace demo
