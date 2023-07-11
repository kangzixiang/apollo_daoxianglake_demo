#include "intrinsics_io.h"

#include <fstream>
#include <iostream>

#include "yaml-cpp/yaml.h"

namespace demo {

bool read_camera_intrinsics_yaml(const std::string& filename, CameraIntrinsics& intrinsics) {
    std::ifstream fin(filename.c_str());
    if (!fin) {
        std::cerr << "Failed to open file \"" + filename + "\"!" << std::endl;
        return false;
    }

    YAML::Node root = YAML::Load(fin);

    // image dimension
    intrinsics.height = root["height"].as<uint32_t>();
    intrinsics.width = root["width"].as<uint32_t>();

    // image distortion
    intrinsics.distortion_model = root["distortion_model"].as<std::string>();
    if (intrinsics.distortion_model != "plumb_bob") {
        std::cerr << "model_type wrong: " << intrinsics.distortion_model << std::endl;
        return false;
    }

    YAML::Node node_dist_params = root["D"];
    intrinsics.dist_params.resize(node_dist_params.size());
    for (int i = 0; i < node_dist_params.size(); ++i) {
        intrinsics.dist_params[i] = node_dist_params[i].as<double>();
    }

    // camera matrix
    YAML::Node node_cam_mat = root["K"];
    if (node_cam_mat.size() != 9) {
        std::cerr << "Camera matrix must have 9 elements!" << std::endl;
        return false;
    }
    for (int i = 0; i < 9; ++i) {
        intrinsics.cam_mat[i] = node_cam_mat[i].as<double>();
    }

    fin.close();
    return true;
}

}  // namespace demo
