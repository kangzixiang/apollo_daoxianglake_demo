#include "poses_interpolation.h"

#include <fstream>
#include <iomanip>

namespace demo {

bool PosesInterpolation::init(const std::string& input_poses_file) {
    _ref_poses_file = input_poses_file;

    // Load input poses   
    if (!load_ref_poses(_ref_poses_file)) {
        std::cerr << "Load ref poses failed" << std::endl;
        return false;
    }

    std::cerr << "Load ref poses size: " << _ref_poses_map.size() << std::endl;
    return true;
}

bool PosesInterpolation::query_pose(const double &timestamp, Eigen::Affine3d *out_pose) const {
    // Interpolation
    return pose_interpolation_by_time(_ref_poses_map, timestamp, out_pose);
}

bool PosesInterpolation::query_pose(const double &timestamp,
        const Eigen::Affine3d &extrinsic, Eigen::Affine3d *out_pose) const {
    // Interpolation
    Eigen::Affine3d tmp_pose;
    bool suc = pose_interpolation_by_time(_ref_poses_map, timestamp, &tmp_pose);

    if (suc == false) {
        return false;
    }

    *out_pose = tmp_pose * extrinsic;
    return true;
}

bool PosesInterpolation::load_ref_poses(const std::string &file) {
    std::fstream poses_stream;
    poses_stream.open(file);
    if (!poses_stream.is_open()) {
        return false;
    }

    unsigned int index = 0;
    double stamp = 0.0;
    double coord[3] = {0.0};
    double quat[4] = {0.0};
    while (!poses_stream.eof()) {
        std::string str;
        std::getline(poses_stream, str);

        constexpr int k_size = 8;
        
        if (sscanf(str.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf\n", 
               &stamp, &coord[0], &coord[1], &coord[2],
               &quat[0], &quat[1], &quat[2], &quat[3]) == k_size) {
            Eigen::Translation3d trans(Eigen::Vector3d(coord[0], coord[1], coord[2]));
            Eigen::Quaterniond quat_eigen(quat[3], quat[0], quat[1], quat[2]);
            auto pose = trans * quat_eigen;
            _ref_poses_map[stamp] = pose;
        }
    }
    poses_stream.close();
    return true;
}

bool PosesInterpolation::pose_interpolation_by_time(
        const Affine3dMap &in_poses_map,
        const double &ref_timestamp,
        Eigen::Affine3d *out_pose) const {
    auto upper_iter = in_poses_map.lower_bound(ref_timestamp);

    if (upper_iter == in_poses_map.begin()) {
        if (std::abs(upper_iter->first - ref_timestamp) < 1e-4) {
            *out_pose = upper_iter->second;
            return true;
        }
        return false;
    }

    auto lower_iter = upper_iter;
    --lower_iter;

    if (upper_iter == in_poses_map.end()) {
        if (std::abs(lower_iter->first - ref_timestamp) < 1e-4) {
            *out_pose = lower_iter->second;
            return true;
        }
        return false;
    }

    // do interpolation
    double cur_timestamp = upper_iter->first;
    double pre_timestamp = lower_iter->first;
    double t = (cur_timestamp - ref_timestamp) / (cur_timestamp - pre_timestamp);
    // std::cout << std::setprecision(15) << "Do interpolation: " << cur_timestamp << " " << pre_timestamp << " " << t << std::endl;
    assert(t >= 0.0);
    assert(t <= 1.0);

    Eigen::Affine3d cur_pose = upper_iter->second;
    Eigen::Affine3d pre_pose = lower_iter->second;
    Eigen::Quaterniond pre_quatd(pre_pose.linear());
    Eigen::Translation3d pre_transd(pre_pose.translation());
    Eigen::Quaterniond cur_quatd(cur_pose.linear());
    Eigen::Translation3d cur_transd(cur_pose.translation());

    Eigen::Quaterniond res_quatd = pre_quatd.slerp(1 - t, cur_quatd);

    Eigen::Translation3d re_transd;
    re_transd.x() = pre_transd.x() * t + cur_transd.x() * (1 - t);
    re_transd.y() = pre_transd.y() * t + cur_transd.y() * (1 - t);
    re_transd.z() = pre_transd.z() * t + cur_transd.z() * (1 - t);

    *out_pose = re_transd * res_quatd;
    return true;
}

}  // namespace demo
