#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <map>

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace demo {

typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> Affine3dVector;
typedef std::map<double, Eigen::Affine3d, std::less<double>, 
        Eigen::aligned_allocator<std::pair<const double, Eigen::Affine3d> > > Affine3dMap;

class PosesInterpolation {
public:
    PosesInterpolation() {}

    bool init(const std::string &input_poses_file);

    bool query_pose(const double &timestamp, Eigen::Affine3d *out_pose) const;
    bool query_pose(const double &timestamp, const Eigen::Affine3d &extrinsic, Eigen::Affine3d *out_pose) const;
    
private:
    bool load_ref_poses(const std::string &file);

    bool pose_interpolation_by_time(
            const Affine3dMap &in_poses_map,
            const double &ref_timestamp,
            Eigen::Affine3d *out_pose) const;

private:
    std::string _ref_poses_file;
    Affine3dMap _ref_poses_map;
};

}  // namespace demo
