#include "point_cloud.h"

#include <pcl/io/pcd_io.h>

namespace demo {

struct PointXYZIT {
    float x;
    float y;
    float z;
    unsigned char intensity;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment

}  // namespace demo

POINT_CLOUD_REGISTER_POINT_STRUCT(
        demo::PointXYZIT,
        (float, x, x)(float, y, y)(float, z, z)
        (uint8_t, intensity, intensity)
        (double, timestamp, timestamp))

namespace demo {

bool load_pcd(const std::string& file_path, PointCloud *pointcloud) {
    pointcloud->resize(0);

    pcl::PointCloud<PointXYZIT>::Ptr cloud(new pcl::PointCloud<PointXYZIT>);
    if (pcl::io::loadPCDFile(file_path, *cloud) >= 0) {
        for (unsigned int i = 0; i < cloud->size(); ++i) {
            Eigen::Vector3d pt3d;
            pt3d[0] = (*cloud)[i].x;
            pt3d[1] = (*cloud)[i].y;
            pt3d[2] = (*cloud)[i].z;

            CloudPoint pt;
            pt.pt3d = pt3d;
            pt.intensity = static_cast<unsigned char>((*cloud)[i].intensity);
            pt.timestamp = static_cast<double>((*cloud)[i].timestamp);
            pointcloud->push_back(pt);
        }
        return true;
    } else {
        std::cout << "Failed to load PCD file: " << file_path << std::endl;
        return false;
    }
}

}  // namespace demo
