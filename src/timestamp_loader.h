#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace demo {

struct TimestampDataConfig {
    std::string timestamp_file = "";
};

struct TimestampInfo {
    std::string name;
    double timestamp = 0.0;
};

class TimestampDataLoader {
public:
    typedef std::vector<TimestampInfo> TimestampInfoVector;

    TimestampDataLoader() {}
    ~TimestampDataLoader() {}

    bool load_data(const TimestampDataConfig &config);

    size_t timestamp_size();
    size_t find_nearest_timestamp(const double &stamp);

    bool get_stamp_info(size_t idx, TimestampInfo *info);

private:
    bool parse_stamp_file(const std::string &path, TimestampInfoVector *data);

private:
    TimestampDataConfig _config;
    TimestampInfoVector _timestamp_infos;
};

}  // namespace demo
