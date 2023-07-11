#include "timestamp_loader.h"

#include <fstream>

namespace demo {

bool TimestampDataLoader::load_data(const TimestampDataConfig &config) {
    _timestamp_infos.resize(0);

    _config = config;

    if (!parse_stamp_file(_config.timestamp_file, &_timestamp_infos)) {
        std::cerr << "Load timestamps failed." << std::endl;
        return false;
    }

    return true;
}

size_t TimestampDataLoader::timestamp_size() {
    return _timestamp_infos.size();
}

size_t TimestampDataLoader::find_nearest_timestamp(const double &stamp) {
    double min_delta_stamp = 1e20;
    size_t target_index = 0;

    for (size_t idx = 0; idx < _timestamp_infos.size(); ++idx) {
        const auto &elm = _timestamp_infos[idx];
        double delta_stamp = std::abs(elm.timestamp - stamp);
        if (delta_stamp < min_delta_stamp) {
            min_delta_stamp = delta_stamp;
            target_index = idx;
        }
    }
    return target_index;
}

bool TimestampDataLoader::get_stamp_info(size_t idx, TimestampInfo *info) {
    if (info == nullptr) {
        return false;
    }

    if (idx >= _timestamp_infos.size()) {
        return false;
    }

    *info = _timestamp_infos.at(idx);
    return true;
}

bool TimestampDataLoader::parse_stamp_file(
        const std::string &path, TimestampInfoVector *infos) {
    std::string file_path = path;
    std::fstream stamps_stream(file_path, std::ios::in);
    if (!stamps_stream.is_open()) {
        return false;
    }

    std::string name;
    double timestamp = 0.0;
    while (!stamps_stream.eof()) {
        std::string line_str;
        std::getline(stamps_stream, line_str);
        if (!line_str.empty()) {
            std::stringstream str_stream;
            str_stream << line_str;
            // read from stamp file
            str_stream >> name >> timestamp;

            TimestampInfo data;
            data.timestamp = timestamp;
            data.name = name;
            infos->push_back(data);
        }
    }
    return true;
}

}  // namespace demo
