#ifndef GRAPH_OPTIMIZATION_ONLINE_LOGGER_HPP
#define GRAPH_OPTIMIZATION_ONLINE_LOGGER_HPP

#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace graph_optimization {

struct OnlineLogEntry {
    int submap_id = -1;
    double pseudo_time_idx = 0.0;
    std::size_t ping_count = 0;
    Eigen::Isometry3f est_pose = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f gt_pose = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f dr_pose = Eigen::Isometry3f::Identity();
};

class OnlineLogWriter {
public:
    OnlineLogWriter() = default;
    OnlineLogWriter(std::string log_dir, std::string ping_csv_path);

    void addEntry(const OnlineLogEntry& entry);
    void writeRawLog(const std::string& filename) const;
    void writePingErrorCsv() const;

    const std::vector<OnlineLogEntry>& entries() const { return entries_; }
    const std::string& logDir() const { return log_dir_; }
    const std::string& pingCsvPath() const { return ping_csv_path_; }

private:
    std::vector<OnlineLogEntry> entries_;
    std::string log_dir_;
    std::string ping_csv_path_;
};

} // namespace graph_optimization

#endif // GRAPH_OPTIMIZATION_ONLINE_LOGGER_HPP

