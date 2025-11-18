#include "graph_optimization/online_logger.hpp"

#include <algorithm>
#include <boost/filesystem.hpp>
#include <cmath>
#include <fstream>

namespace graph_optimization {
namespace {

constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

double extractYaw(const Eigen::Isometry3f& pose) {
    const Eigen::Matrix3f& rot = pose.rotation();
    return std::atan2(rot(1, 0), rot(0, 0));
}

double horizontalError(const Eigen::Isometry3f& a, const Eigen::Isometry3f& b) {
    Eigen::Vector3f diff = a.translation() - b.translation();
    return std::sqrt(diff.x() * diff.x() + diff.y() * diff.y());
}

double yawError(const Eigen::Isometry3f& estimate, const Eigen::Isometry3f& gt) {
    return normalizeAngle(extractYaw(estimate) - extractYaw(gt));
}

void ensureDirectory(const boost::filesystem::path& file_path) {
    if (!file_path.parent_path().empty()) {
        boost::filesystem::create_directories(file_path.parent_path());
    }
}

} // namespace

OnlineLogWriter::OnlineLogWriter(std::string log_dir, std::string ping_csv_path)
    : log_dir_(std::move(log_dir)),
      ping_csv_path_(std::move(ping_csv_path)) {}

void OnlineLogWriter::addEntry(const OnlineLogEntry& entry) {
    entries_.push_back(entry);
}

void OnlineLogWriter::writeRawLog(const std::string& filename) const {
    if (entries_.empty()) {
        return;
    }
    boost::filesystem::path file_path(filename);
    ensureDirectory(file_path);
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        return;
    }
    ofs << "submap_id,pseudo_idx,ping_count,"
           "est_x,est_y,est_z,gt_x,gt_y,gt_z,dr_x,dr_y,dr_z\n";
    for (const auto& entry : entries_) {
        const Eigen::Vector3f est = entry.est_pose.translation();
        const Eigen::Vector3f gt = entry.gt_pose.translation();
        const Eigen::Vector3f dr = entry.dr_pose.translation();
        ofs << entry.submap_id << ","
            << entry.pseudo_time_idx << ","
            << entry.ping_count << ","
            << est.x() << "," << est.y() << "," << est.z() << ","
            << gt.x() << "," << gt.y() << "," << gt.z() << ","
            << dr.x() << "," << dr.y() << "," << dr.z() << "\n";
    }
}

void OnlineLogWriter::writePingErrorCsv() const {
    if (entries_.empty()) {
        return;
    }
    boost::filesystem::path file_path(ping_csv_path_);
    ensureDirectory(file_path);
    std::ofstream ofs(ping_csv_path_);
    if (!ofs.is_open()) {
        return;
    }
    ofs << "ping_index,err_xy,err_yaw,err_xy_dr,err_yaw_dr,source_submap_id\n";
    for (const auto& entry : entries_) {
        const double err_xy = horizontalError(entry.est_pose, entry.gt_pose);
        const double err_yaw = yawError(entry.est_pose, entry.gt_pose);
        const double err_xy_dr = horizontalError(entry.dr_pose, entry.gt_pose);
        const double err_yaw_dr = yawError(entry.dr_pose, entry.gt_pose);
        const std::size_t ping_count = std::max<std::size_t>(1, entry.ping_count);
        for (std::size_t k = 0; k < ping_count; ++k) {
            const double ratio = (ping_count == 1)
                                     ? 1.0
                                     : static_cast<double>(k + 1) / static_cast<double>(ping_count);
            const double ping_index = entry.pseudo_time_idx + static_cast<double>(k);
            ofs << ping_index << ","
                << err_xy * ratio << ","
                << err_yaw * ratio << ","
                << err_xy_dr * ratio << ","
                << err_yaw_dr * ratio << ","
                << entry.submap_id << "\n";
        }
    }
}

} // namespace graph_optimization

