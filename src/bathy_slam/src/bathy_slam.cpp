#include "bathy_slam/bathy_slam.hpp"
#include "graph_optimization/online_logger.hpp"
#include "graph_optimization/ceres_optimizer.hpp"

#include <boost/filesystem.hpp>
#include <glog/logging.h>
#include <algorithm>
#include <memory>
#include <vector>

BathySlam::BathySlam(GraphConstructor &graph_obj, SubmapRegistration &gicp_reg):
    graph_obj_(&graph_obj), gicp_reg_(&gicp_reg){

}

BathySlam::~BathySlam(){

}

// 运行离线的Bathyslam算法
// submaps_gt: 地面真值子图
// transSampler_DR, rotSampler_DR: DR边噪声的高斯采样器（用于图优化约束）
// transSampler_SM, rotSampler_SM: 子地图噪声的高斯采样器（用于GICP配准前）
// config: 配置参数
SubmapsVec BathySlam::runOffline(SubmapsVec& submaps_gt, 
                                 GaussianGen& transSampler_DR, GaussianGen& rotSampler_DR,
                                 GaussianGen& transSampler_SM, GaussianGen& rotSampler_SM,
                                 YAML::Node config){
    // 从文件中加载DR噪声参数
    DRNoise dr_noise = loadDRNoiseFromFile(config);
    // 初始化目标子图并应用DR噪声
    SubmapObj submap_trg(dr_noise);
    // 初始化前一个和已注册的子图容器
    SubmapsVec submaps_prev, submaps_reg;
    // 打开文件流以写入回环闭合信息
    ofstream fileOutputStream;
    fileOutputStream.open("loop_closures.txt", std::ofstream::out);

    // 解析在线 SLAM 相关参数
    const bool add_gaussian_noise = (config["add_gaussian_noise"]) ? config["add_gaussian_noise"].as<bool>() : false;
    const double submap_level_yaw_std = (config["submap_level_yaw_std"]) ? config["submap_level_yaw_std"].as<double>() : 0.05;  // 默认值与硬编码一致
    const double graph_level_yaw_std = (config["graph_level_yaw_std"]) ? config["graph_level_yaw_std"].as<double>() : 0.005;  // 默认值与硬编码一致
    const bool online_enabled = (config["online_opt_enable"]) ? config["online_opt_enable"].as<bool>() : false;
    const int online_opt_freq = (config["online_opt_freq"]) ? config["online_opt_freq"].as<int>() : 1;
    const int online_opt_max_iter = (config["online_opt_max_iter"]) ? config["online_opt_max_iter"].as<int>() : 50;
    const std::string online_log_path = (config["online_log_path"]) ? config["online_log_path"].as<std::string>() : "build";
    const std::string online_plot_input = (config["online_plot_input"]) ? config["online_plot_input"].as<std::string>() : "build/ping_error.csv";
    const int default_submap_size = (config["submap_size"]) ? config["submap_size"].as<int>() : 1;
    const bool use_huber_loss = (config["enable_huber_loss"])
                                    ? config["enable_huber_loss"].as<bool>()
                                    : false;
    const bool online_snapshot_enable = (config["online_opt_snapshot_enable"])
                                            ? config["online_opt_snapshot_enable"].as<bool>()
                                            : false;
    const int online_snapshot_freq = (config["online_opt_snapshot_freq"])
                                         ? config["online_opt_snapshot_freq"].as<int>()
                                         : 1;
    const std::string online_snapshot_dir = (config["online_opt_snapshot_dir"])
                                                ? config["online_opt_snapshot_dir"].as<std::string>()
                                                : "poses_optimized_online_log";

    std::unique_ptr<graph_optimization::OnlineLogWriter> online_logger;
    std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> gt_poses;
    std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> dr_poses;
    double pseudo_time_idx = 0.0;
    if (online_enabled) {
        int max_id = 0;
        for (const auto& submap : submaps_gt) {
            max_id = std::max(max_id, submap.submap_id_);
        }
        gt_poses.resize(max_id + 1, Eigen::Isometry3f::Identity());
        dr_poses.resize(max_id + 1, Eigen::Isometry3f::Identity());
        for (const auto& submap : submaps_gt) {
            if (submap.submap_id_ >= 0 && submap.submap_id_ < static_cast<int>(gt_poses.size())) {
                gt_poses[submap.submap_id_] = submap.submap_tf_;
                dr_poses[submap.submap_id_] = submap.submap_tf_;
            }
        }
        online_logger = std::make_unique<graph_optimization::OnlineLogWriter>(online_log_path, online_plot_input);
    }
    boost::filesystem::path log_dir(online_log_path);
    boost::filesystem::path snapshot_dir_path(online_snapshot_dir);
    if (online_enabled && !log_dir.empty()) {
        boost::filesystem::create_directories(log_dir);
        if (online_snapshot_enable) {
            if (snapshot_dir_path.is_relative()) {
                snapshot_dir_path = log_dir / snapshot_dir_path;
            }
            boost::filesystem::create_directories(snapshot_dir_path);
        }
    }

    ::ceres::optimizer::MapOfPoses last_online_poses;

    auto tryRunOnlineOptimization = [&](SubmapsVec& registered_submaps) {
        if (!online_enabled || !online_logger) {
            return;
        }
        if (online_opt_freq <= 0) {
            return;
        }
        if (graph_obj_->drEdges_.empty() && graph_obj_->lcEdges_.empty()) {
            return;
        }
        const int registered_count = static_cast<int>(registered_submaps.size());
        if (registered_count == 0 || (registered_count % online_opt_freq) != 0) {
            return;
        }
        static bool glog_initialized = false;
        if (!glog_initialized) {
            google::InitGoogleLogging("online_opt");
            glog_initialized = true;
        }
        boost::filesystem::path graph_path = log_dir / "graph_online_tmp.g2o";
        graph_obj_->saveG2OFile(graph_path.string());
        ::ceres::optimizer::MapOfPoses poses = ::ceres::optimizer::ceresSolver(
            graph_path.string(), graph_obj_->drEdges_.size(), online_opt_max_iter, false, use_huber_loss);
        ::ceres::optimizer::updateSubmapsCeres(poses, registered_submaps);
        last_online_poses = poses;

        if (online_snapshot_enable && online_snapshot_freq > 0) {
            if (registered_count > 0 && (registered_count % online_snapshot_freq) == 0) {
                boost::filesystem::path snapshot_path = snapshot_dir_path / ("poses_optimized_step_" + std::to_string(registered_count) + ".txt");
                CHECK(::ceres::optimizer::OutputPoses(snapshot_path.string(), poses))
                    << "Error outputting online snapshot to " << snapshot_path.string();
                std::cout << "[在线模式] 已保存优化快照: " << snapshot_path.string() << std::endl;
            }
        }
    };

    // 初始化回环闭合的信息阈值，较高的信息阈值意味着更严格的回环闭合筛选标准
    double info_thres = 0.1; // 信息阈值 原始数值：0.1  注意，实际代码中没有使用这个参数
    // 遍历每个地面真值子图
    for(SubmapObj& submap_i: submaps_gt){
        // 输出当前子图信息
        std::cout << " ----------- 子图" 
                    << submap_i.submap_id_ << ", 扫描条带"
                    << submap_i.swath_id_ << " ------------"
                    << std::endl;

        // 查找回环闭合
        for(SubmapObj& submap_k: submaps_reg){
            // 不查找同一扫描条带或前一个子图之间的重叠
            if(submap_k.submap_id_ != submap_i.submap_id_ - 1){
                submaps_prev.push_back(submap_k);
            }
        }
        // 子图是否在地图框架中？
        bool submaps_in_map_tf = true;
        // 查找子图之间的重叠
        submap_i.findOverlaps(submaps_in_map_tf, submaps_prev, config["overlap_coverage"].as<double>());
        // 清除前一个子图以供下次迭代使用
        submaps_prev.clear();

    #if INTERACTIVE == 1
        // 更新可视化
        submaps_reg.push_back(submap_i); // 将 submap_i 添加到已注册集合中（仅用于可视化）
        visualizer->updateVisualizer(submaps_reg);
        while(!viewer.wasStopped ()){
            viewer.spinOnce ();
        }
        viewer.resetStoppedFlag();
        submaps_reg.pop_back();
    #endif
        // 创建图顶点 i
        graph_obj_->createNewVertex(submap_i);
        if (online_enabled && !dr_poses.empty() &&
            submap_i.submap_id_ >= 0 &&
            submap_i.submap_id_ < static_cast<int>(dr_poses.size()) &&
            submap_i.submap_id_ == 0) {
            dr_poses[submap_i.submap_id_] = submap_i.submap_tf_;
        }

        // 创建 DR 边 i 并存储（跳过子图 0）
        if(submap_i.submap_id_ != 0 ){
            std::cout << "推位边 DR from " << submap_i.submap_id_ -1 << " to " << submap_i.submap_id_<< std::endl;
            graph_obj_->createDREdge(submap_i);
            // 在线模式：逐边加噪（用于实时误差累积）；离线模式：保持批量加噪（保证随机数消耗顺序一致）
            if (add_gaussian_noise && online_enabled) {
                graph_obj_->addNoiseToLastDREdge(transSampler_DR, rotSampler_DR, graph_level_yaw_std);
            }
            if (online_enabled && !dr_poses.empty() &&
                submap_i.submap_id_ >= 0 &&
                submap_i.submap_id_ < static_cast<int>(dr_poses.size())) {
                const Eigen::Isometry3f& prev_pose = dr_poses[submap_i.submap_id_ - 1];
                const Eigen::Isometry3d& meas = graph_obj_->drMeas_.back();
                Eigen::Isometry3d prev_d = prev_pose.cast<double>();
                dr_poses[submap_i.submap_id_] = (prev_d * meas).cast<float>();
            }
        }

        // 如果检测到潜在的回环闭合
        SubmapObj submap_final = submap_i;
        if(!submap_i.overlaps_idx_.empty()){
            // 将回环闭合保存到文本文件
            if(fileOutputStream.is_open()){
                fileOutputStream << submap_i.submap_id_;
                for(unsigned int j=0; j<submap_i.overlaps_idx_.size(); j++){
                    fileOutputStream << " " << submap_i.overlaps_idx_.at(j);
                }
                fileOutputStream << "\n";
            }

            // 构建目标子地图，合并与当前子图重叠的已注册子地图
            submap_trg = gicp_reg_->constructTrgSubmap(submaps_reg, submap_i.overlaps_idx_, dr_noise);
            if (add_gaussian_noise) {
                addNoiseToSubmap(transSampler_SM, rotSampler_SM, submap_i, submap_level_yaw_std); // 向子地图添加误差扰动（使用独立种子）
            }

            // Compute initial guess for GICP
            Eigen::Matrix4f tf_i = submap_i.submap_tf_.matrix();
            // We use the pose of the first overlapping submap as the reference for the target
            Eigen::Matrix4f tf_trg = submaps_reg.at(submap_i.overlaps_idx_.at(0)).submap_tf_.matrix();
            // Eigen::Matrix4f initial_guess = tf_trg.inverse() * tf_i;
            Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();

            // 输出初始猜测矩阵到终端
            std::cout << "GICP初始猜测矩阵 (子图 " << submap_i.submap_id_ << " -> 目标子图):" << std::endl;
            std::cout << initial_guess << std::endl;

            // 使用GICP算法对目标子地图和当前子图进行配准，如果配准成功，则更新最终子地图
            if(gicp_reg_->gicpSubmapRegistration(submap_trg, submap_i, initial_guess)){
                submap_final = submap_i;
            }
             // 清除目标子地图的点云数据，以便后续使用
            submap_trg.submap_pcl_.clear();

            // 创建回环闭合
            graph_obj_->edge_covs_type_ = config["lc_edge_covs_type"].as<int>();
            graph_obj_->findLoopClosures(submap_final, submaps_reg, info_thres);
        }
        submaps_reg.push_back(submap_final);    // 将注册后的 submap_i 添加到已注册子图集合中

        if (online_enabled && online_logger) {
            std::size_t ping_count = (submap_final.auv_tracks_.rows() > 0)
                                         ? static_cast<std::size_t>(submap_final.auv_tracks_.rows())
                                         : static_cast<std::size_t>(std::max(default_submap_size, 1));
            if (ping_count == 0) {
                ping_count = 1;
            }
            graph_optimization::OnlineLogEntry entry;
            entry.submap_id = submap_final.submap_id_;
            entry.pseudo_time_idx = pseudo_time_idx;
            entry.ping_count = ping_count;
            entry.est_pose = submap_final.submap_tf_;
            if (!gt_poses.empty() && submap_final.submap_id_ >= 0 &&
                submap_final.submap_id_ < static_cast<int>(gt_poses.size())) {
                entry.gt_pose = gt_poses[submap_final.submap_id_];
            }
            if (!dr_poses.empty() && submap_final.submap_id_ >= 0 &&
                submap_final.submap_id_ < static_cast<int>(dr_poses.size())) {
                entry.dr_pose = dr_poses[submap_final.submap_id_];
            }
            online_logger->addEntry(entry);
            pseudo_time_idx += static_cast<double>(ping_count);
        }
        tryRunOnlineOptimization(submaps_reg);

    #if INTERACTIVE == 1
        // 更新可视化
        visualizer->updateVisualizer(submaps_reg);
        while(!viewer.wasStopped ()){
            viewer.spinOnce ();
        }
        viewer.resetStoppedFlag();
    #endif
    }
    // 关闭文件流
    fileOutputStream.close();

    if (online_enabled && online_logger) {
        const std::string raw_log_file = (log_dir / "online_log.csv").string();
        online_logger->writeRawLog(raw_log_file);
        online_logger->writePingErrorCsv();
        
        // 在线模式结束时，输出poses_corrupted.txt和poses_optimized.txt
        std::cout << "[在线模式] 正在导出最终位姿文件..." << std::endl;
        
        // 1. 输出最终poses_optimized.txt（优先使用最后一次优化返回的poses缓存）
        ::ceres::optimizer::MapOfPoses optimized_poses_map;
        if (!last_online_poses.empty()) {
            optimized_poses_map = last_online_poses;
        } else {
            for (size_t i = 0; i < submaps_reg.size(); ++i) {
                ::ceres::optimizer::Pose3d pose;
                Eigen::Isometry3f tf = submaps_reg[i].submap_tf_;
                pose.p = tf.translation().cast<double>();
                // 从旋转矩阵转换为欧拉角 (roll, pitch, yaw)
                Eigen::Matrix3f rot = tf.rotation();
                Eigen::Vector3d euler;
                euler[0] = atan2(rot(2,1), rot(2,2));  // roll
                euler[1] = atan2(-rot(2,0), sqrt(rot(2,1)*rot(2,1) + rot(2,2)*rot(2,2)));  // pitch
                euler[2] = atan2(rot(1,0), rot(0,0));  // yaw
                pose.q = euler;
                optimized_poses_map[i] = pose;
            }
        }
        CHECK(::ceres::optimizer::OutputPoses("poses_optimized.txt", optimized_poses_map))
            << "Error outputting poses_optimized.txt in online mode";
        std::cout << "[在线模式] 已导出 poses_optimized.txt" << std::endl;
        
        // 2. 计算poses_corrupted.txt（从原始GT位姿开始累积已加噪的DR边测量）
        // 这样计算的结果和离线模式完全等效：离线也是从原始位姿通过createInitialEstimate累积DR链
        ::ceres::optimizer::MapOfPoses corrupted_poses_map;
        
        // 调试输出
        std::cout << "\n========== 在线模式 poses_corrupted 生成前检查 ==========" << std::endl;
        std::cout << "[在线] drMeas_ 数量: " << graph_obj_->drMeas_.size() << std::endl;
        std::cout << "[在线] submaps_gt[0] 位姿:" << std::endl;
        std::cout << submaps_gt[0].submap_tf_.matrix() << std::endl;
        if (!graph_obj_->drMeas_.empty()) {
            std::cout << "[在线] drMeas_[0-2] translation 和 yaw:" << std::endl;
            for (size_t i = 0; i < std::min((size_t)3, graph_obj_->drMeas_.size()); ++i) {
                Eigen::Matrix3d rot = graph_obj_->drMeas_[i].rotation();
                double yaw = atan2(rot(1,0), rot(0,0));
                std::cout << "  DR[" << i << "]: translation=" 
                          << graph_obj_->drMeas_[i].translation().transpose() 
                          << ", yaw=" << yaw << " rad" << std::endl;
            }
        }
        std::cout << "========================================================\n" << std::endl;
        
        // 第一个子图使用原始GT位姿
        {
            ::ceres::optimizer::Pose3d pose;
            Eigen::Isometry3f tf = submaps_gt[0].submap_tf_;
            pose.p = tf.translation().cast<double>();
            Eigen::Matrix3f rot = tf.rotation();
            Eigen::Vector3d euler;
            euler[0] = atan2(rot(2,1), rot(2,2));  // roll
            euler[1] = atan2(-rot(2,0), sqrt(rot(2,1)*rot(2,1) + rot(2,2)*rot(2,2)));  // pitch
            euler[2] = atan2(rot(1,0), rot(0,0));  // yaw
            pose.q = euler;
            corrupted_poses_map[0] = pose;
        }
        
        // 后续子图通过累积已加噪的DR边测量计算（等效于离线的createInitialEstimate）
        Eigen::Isometry3d current_pose = submaps_gt[0].submap_tf_.cast<double>();
        std::cout << "[在线累积] 起始位姿 translation: " 
                  << current_pose.translation().transpose() << std::endl;
        for (size_t i = 0; i < graph_obj_->drMeas_.size(); ++i) {
            const Eigen::Isometry3d& meas = graph_obj_->drMeas_[i];
            current_pose = current_pose * meas;  // 累积DR测量
            if (i < 3) {  // 只输出前3次累积结果
                std::cout << "[在线累积] 累积DR[" << i << "]后位姿 translation: " 
                          << current_pose.translation().transpose() << std::endl;
            }
            
            ::ceres::optimizer::Pose3d pose;
            pose.p = current_pose.translation();
            Eigen::Matrix3d rot = current_pose.rotation();
            Eigen::Vector3d euler;
            euler[0] = atan2(rot(2,1), rot(2,2));  // roll
            euler[1] = atan2(-rot(2,0), sqrt(rot(2,1)*rot(2,1) + rot(2,2)*rot(2,2)));  // pitch
            euler[2] = atan2(rot(1,0), rot(0,0));  // yaw
            pose.q = euler;
            corrupted_poses_map[i + 1] = pose;
        }
        
        CHECK(::ceres::optimizer::OutputPoses("poses_corrupted.txt", corrupted_poses_map))
            << "Error outputting poses_corrupted.txt in online mode";
        std::cout << "[在线模式] 已导出 poses_corrupted.txt（从GT位姿累积DR链）" << std::endl;
        std::cout << "[在线模式] poses文件导出完成" << std::endl;
    }

    // 返回已注册的子图
    return submaps_reg;
}
