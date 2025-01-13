#include "bathy_slam/bathy_slam.hpp"


BathySlam::BathySlam(GraphConstructor &graph_obj, SubmapRegistration &gicp_reg):
    graph_obj_(&graph_obj), gicp_reg_(&gicp_reg){

}

BathySlam::~BathySlam(){

}

// 运行离线的Bathyslam算法
// submaps_gt: 地面真值子图
// transSampler: 平移噪声的高斯采样器
// rotSampler: 旋转噪声的高斯采样器
// config: 配置参数
SubmapsVec BathySlam::runOffline(SubmapsVec& submaps_gt, GaussianGen& transSampler, GaussianGen& rotSampler, YAML::Node config){
    // 从文件中加载死区噪声参数
    DRNoise dr_noise = loadDRNoiseFromFile(config);
    // 初始化目标子图并应用DR噪声
    SubmapObj submap_trg(dr_noise);
    // 初始化前一个和已注册的子图容器
    SubmapsVec submaps_prev, submaps_reg;
    // 打开文件流以写入回环闭合信息
    ofstream fileOutputStream;
    fileOutputStream.open("loop_closures.txt", std::ofstream::out);

    // 初始化回环闭合的信息阈值
    double info_thres = 0.1;
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

        // 创建 DR 边 i 并存储（跳过子图 0）
        if(submap_i.submap_id_ != 0 ){
            std::cout << "DR 边从 " << submap_i.submap_id_ -1 << " 到 " << submap_i.submap_id_<< std::endl;
            graph_obj_->createDREdge(submap_i);
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

            // 注册重叠的子图
            submap_trg = gicp_reg_->constructTrgSubmap(submaps_reg, submap_i.overlaps_idx_, dr_noise);
            if (config["add_gaussian_noise"].as<bool>()) {
                addNoiseToSubmap(transSampler, rotSampler, submap_i); // 向源子图添加扰动
            }

            if(gicp_reg_->gicpSubmapRegistration(submap_trg, submap_i)){
                submap_final = submap_i;
            }
            submap_trg.submap_pcl_.clear();

            // 创建回环闭合
            graph_obj_->edge_covs_type_ = config["lc_edge_covs_type"].as<int>();
            graph_obj_->findLoopClosures(submap_final, submaps_reg, info_thres);
        }
        submaps_reg.push_back(submap_final);    // 将注册后的 submap_i 添加到已注册子图集合中

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

    // 返回已注册的子图
    return submaps_reg;
}
