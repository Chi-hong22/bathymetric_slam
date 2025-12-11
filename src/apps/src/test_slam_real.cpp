/* Copyright 2019 Ignacio Torroba (torroba@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <cereal/archives/binary.hpp>

#include "data_tools/std_data.h"
#include "data_tools/benchmark.h"
// #include "/home/link/auvlib/auvlib/src/data_tools/include/data_tools/std_data.h"
// #include "/home/link/auvlib/auvlib/src/data_tools/include/data_tools/benchmark.h"

#include "submaps_tools/cxxopts.hpp"
#include "submaps_tools/submaps.hpp"

#include "registration/utils_visualization.hpp"
#include "registration/gicp_reg.hpp"

#include "graph_optimization/utils_g2o.hpp"
#include "graph_optimization/graph_construction.hpp"
#include "graph_optimization/ceres_optimizer.hpp"
#include "graph_optimization/read_g2o.h"

#include "bathy_slam/bathy_slam.hpp"

#include <pcl/filters/voxel_grid.h>

#define INTERACTIVE 0
#define VISUAL 0

using namespace Eigen;
using namespace std;
using namespace g2o;

bool next_step = false;
int current_step = 0;

//将子地图和轨迹转化为矩阵格式
void add_benchmark(SubmapsVec& submaps, benchmark::track_error_benchmark& benchmark, const string& name, bool is_groundtruth=false) {
    PointsT map = pclToMatrixSubmap(submaps);//map中存多个子地图，每一个子地图包含当前子地图的所有多波束测点，每一个子地图的点包括N行3列
    PointsT track = trackToMatrixSubmap(submaps);//将所有子地图的关键帧的位姿存在track中
    if (is_groundtruth) {
        benchmark.add_ground_truth(map, track);
    } else {
        benchmark.add_benchmark(map, track, name);
    }
}
//主要用于将地面真值 (ground truth, GT) 的子地图数据添加到基准测试对象中，并保存原始轨迹
void benchmark_gt(SubmapsVec& submaps_gt, benchmark::track_error_benchmark& benchmark) {
    // Benchmark GT
    add_benchmark(submaps_gt, benchmark, "0_original", true);
    ::ceres::optimizer::saveOriginalTrajectory(submaps_gt); // 将原始轨迹保存到txt文件
    std::cout << "可视化原始扫描数据，按空格键继续" << std::endl;
}

// 函数通过GICP子图配准和SLAM求解器，构建一个包含海洋深度图的SLAM图。
SubmapsVec build_bathymetric_graph(GraphConstructor& graph_obj, SubmapsVec& submaps_gt,
                                   GaussianGen& transSampler_DR, GaussianGen& rotSampler_DR,
                                   GaussianGen& transSampler_SM, GaussianGen& rotSampler_SM,
                                   YAML::Node config) {

    // GICP reg for submaps
    SubmapRegistration gicp_reg(config);

    // 创建SLAM求解器
    std::cout << "使用GICP子图配准构建水下地形图SLAM图" << std::endl;
    BathySlam slam_solver(graph_obj, gicp_reg);

    // 运行离线的Bathyslam算法
        // submaps_gt: 地面真值子图
        // transSampler_DR, rotSampler_DR: DR边噪声生成器
        // transSampler_SM, rotSampler_SM: 子地图噪声生成器
        // config: 配置参数
    SubmapsVec submaps_reg = slam_solver.runOffline(submaps_gt, 
                                                     transSampler_DR, rotSampler_DR,
                                                     transSampler_SM, rotSampler_SM, 
                                                     config);
    std::cout << "图构建完成，按空格键继续" << std::endl;

    return submaps_reg;
}

// 创建初始图形估计，如果add_gaussian_noise=true，则可选择添加高斯噪声
void create_initial_graph_estimate(GraphConstructor& graph_obj, SubmapsVec& submaps_reg, 
                                   GaussianGen& transSampler_DR, GaussianGen& rotSampler_DR, 
                                   bool add_gaussian_noise) {
    std::cout << "是否添加高斯噪声 = " << add_gaussian_noise << std::endl;
    if (add_gaussian_noise && !graph_obj.isDRNoiseApplied()) {
        // 向图中的DR边添加噪声（离线模式批量加噪，保证随机序列一致性）
        std::cout << "正在添加高斯噪声到所有DR边（离线批量模式）..." << std::endl;
        graph_obj.addNoiseToGraph(transSampler_DR, rotSampler_DR);
        std::cout << "已成功向图添加高斯噪声" << std::endl;
    }
    // 创建初始DR链并可视化
    graph_obj.createInitialEstimate(submaps_reg);
    std::cout << "初始图形估计构建完成，按空格键继续" << std::endl;
}
//图优化
void optimize_graph(GraphConstructor& graph_obj, SubmapsVec& submaps_reg, std::string outFilename, char* argv0, boost::filesystem::path output_path, bool use_huber_loss) {
    // 将图保存为g2o文件格式,以便可以用G2O工具进行优化
    graph_obj.saveG2OFile(outFilename);

    // 使用Ceres求解器优化图结构
    // poses存储优化后的位姿结果
    // graph_obj.drEdges_.size()表示Dead Reckoning边的数量
    // 离线模式显式保持旧版行为：300 次迭代、导出 debug 文件，保留可控的 Huber 开关
    ::ceres::optimizer::MapOfPoses poses = ::ceres::optimizer::ceresSolver(
        outFilename, graph_obj.drEdges_.size(), 300, true, use_huber_loss);

    // 使用优化后的位姿更新子地图
    ::ceres::optimizer::updateSubmapsCeres(poses, submaps_reg);

    // 输出优化后的结果到cereal序列化文件
    std::cout << "Output cereal: " << boost::filesystem::basename(output_path) << std::endl;
    try {
        std::ofstream os(boost::filesystem::basename(output_path) + ".cereal", std::ofstream::binary);
        if (!os.is_open()) {
            std::cerr << "Error: 无法打开输出文件进行写入" << std::endl;
            return;
        }
        
        // 检查子地图数量和估计的内存使用量
        std::cout << "正在序列化 " << submaps_reg.size() << " 个子地图..." << std::endl;
        
        {
            // 使用cereal的二进制存档器序列化子地图数据
            cereal::BinaryOutputArchive oarchive(os);
            oarchive(submaps_reg);
        }
        os.close();
        std::cout << "序列化完成" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Cereal序列化错误: " << e.what() << std::endl;
        std::cerr << "可能的原因: 内存不足或数据过大" << std::endl;
    }
    std::cout << "已进行图优化，按空格键继续" << std::endl;
}
//实现了基准测试结果的打印和可视化
void print_benchmark_results(SubmapsVec& submaps_reg, benchmark::track_error_benchmark& benchmark) {
    benchmark.print_summary();

    std::string command_str = "python ../scripts/plot_results.py --initial_poses poses_original.txt --corrupted_poses poses_corrupted.txt --optimized_poses poses_optimized.txt";
    const char *command = command_str.c_str();
    system(command);
}
//用于处理键盘事件，特别是当用户按下空格键时更新全局变量
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing) {
    if (event.getKeySym() == "space" && event.keyDown()) {
        next_step = true;
        current_step++;
    }
}

int main(int argc, char** argv){
    int linkto=100;
    std::cout<<linkto<<std::endl;
    // Inputs这里使用了 cxxopts 库解析命令行参数，包括帮助信息、是否使用模拟数据、输入路径和配置文件路径。
    std::string folder_str, path_str, output_str, simulation, config_path;
    cxxopts::Options options("MyProgram", "One line description of MyProgram");
    options.add_options()
        ("help", "Print help")
        ("simulation", "Simulation data from Gazebo", cxxopts::value(simulation))
        ("bathy_survey", "Input MBES pings in cereal file if simulation = no. If in simulation"
                          "input path to map_small folder", cxxopts::value(path_str))
        ("config", "YAML config file", cxxopts::value(config_path));

    auto result = options.parse(argc, argv);
    if (result.count("help")) {
        cout << options.help({ "", "Group" }) << endl;
        exit(0);
    }//配置文件加载和初始设置，设置输出文件路径，加载YAML配置文件，并读取其中定义的噪声参数。
    if(output_str.empty()){
        output_str = "output_cereal.cereal";
    }
    boost::filesystem::path output_path(output_str);
    string outFilename = "graph_corrupted.g2o";   // G2O output file

    YAML::Node config = YAML::LoadFile(config_path);
    std::cout << "已加载 Config file: " << config_path << std::endl;
    DRNoise dr_noise = loadDRNoiseFromFile(config);

    // 将在线参数写入 config，以便后续模块访问
    boost::filesystem::path config_abs = boost::filesystem::absolute(config_path);
    boost::filesystem::path config_dir = config_abs.parent_path();

    auto resolvePath = [&](const std::string& path_str) {
        boost::filesystem::path p(path_str);
        boost::filesystem::path abs_p = boost::filesystem::absolute(p, config_dir);
        return abs_p.lexically_normal().string();
    };

    const bool online_opt_enable = config["online_opt_enable"] ? config["online_opt_enable"].as<bool>() : false;
    const bool online_benchmark_enable = config["online_benchmark_enable"] ? config["online_benchmark_enable"].as<bool>() : true;
    const int online_opt_freq = config["online_opt_freq"] ? config["online_opt_freq"].as<int>() : 1;
    const int online_opt_max_iter = config["online_opt_max_iter"] ? config["online_opt_max_iter"].as<int>() : 50;
    std::string online_log_path = config["online_log_path"] ? config["online_log_path"].as<std::string>() : "build";
    std::string online_plot_input = config["online_plot_input"] ? config["online_plot_input"].as<std::string>() : "build/ping_error.csv";
    const bool use_huber_loss = config["enable_huber_loss"] ? config["enable_huber_loss"].as<bool>() : false;

    online_log_path = resolvePath(online_log_path);
    online_plot_input = resolvePath(online_plot_input);

    config["online_opt_enable"] = online_opt_enable;
    config["online_benchmark_enable"] = online_benchmark_enable;
    config["online_opt_freq"] = online_opt_freq;
    config["online_opt_max_iter"] = online_opt_max_iter;
    config["online_log_path"] = online_log_path;
    config["online_plot_input"] = online_plot_input;
    config["enable_huber_loss"] = use_huber_loss;

    // 设置高斯噪声的随机种子（分离DR与子地图种子以保证在线/离线一致性）
    int seed_dr = -1, seed_submap = -1;
    if (config["noise_seed_dr"]) {
        seed_dr = config["noise_seed_dr"].as<int>();
        std::cout << "DR边噪声种子已设置为: " << seed_dr << " (用户指定)" << std::endl;
    } else {
        std::cout << "未指定DR边噪声种子，将使用随机种子" << std::endl;
    }
    if (config["noise_seed_submap"]) {
        seed_submap = config["noise_seed_submap"].as<int>();
        std::cout << "子地图噪声种子已设置为: " << seed_submap << " (用户指定)" << std::endl;
    } else {
        std::cout << "未指定子地图噪声种子，将使用随机种子" << std::endl;
    }

    // Parse submaps from cereal file
    //解析输入数据并生成子地图
    //解析输入数据，根据是否使用模拟数据选择不同的解析方法。
    //如果使用真实数据，读取声呐ping数据并生成子地图，同时对点云数据进行体素滤波处理。
    boost::filesystem::path submaps_path(path_str);
    std::cout << "Input data " << submaps_path << std::endl;

    SubmapsVec submaps_gt, submaps_reg;
    if(simulation == "yes"){
        submaps_gt = readSubmapsInDir(submaps_path.string(), dr_noise);
    }
    else{
        std_data::mbes_ping::PingsT std_pings = std_data::read_data<std_data::mbes_ping::PingsT>(submaps_path);
        std::cout << "Number of pings in survey " << std_pings.size() << std::endl;
        
        {
            SubmapsVec traj_pings = parsePingsAUVlib(std_pings, dr_noise);
            int submap_size = config["submap_size"].as<int>();
            submaps_gt = createSubmaps(traj_pings, submap_size, dr_noise);

            // Filtering of submaps
            PointCloudT::Ptr cloud_ptr (new PointCloudT);
            pcl::VoxelGrid<PointT> voxel_grid_filter;
            voxel_grid_filter.setInputCloud (cloud_ptr);
            voxel_grid_filter.setLeafSize(config["downsampling_leaf_x"].as<double>(),
                                          config["downsampling_leaf_y"].as<double>(),
                                          config["downsampling_leaf_z"].as<double>());
            for(SubmapObj& submap_i: submaps_gt){
                *cloud_ptr = submap_i.submap_pcl_;
                voxel_grid_filter.setInputCloud(cloud_ptr);
                voxel_grid_filter.filter(*cloud_ptr);
                submap_i.submap_pcl_ = *cloud_ptr;
            }
        }
    }//构建图优化对象，读取协方差矩阵并初始化图优化构造对象 graph_obj。
    std::cout << "Number of submaps " << submaps_gt.size() << std::endl;

    // Graph constructor
    // Read training covs from folder
    covs covs_lc;
    boost::filesystem::path folder(folder_str);
    if(boost::filesystem::is_directory(folder)) {
        covs_lc = readCovsFromFiles(folder);
    }
    GraphConstructor graph_obj(covs_lc);//

    // Noise generators - 分离DR与子地图生成器以保证在线/离线一致性
    // DR生成器：用于图优化约束的DR边噪声
    GaussianGen transSampler_DR, rotSampler_DR;
    if (seed_dr >= 0) {
        setNoiseRandomSeed(seed_dr);
    }
    Matrix<double, 6,6> information_DR = generateGaussianNoise(transSampler_DR, rotSampler_DR);
    int actualSeed_DR = getCurrentNoiseSeed();
    
    // 子地图生成器：用于GICP配准前的子地图噪声
    GaussianGen transSampler_SM, rotSampler_SM;
    if (seed_submap >= 0) {
        setNoiseRandomSeed(seed_submap);
    }
    Matrix<double, 6,6> information_SM = generateGaussianNoise(transSampler_SM, rotSampler_SM);
    int actualSeed_SM = getCurrentNoiseSeed();
    
    std::cout << "=== 噪声系统已初始化（双种子模式）===" << std::endl;
    std::cout << "DR边实际种子: " << actualSeed_DR << std::endl;
    std::cout << "子地图实际种子: " << actualSeed_SM << std::endl;

    // flag for adding gaussian noise to submaps and graph
    bool add_gaussian_noise = config["add_gaussian_noise"].as<bool>();
    
    benchmark::track_error_benchmark benchmark("real_data", config["benchmark_nbr_rows"].as<int>(), config["benchmark_nbr_cols"].as<int>());
    std::cout << "Benchmark nbr rows and cols: " << benchmark.benchmark_nbr_rows << ", " << benchmark.benchmark_nbr_cols << std::endl;

#if VISUAL != 1
    if (online_opt_enable) {
        std::cout << "[ONLINE MODE] online_opt_enable=true, freq=" << online_opt_freq
                  << ", max_iter=" << online_opt_max_iter << std::endl;
        if (online_benchmark_enable) {
            benchmark_gt(submaps_gt, benchmark);
            std::cout << "---benchmark_gt (online mode)---" << std::endl;
        }

        submaps_reg = build_bathymetric_graph(graph_obj, submaps_gt, 
                                              transSampler_DR, rotSampler_DR,
                                              transSampler_SM, rotSampler_SM, config);
        std::cout << "---build_bathymetric_graphe (online mode)---" <<  std::endl;

        if (online_benchmark_enable) {
            add_benchmark(submaps_gt, benchmark, "1_After_GICP_GT");
            add_benchmark(submaps_reg, benchmark, "2_After_GICP_reg_online");
        }

        std::cout << "[ONLINE MODE] 完成在线增量流程，日志目录: " << online_log_path
                  << "，误差CSV: " << online_plot_input << std::endl;
    } else {
        // 离线批处理流程（保持原有行为）
        // 使用ground truth数据对benchmark进行评估
        benchmark_gt(submaps_gt, benchmark);
        std::cout << "---benchmark_gt---" <<  std::endl;

        // 进行离线SLAM
        // 注意：此处add_benchmark代码在/home/u/code_workplace/cpp/external/auvlib/src/data_tools/src/benchmark.cpp中
        submaps_reg = build_bathymetric_graph(graph_obj, submaps_gt, 
                                              transSampler_DR, rotSampler_DR,
                                              transSampler_SM, rotSampler_SM, config);
        std::cout << "---build_bathymetric_graphe---" <<  std::endl;
        add_benchmark(submaps_gt, benchmark, "1_After_GICP_GT");
        std::cout << "-1_After_GICP_GT-" <<  std::endl;
        add_benchmark(submaps_reg, benchmark, "2_After_GICP_reg");
        std::cout << "-2_After_GICP_reg-" <<  std::endl;
        add_benchmark(submaps_reg, benchmark, "3_Before_init_graph_estimates_reg");
        std::cout << "-3_Before_init_graph_estimates_reg-" <<  std::endl;

        // 创建初始图估计
        create_initial_graph_estimate(graph_obj, submaps_reg, 
                                      transSampler_DR, rotSampler_DR, add_gaussian_noise);
        std::cout << "---create_initial_graph_estimate---" <<  std::endl;
        // 动态重算 range：本阶段因注入误差/初始估计后位姿变换，XY 可能越过以 GT±K 固定的画布，
        // 这里基于当前阶段点云包围盒刷新 track 映射参数，防止越界（注意：仅本阶段像素坐标系与其他阶段不同）。
        {
            PointsT map_dyn = pclToMatrixSubmap(submaps_reg);
            benchmark.track_img_params(map_dyn, /*compute_range_from_points=*/true);
        }
        add_benchmark(submaps_reg, benchmark, "4_After_init_graph_estimates_reg");
        std::cout << "-4_After_init_graph_estimates_reg-" <<  std::endl;
        // 动态重算 range：优化前同样可能出现越界，重复基于点云更新映射参数，保证出图完整
        {
            PointsT map_dyn = pclToMatrixSubmap(submaps_reg);
            benchmark.track_img_params(map_dyn, /*compute_range_from_points=*/true);
        }
        add_benchmark(submaps_reg, benchmark, "5_before_optimize_graph");
        std::cout << "-5_before_optimize_graph-" <<  std::endl;

        // 优化图
        optimize_graph(graph_obj, submaps_reg, outFilename, argv[0], output_path, use_huber_loss);
        std::cout << "---optimize_graph---" <<  std::endl;
        add_benchmark(submaps_reg, benchmark, "6_optimized");
        std::cout << "-6_optimizedg-" <<  std::endl;
    }
#endif

    // Visualization
#if VISUAL == 1
    PCLVisualizer viewer ("Submaps viewer");
    viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*) NULL);
    viewer.loadCameraParameters("Antarctica7");
    SubmapsVisualizer* visualizer = new SubmapsVisualizer(viewer);
    visualizer->setVisualizer(submaps_gt, 1);

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
        if (next_step) {
            next_step = false;
            switch (current_step)
            {
            case 1:
                // Benchmark GT
                benchmark_gt(submaps_gt, benchmark);
                submaps_reg = build_bathymetric_graph(graph_obj, submaps_gt, 
                                                      transSampler_DR, rotSampler_DR,
                                                      transSampler_SM, rotSampler_SM, config);
                visualizer->updateVisualizer(submaps_reg);
                // Benchmark GT after GICP, the GT submaps have now been moved due to GICP registration
                add_benchmark(submaps_gt, benchmark, "-1_After_GICP_GT-");
                add_benchmark(submaps_reg, benchmark, "-2_After_GICP_reg-");
                break;
            case 2:
                add_benchmark(submaps_reg, benchmark, "-3_Before_init_graph_estimates_reg-");
                create_initial_graph_estimate(graph_obj, submaps_reg, 
                                              transSampler_DR, rotSampler_DR, add_gaussian_noise);
                visualizer->plotPoseGraphG2O(graph_obj, submaps_reg);
                // Benchmark corrupted (or not corrupted if add_gaussian_noise = false)
                add_benchmark(submaps_reg, benchmark, "-4_After_init_graph_estimates_reg-");
                break;
            case 3:
                add_benchmark(submaps_reg, benchmark, "-5_before_optimize_graph-");
                optimize_graph(graph_obj, submaps_reg, outFilename, argv[0], output_path, use_huber_loss);
                // Visualize Ceres output
                visualizer->plotPoseGraphCeres(submaps_reg);
                // Benchmark Optimized
                add_benchmark(submaps_reg, benchmark, "-6_optimized-");
                break;
            default:
                break;
            }
        }
    }
    delete(visualizer);
    print_benchmark_results(submaps_reg, benchmark);
#endif
    std::cout << "程序运行成功 " <<  std::endl;
    return 0;
}
