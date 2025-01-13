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
                                   GaussianGen& transSampler, GaussianGen& rotSampler, YAML::Node config) {

    // GICP reg for submaps
    SubmapRegistration gicp_reg(config);

    // 创建SLAM求解器
    std::cout << "使用GICP子图配准构建水下地形图SLAM图" << std::endl;
    BathySlam slam_solver(graph_obj, gicp_reg);

    // 运行离线的Bathyslam算法
        // submaps_gt: 地面真值子图
        // transSampler: 平移噪声的高斯采样器
        // rotSampler: 旋转噪声的高斯采样器
        // config: 配置参数
    SubmapsVec submaps_reg = slam_solver.runOffline(submaps_gt, transSampler, rotSampler, config);
    std::cout << "图构建完成，按空格键继续" << std::endl;

    return submaps_reg;
}

// 创建初始图形估计，如果add_gaussian_noise=true，则可选择添加高斯噪声
void create_initial_graph_estimate(GraphConstructor& graph_obj, SubmapsVec& submaps_reg, GaussianGen& transSampler, GaussianGen& rotSampler, bool add_gaussian_noise) {
    std::cout << "是否添加高斯噪声 = " << add_gaussian_noise << std::endl;
    if (add_gaussian_noise) {
        // 向图中的边添加噪声
        graph_obj.addNoiseToGraph(transSampler, rotSampler);
        std::cout << "已向图添加高斯噪声" << std::endl;
    }
    // 创建初始DR链并可视化
    graph_obj.createInitialEstimate(submaps_reg);
    std::cout << "初始图形估计构建完成，按空格键继续" << std::endl;
}
//图优化
void optimize_graph(GraphConstructor& graph_obj, SubmapsVec& submaps_reg, std::string outFilename, char* argv0, boost::filesystem::path output_path) {
    // Save graph to output g2o file (optimization can be run with G2O)
    graph_obj.saveG2OFile(outFilename);

    // Optimize graph and save to cereal
    google::InitGoogleLogging(argv0);
    ::ceres::optimizer::MapOfPoses poses = ::ceres::optimizer::ceresSolver(outFilename, graph_obj.drEdges_.size());
    ::ceres::optimizer::updateSubmapsCeres(poses, submaps_reg);
    std::cout << "Output cereal: " << boost::filesystem::basename(output_path) << std::endl;
    std::ofstream os(boost::filesystem::basename(output_path) + ".cereal", std::ofstream::binary);
    {
        cereal::BinaryOutputArchive oarchive(os);
        oarchive(submaps_reg);
        os.close();
    }
    std::cout << "Graph optimized, press space to continue" << std::endl;
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
    std::cout << "Config file: " << config_path << std::endl;
    DRNoise dr_noise = loadDRNoiseFromFile(config);

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

    // Noise generators
    //初始化噪声生成器和基准测试对象，用于后续的误差评估。
    GaussianGen transSampler, rotSampler;
    Matrix<double, 6,6> information = generateGaussianNoise(transSampler, rotSampler);

    // flag for adding gaussian noise to submaps and graph
    bool add_gaussian_noise = config["add_gaussian_noise"].as<bool>();
    
    benchmark::track_error_benchmark benchmark("real_data", config["benchmark_nbr_rows"].as<int>(), config["benchmark_nbr_cols"].as<int>());
    std::cout << "Benchmark nbr rows and cols: " << benchmark.benchmark_nbr_rows << ", " << benchmark.benchmark_nbr_cols << std::endl;

#if VISUAL != 1
    //如果 VISUAL 宏定义不等于1，按照以下顺序执行：
        // 对地面真值（GT）数据进行基准测试。
        // 使用GICP方法构建测深图。
        // 添加基准测试，标记不同阶段的数据状态。
        // 创建初始图优化估计。
        // 优化图，并将优化后的结果记录到基准测试中。

    // 使用ground truth数据对benchmark进行评估
    benchmark_gt(submaps_gt, benchmark);
    std::cout << "---benchmark_gt---" <<  std::endl;

    // 进行离线SLAM
    submaps_reg = build_bathymetric_graph(graph_obj, submaps_gt, transSampler, rotSampler, config);
    std::cout << "---build_bathymetric_graphe---" <<  std::endl;
    add_benchmark(submaps_gt, benchmark, "1_After_GICP_GT");
    std::cout << "-1_After_GICP_GT-" <<  std::endl;
    add_benchmark(submaps_reg, benchmark, "2_After_GICP_reg");
    std::cout << "-2_After_GICP_reg-" <<  std::endl;
    add_benchmark(submaps_reg, benchmark, "3_Before_init_graph_estimates_reg");
    std::cout << "-3_Before_init_graph_estimates_reg-" <<  std::endl;

    // 创建初始图估计
    create_initial_graph_estimate(graph_obj, submaps_reg, transSampler, rotSampler, add_gaussian_noise);
    std::cout << "---create_initial_graph_estimate---" <<  std::endl;
    add_benchmark(submaps_reg, benchmark, "4_After_init_graph_estimates_reg");
    std::cout << "-4_After_init_graph_estimates_reg-" <<  std::endl;
    add_benchmark(submaps_reg, benchmark, "5_before_optimize_graph");
    std::cout << "-5_before_optimize_graph-" <<  std::endl;

    // 优化图
    optimize_graph(graph_obj, submaps_reg, outFilename, argv[0], output_path);
    std::cout << "---optimize_graph---" <<  std::endl;
    add_benchmark(submaps_reg, benchmark, "6_optimized");
    std::cout << "-6_optimizedg-" <<  std::endl;

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
                submaps_reg = build_bathymetric_graph(graph_obj, submaps_gt, transSampler, rotSampler, config);
                visualizer->updateVisualizer(submaps_reg);
                // Benchmark GT after GICP, the GT submaps have now been moved due to GICP registration
                add_benchmark(submaps_gt, benchmark, "1_After_GICP_GT");
                add_benchmark(submaps_reg, benchmark, "2_After_GICP_reg");
                break;
            case 2:
                add_benchmark(submaps_reg, benchmark, "3_Before_init_graph_estimates_reg");
                create_initial_graph_estimate(graph_obj, submaps_reg, transSampler, rotSampler, add_gaussian_noise);
                visualizer->plotPoseGraphG2O(graph_obj, submaps_reg);
                // Benchmark corrupted (or not corrupted if add_gaussian_noise = false)
                add_benchmark(submaps_reg, benchmark, "4_After_init_graph_estimates_reg");
                break;
            case 3:
                add_benchmark(submaps_reg, benchmark, "5_before_optimize_graph");
                optimize_graph(graph_obj, submaps_reg, outFilename, argv[0], output_path);
                // Visualize Ceres output
                visualizer->plotPoseGraphCeres(submaps_reg);
                // Benchmark Optimized
                add_benchmark(submaps_reg, benchmark, "6_optimized");
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
