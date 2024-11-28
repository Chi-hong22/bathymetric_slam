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


/**
 * 添加基准测试
 * 
 * 该函数用于将子地图数据添加到基准测试中，以评估跟踪算法的性能
 * 它可以处理普通子地图数据和地面真实数据
 * 
 * @param submaps 子地图数据的向量，类型为SubmapsVec
 * @param benchmark 基准测试对象，用于跟踪误差评估
 * @param name 基准测试的名称，用于标识不同的测试数据
 * @param is_groundtruth 标志位，指示数据是否为地面真实数据，默认为false
 */
void add_benchmark(SubmapsVec& submaps, benchmark::track_error_benchmark& benchmark, const string& name, bool is_groundtruth=false) {
    // 将子地图数据转换为矩阵形式的点云数据
    PointsT map = pclToMatrixSubmap(submaps);
    // 将跟踪数据转换为矩阵形式的点云数据
    PointsT track = trackToMatrixSubmap(submaps);

    // 输出调试信息
    std::cout << "map size: " << map.size() << ", track size: " << track.size() << std::endl;

    // 如果数据是地面真实数据，则打印数据维度并添加到基准测试中
    if (is_groundtruth) {
        std::cout << "map[0] rows and cols: " << map[0].rows() << ", " << map[0].cols()  <<std::endl;
        benchmark.add_ground_truth(map, track);
    } else {
        // 如果数据不是地面真实数据，则将其作为基准测试数据添加到基准测试中
        benchmark.add_benchmark(map, track, name);
    }
}

void benchmark_gt(SubmapsVec& submaps_gt, benchmark::track_error_benchmark& benchmark) {
    // Benchmark GT
    add_benchmark(submaps_gt, benchmark, "0_original", true);
    ceres_::optimizer::saveOriginalTrajectory(submaps_gt); // Save original trajectory to txt
    std::cout << "Visualizing original survey, press space to continue" << std::endl;
}

SubmapsVec build_bathymetric_graph(GraphConstructor& graph_obj, SubmapsVec& submaps_gt,
GaussianGen& transSampler, GaussianGen& rotSampler, YAML::Node config) {

    // GICP reg for submaps
    SubmapRegistration gicp_reg(config);

    // Create SLAM solver and run offline
    std::cout << "Building bathymetric graph with GICP submap registration" << std::endl;
    BathySlam slam_solver(graph_obj, gicp_reg);
    SubmapsVec submaps_reg = slam_solver.runOffline(submaps_gt, transSampler, rotSampler, config);
    std::cout << "Done building graph, press space to continue" << std::endl;
    return submaps_reg;
}

// Create initial graph estimates, optionally add gaussian noise if add_gaussian_noise = true
void create_initial_graph_estimate(GraphConstructor& graph_obj, SubmapsVec& submaps_reg, GaussianGen& transSampler, GaussianGen& rotSampler, bool add_gaussian_noise) {
    std::cout << "Add gaussian noise = " << add_gaussian_noise << std::endl;
    if (add_gaussian_noise) {
        // Add noise to edges on the graph
        graph_obj.addNoiseToGraph(transSampler, rotSampler);
        std::cout << "Gaussian noise added to graph" << std::endl;
    }
    // Create initial DR chain and visualize
    graph_obj.createInitialEstimate(submaps_reg);
    std::cout << "Initial graph estimate constructed, press space to continue" << std::endl;

}

void optimize_graph(GraphConstructor& graph_obj, SubmapsVec& submaps_reg, std::string outFilename, char* argv0, boost::filesystem::path output_path) {
    // Save graph to output g2o file (optimization can be run with G2O)
    graph_obj.saveG2OFile(outFilename);

    // Optimize graph and save to cereal
    google::InitGoogleLogging(argv0);
    ceres_::optimizer::MapOfPoses poses = ceres_::optimizer::ceresSolver(outFilename, graph_obj.drEdges_.size());
    ceres_::optimizer::updateSubmapsCeres(poses, submaps_reg);
    std::cout << "Output cereal: " << boost::filesystem::basename(output_path) << std::endl;
    std::ofstream os(boost::filesystem::basename(output_path) + ".cereal", std::ofstream::binary);
    {
        cereal::BinaryOutputArchive oarchive(os);
        oarchive(submaps_reg);
        os.close();
    }
    std::cout << "Graph optimized, press space to continue" << std::endl;
}

void print_benchmark_results(SubmapsVec& submaps_reg, benchmark::track_error_benchmark& benchmark) {
    benchmark.print_summary();

    std::string command_str = "python ../scripts/plot_results.py --initial_poses poses_original.txt --corrupted_poses poses_corrupted.txt --optimized_poses poses_optimized.txt";
    const char *command = command_str.c_str();
    system(command);
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing) {
    if (event.getKeySym() == "space" && event.keyDown()) {
        next_step = true;
        current_step++;
    }
}

int main(int argc, char** argv){
    // 1. 配置与数据加载阶段
    // 1.1 解析命令行参数

    // 定义字符串变量，用于存储命令行参数
    std::string folder_str, path_str, output_str, simulation, config_path;
    // 创建cxxopts::Options对象，用于解析命令行参数
    // 参数包括程序名称和简短描述
    cxxopts::Options options("MyProgram", "One line description of MyProgram");

    // 添加命令行参数选项
    options.add_options()
        // 添加help选项，用于打印帮助信息
        ("help", "Print help")
        // 添加simulation选项，用于指定是否使用Gazebo模拟数据
        ("simulation", "Simulation data from Gazebo", cxxopts::value(simulation))
        // 添加bathy_survey选项，根据是否使用模拟数据，指定不同的输入路径
        // 如果不使用模拟数据，则输入MBES pings的cereal文件路径
        // 如果在模拟环境中，则输入map_small文件夹的路径
        ("bathy_survey", "Input MBES pings in cereal file if simulation = no. If in simulation"
                        "input path to map_small folder", cxxopts::value(path_str))
        // 添加config选项，用于指定YAML配置文件的路径
        ("config", "YAML config file", cxxopts::value(config_path));
    // 解析命令行参数
    // argc 是命令行参数的个数
    // argv 是一个包含命令行参数的字符数组
    // 返回值 result 表示解析的结果
    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        cout << options.help({ "", "Group" }) << endl;
        exit(0);
    }
    if(output_str.empty()){
        output_str = "output_cereal.cereal";
    }

    boost::filesystem::path output_path(output_str);
    string outFilename = "graph_corrupted.g2o";   // G2O output file

    // 1.2 加载配置文件
    YAML::Node config = YAML::LoadFile(config_path);
    std::cout << "Config file: " << config_path << std::endl;
    DRNoise dr_noise = loadDRNoiseFromFile(config);

    // 解析来自cereal文件的子地图
    // 使用给定的路径字符串创建一个boost::filesystem::path对象，用于处理子地图文件
    boost::filesystem::path submaps_path(path_str);
    // 输出输入数据的路径，用于调试和确认
    std::cout << "Input data " << submaps_path << std::endl;

    // 1.3 读取子图数据
    // 定义两个SubmapsVec类型的变量submaps_gt和submaps_reg
    // submaps_gt用于存储地面真值的子地图集合
    // submaps_reg用于存储注册或配准后的子地图集合
    SubmapsVec submaps_gt, submaps_reg;
    
    // 根据是否为仿真数据选择不同的加载方式
    if(simulation == "yes"){
        // 加载仿真数据
        auto path_temp = submaps_path.string();
        submaps_gt = readSubmapsInDir(path_temp, dr_noise);
    }
    else{
        // 加载实际声呐数据并进行预处理
        // - 解析ping数据
        // - 创建子图
        // - 点云降采样
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
            voxel_grid_filter.setLeafSize ( config["downsampling_leaf_x"].as<double>(),
                                            config["downsampling_leaf_y"].as<double>(),
                                            config["downsampling_leaf_z"].as<double>());
            for(SubmapObj& submap_i: submaps_gt){
                *cloud_ptr = submap_i.submap_pcl_;
                voxel_grid_filter.setInputCloud(cloud_ptr);
                voxel_grid_filter.filter(*cloud_ptr);
                submap_i.submap_pcl_ = *cloud_ptr;
            }
        }
    }
    std::cout << "Number of submaps " << submaps_gt.size() << std::endl;

    // Graph constructor
    // Read training covs from folder
    // 声明一个变量用于存储覆盖率信息
    covs covs_lc;
    // 使用给定的字符串初始化一个Boost.Filesystem路径对象，用于后续的目录操作
    boost::filesystem::path folder(folder_str);
    // 检查路径是否对应一个目录
    if(boost::filesystem::is_directory(folder)) {
        // 如果是目录，从目录中的文件读取覆盖率信息
        covs_lc = readCovsFromFiles(folder);
    }

    // 2. 图构建与优化准备阶段
    // 2.1 初始化图构造器
    GraphConstructor graph_obj(covs_lc);

    // 2.2 初始化噪声生成器
    GaussianGen transSampler, rotSampler;
    Matrix<double, 6,6> information = generateGaussianNoise(transSampler, rotSampler);

    // flag for adding gaussian noise to submaps and graph
    bool add_gaussian_noise = config["add_gaussian_noise"].as<bool>();
    // 2.3 设置基准测试工具
    benchmark::track_error_benchmark benchmark("real_data", config["benchmark_nbr_rows"].as<int>(), config["benchmark_nbr_cols"].as<int>());
    std::cout << "Benchmark nbr rows and cols: " << benchmark.benchmark_nbr_rows << ", " << benchmark.benchmark_nbr_cols << std::endl;

    // 3. SLAM处理主循环（非可视化模式）
    #if VISUAL != 1
        // 3.1 对原始数据进行基准测试
        benchmark_gt(submaps_gt, benchmark);
        // 3.2 构建水下地形图并进行GICP配准
        submaps_reg = build_bathymetric_graph(graph_obj, submaps_gt, transSampler, rotSampler, config);
        add_benchmark(submaps_gt, benchmark, "1_After_GICP_GT");
        add_benchmark(submaps_reg, benchmark, "2_After_GICP_reg");
        add_benchmark(submaps_reg, benchmark, "3_Before_init_graph_estimates_reg");
        // 3.3 创建并优化位姿图
        create_initial_graph_estimate(graph_obj, submaps_reg, transSampler, rotSampler, add_gaussian_noise);
        add_benchmark(submaps_reg, benchmark, "4_After_init_graph_estimates_reg");
        add_benchmark(submaps_reg, benchmark, "5_before_optimize_graph");
        // 3.4 图优化与结果输出
        optimize_graph(graph_obj, submaps_reg, outFilename, argv[0], output_path);
        add_benchmark(submaps_reg, benchmark, "6_optimized");
    #endif

    // 4. 可视化处理（可视化模式）
    #if VISUAL == 1
        // 4.1 初始化可视化器
        PCLVisualizer viewer ("Submaps viewer");

        // 4.2 交互式处理循环
        // - 显示原始数据
        // - GICP配准结果展示
        // - 图优化结果展示
        // - 实时更新可视化
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
        // 4.3 输出最终结果
        print_benchmark_results(submaps_reg, benchmark);
    #endif

    return 0;
}
