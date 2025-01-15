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

// #include "/home/link/bathymetric_slam/src/registration/include/registration/gicp_reg.hpp"
#include "registration/gicp_reg.hpp"

using namespace std;
using namespace Eigen;
using PointsT = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;



SubmapRegistration::SubmapRegistration(YAML::Node config){

    ret_tf_ = Eigen::Matrix4f::Identity();
    benchmark_ = benchmark::track_error_benchmark();

    // Constrain GICP to x,y, yaw
    pcl::registration::WarpPointRigid3D<PointT, PointT>::Ptr warp_fcn
      (new pcl::registration::WarpPointRigid3D<PointT, PointT>);

    pcl::registration::TransformationEstimationLM<PointT, PointT>::Ptr te
            (new pcl::registration::TransformationEstimationLM<PointT, PointT>);
    te->setWarpFunction (warp_fcn);
    gicp_.setTransformationEstimation(te);

    // GICP parameters
    loadConfig(config);
}

SubmapRegistration::~SubmapRegistration(){

}

/**
 * @brief 加载并配置GICP算法的参数。
 * 
 * 该函数从YAML配置文件中读取参数，并将其应用到GICP（Generalized Iterative Closest Point）算法中。
 * 配置项包括最大对应距离、最大迭代次数、变换增量阈值、欧几里得适配度阈值、RANSAC离群点剔除阈值、法向量计算方法等。
 * 这些参数用于优化点云配准的精度和鲁棒性。
 * 
 * @param config YAML配置节点，包含GICP算法所需的各项参数。
 */
void SubmapRegistration::loadConfig(YAML::Node config){
    // 设置最大对应距离，此参数决定了在两点之间的匹配点的最大距离，超过该距离的点对将被忽略，用于减少不相关的噪声点的影响。
    gicp_.setMaxCorrespondenceDistance(config["gicp_max_correspondence_distance"].as<double>());

    // 设置GICP算法的最大迭代次数，用于控制迭代收敛的时间。
    gicp_.setMaximumIterations(config["gicp_maximum_iterations"].as<int>());

    // 设置变换增量的阈值。此参数控制了算法的收敛精度，当变换的变化量小于此值时，算法认为已经收敛。
    gicp_.setTransformationEpsilon(config["gicp_transform_epsilon"].as<double>());

    // 设置欧几里得适配度阈值，用于评估点云配准的效果，通常配准越好，该值越小。
    gicp_.setEuclideanFitnessEpsilon(config["gicp_euclidean_fitness_epsilon"].as<double>());

    // 设置RANSAC的离群点剔除阈值，用于剔除不符合配准模型的离群点，提升配准的鲁棒性。
    gicp_.setRANSACOutlierRejectionThreshold(config["gicp_ransac_outlier_rejection_threshold"].as<double>());

    // 指定法向量计算是否使用K近邻搜索，通常用在法向量估计时，设置为true表示使用。
    normal_use_knn_search = config["gicp_normal_use_knn_search"].as<bool>();

    // 指定法向量计算的搜索半径，用于选择法向量估计时考虑的临近点范围。
    normal_search_radius = config["gicp_normal_search_radius"].as<double>();

    // 指定K近邻算法的邻居数量，用于法向量计算，控制考虑的最近点数量。
    normal_search_k_neighbours = config["gicp_normal_search_k_neighbours"].as<int>();

    // 设置信息矩阵的对角线值（通常为3个数值，分别对应x\y\z方向）。信息矩阵用于配准后位姿的优化，指定不同方向上的不确定性，用于约束配准结果的准确性。
    info_diag_values = config["gicp_info_diag"].as<std::vector<double>>();
}

SubmapObj SubmapRegistration::constructTrgSubmap(const SubmapsVec& submaps_set, std::vector<int>& overlaps, const DRNoise& dr_noise){

    // Merge submaps in overlaps into submap_trg
    
    SubmapObj submap_trg(dr_noise);
    std::cout << "Target submap consists of: ";
    for(SubmapObj submap_j: submaps_set){
        if(std::find(overlaps.begin(), overlaps.end(), submap_j.submap_id_) != overlaps.end()){
            std::cout << submap_j.submap_id_ << ", ";
            submap_trg.submap_pcl_ += submap_j.submap_pcl_;
        }
    }

    return submap_trg;
}

void SubmapRegistration::transformSubmap(SubmapObj& submap){

    // Apply tranform to submap frame
    Isometry3f rel_tf = Isometry3f (Isometry3f(Translation3f(ret_tf_.block<3,1>(0,3)))*
                                    Isometry3f(Quaternionf(ret_tf_.block<3,3>(0,0)).normalized()));

    submap.submap_tf_ = rel_tf * submap.submap_tf_;
}

double SubmapRegistration::consistencyErrorOverlap(const SubmapObj& trg_submap,
                                                   const SubmapObj& src_submap){

    // Compute consistency error in overlapped area
    PointsT submaps;
    submaps.push_back(trg_submap.submap_pcl_.getMatrixXfMap(3,4,0).transpose().cast<double>());
    submaps.push_back(src_submap.submap_pcl_.getMatrixXfMap(3,4,0).transpose().cast<double>());

    Eigen::MatrixXd error_vals;
    double consistency_rms_error;
    std::vector<std::vector<std::vector<MatrixXd>>> grid_maps = benchmark_.create_grids_from_matrices(submaps);
    tie(consistency_rms_error, error_vals) = benchmark_.compute_consistency_error(grid_maps);

    return (consistency_rms_error);

}


/**
 * @brief 使用广义迭代最近点算法（GICP）进行子图注册。
 *
 * 该函数通过GICP算法将源子图（src_submap）与目标子图（trg_submap）对齐，并修改源子图以反映对齐结果。
 * 
 * @param trg_submap 目标子图对象，用于作为参考进行对齐
 * @param src_submap 源子图对象，将被对齐到目标子图并在此过程中被修改
 * 
 * @return 返回布尔值，表示GICP算法是否成功收敛。如果算法收敛则返回true，否则返回false。
 */
bool SubmapRegistration::gicpSubmapRegistration(SubmapObj& trg_submap, SubmapObj& src_submap){
    //N.B. this function modifies the src_submap when aligning it to

    // 复制原始点云数据以便在副本上操作
    PointCloudT::Ptr src_pcl_ptr (new PointCloudT(src_submap.submap_pcl_));
    PointCloudT::Ptr trg_pcl_ptr (new PointCloudT(trg_submap.submap_pcl_));

    // 计算GICP协方差矩阵
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod(tree);
    if (normal_use_knn_search) {
        std::cout << "使用KNN搜索，邻居数: " << normal_search_k_neighbours << std::endl;
        ne.setKSearch(normal_search_k_neighbours);
    } else {
        std::cout << "使用半径搜索，半径: " << normal_search_radius << std::endl;
        ne.setRadiusSearch(normal_search_radius);
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals_src(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(src_pcl_ptr);
    ne.compute(*normals_src);

    pcl::PointCloud<pcl::Normal>::Ptr normals_trg(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(trg_pcl_ptr);
    ne.compute(*normals_trg);

    CovsVec covs_src;
    CovsVecPtr covs_src_ptr;
    pcl::features::computeApproximateCovariances(*src_pcl_ptr, *normals_src, covs_src);
    covs_src_ptr.reset(new CovsVec(covs_src));

    CovsVec covs_trg;
    CovsVecPtr covs_trg_ptr;
    pcl::features::computeApproximateCovariances(*trg_pcl_ptr, *normals_trg, covs_trg);
    covs_trg_ptr.reset(new CovsVec(covs_trg));

    // 计算GICP信息矩阵
    Eigen::Matrix3d avg_cov_src;
    avg_cov_src.setZero();
    for(unsigned int n=0; n<covs_src_ptr->size(); n++){
        avg_cov_src += covs_src_ptr->at(n);
    }
    avg_cov_src /= (covs_src_ptr->size());

    Eigen::Matrix3d avg_cov_trg;
    avg_cov_trg.setZero();
    for(unsigned int n=0; n<covs_trg_ptr->size(); n++){
        avg_cov_trg += covs_trg_ptr->at(n);
    }
    avg_cov_trg /= (covs_trg_ptr->size());

    // 执行迭代最近点算法
    gicp_.setInputSource(src_pcl_ptr);
    gicp_.setInputTarget(trg_pcl_ptr);
    gicp_.setSourceCovariances(covs_src_ptr);
    gicp_.setTargetCovariances(covs_trg_ptr);
    gicp_.align (src_submap.submap_pcl_);

    // 应用变换到子图
    ret_tf_ =  gicp_.getFinalTransformation();
    this->transformSubmap(src_submap);

    std::cout << "源子图平均协方差: " << avg_cov_src << std::endl;
    std::cout << "目标子图平均协方差: " << avg_cov_trg << std::endl;

    std::cout << "最终变换矩阵: " << ret_tf_ << std::endl;

    // 设置GICP信息矩阵
    src_submap.submap_lc_info_.setZero();
    Eigen::VectorXd info_diag = Eigen::Map<Eigen::Vector4d>(info_diag_values.data());
    std::cout << "GICP信息矩阵对角线元素: " << info_diag << std::endl;
    src_submap.submap_lc_info_.bottomRightCorner(4,4) = info_diag.asDiagonal();

    Eigen::Matrix3d rot = ret_tf_.topLeftCorner(3,3).cast<double>();
    std::cout << "旋转矩阵: " << rot << std::endl;
    std::cout << "旋转矩阵转置: " << rot.transpose() << std::endl;

    Eigen::Matrix3d gicp_cov = avg_cov_trg + rot*avg_cov_src*rot.transpose();
    std::cout << "GICP协方差矩阵: " << gicp_cov << std::endl;
    src_submap.submap_lc_info_.topLeftCorner(2,2) = gicp_cov.topLeftCorner(2,2).inverse();

    for(int x=0; x<src_submap.submap_lc_info_.array().size(); x++){
        if(isnan(src_submap.submap_lc_info_.array()(x))){
            throw std::runtime_error("矩阵中存在NaN成分");
            std::exit(0);
        }
    }

    // TODO: 正确计算此值
    bool convergence = (gicp_.hasConverged())? true: false;
    return convergence;
}


bool SubmapRegistration::gicpSubmapRegistrationSimple(SubmapObj& trg_submap, SubmapObj& src_submap){

    // Copy the originals to work over them
    PointCloudT::Ptr src_pcl_ptr (new PointCloudT(src_submap.submap_pcl_));
    PointCloudT::Ptr trg_pcl_ptr (new PointCloudT(trg_submap.submap_pcl_));

    // The Iterative Closest Point algorithm
    gicp_.setInputSource(src_pcl_ptr);
    gicp_.setInputTarget(trg_pcl_ptr);
    gicp_.align (src_submap.submap_pcl_);

    // Apply transform to submap
    ret_tf_ =  gicp_.getFinalTransformation();
    this->transformSubmap(src_submap);

    // Check for nan values
    for(int x=0; x<src_submap.submap_lc_info_.array().size(); x++){
        if(isnan(src_submap.submap_lc_info_.array()(x))){
            throw std::runtime_error("Nan components in the matrix");
            std::exit(0);
        }
    }

    // TODO: compute this value properly
    bool convergence = (gicp_.hasConverged())? true: false;
    return convergence;
}

