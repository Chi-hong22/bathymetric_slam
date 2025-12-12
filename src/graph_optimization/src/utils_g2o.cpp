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

#include "graph_optimization/utils_g2o.hpp"
#include <random>

namespace {
    // 标记dr随机数生成器种子是否已设置
    bool dr_seed_set_ = false;
    // 标记submap随机数生成器种子是否已设置
    bool submap_seed_set_ = false;
    // dr随机数生成器的种子值
    int dr_seed_ = 0;
    // submap随机数生成器的种子值
    int submap_seed_ = 0;
    // dr随机数生成器实例
    std::unique_ptr<std::mt19937> dr_rng_;
    // submap随机数生成器实例
    std::unique_ptr<std::mt19937> submap_rng_;

    /**
     * @brief 创建指定种子或生成随机种子
     * 
     * 如果输入的种子值非负，则直接返回该种子值；
     * 如果输入的种子值为负，则使用随机设备生成一个随机种子并返回。
     * 
     * @param seed 输入的种子值，若为负数则表示需要生成随机种子
     * @return 返回有效的种子值，非负整数
     */
    int makeSeedOrRandom(int seed) {
        if (seed >= 0) {
            return seed;
        }
        std::random_device rd;
        return static_cast<int>(rd());
    }
}

// 双通道种子初始化，保证 DR / 子图 噪声互不干扰
void initNoiseRNGs(int seed_dr, int seed_submap) {
    dr_seed_ = makeSeedOrRandom(seed_dr);
    submap_seed_ = makeSeedOrRandom(seed_submap);
    dr_rng_ = std::make_unique<std::mt19937>(dr_seed_);
    submap_rng_ = std::make_unique<std::mt19937>(submap_seed_);
    dr_seed_set_ = true;
    submap_seed_set_ = true;
}

std::mt19937& getDRNoiseRNG() {
    if (!dr_rng_) {
        dr_seed_ = makeSeedOrRandom(dr_seed_set_ ? dr_seed_ : -1);
        dr_rng_ = std::make_unique<std::mt19937>(dr_seed_);
        dr_seed_set_ = true;
    }
    return *dr_rng_;
}

std::mt19937& getSubmapNoiseRNG() {
    if (!submap_rng_) {
        submap_seed_ = makeSeedOrRandom(submap_seed_set_ ? submap_seed_ : -1);
        submap_rng_ = std::make_unique<std::mt19937>(submap_seed_);
        submap_seed_set_ = true;
    }
    return *submap_rng_;
}

int getCurrentDRSeed() {
    getDRNoiseRNG();
    return dr_seed_;
}

int getCurrentSubmapSeed() {
    getSubmapNoiseRNG();
    return submap_seed_;
}


using namespace std;
using namespace g2o;
using namespace Eigen;

Matrix<double, 6,6> generateGaussianNoise(GaussianGen& transSampler,
                                          GaussianGen& rotSampler,
                                          std::mt19937& rng){

    std::vector<double> noiseTranslation;
    std::vector<double> noiseRotation;
    noiseTranslation.push_back(3);
    noiseTranslation.push_back(3);
    noiseTranslation.push_back(0.001);
    noiseRotation.push_back(0.0001);
    noiseRotation.push_back(0.0001);
    noiseRotation.push_back(0.001);

    Eigen::Matrix3d transNoise = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; ++i)
      transNoise(i, i) = std::pow(noiseTranslation[i], 2);

    Eigen::Matrix3d rotNoise = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; ++i)
      rotNoise(i, i) = std::pow(noiseRotation[i], 2);

    // Information matrix of the distribution
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
    information.block<3,3>(0,0) = transNoise.inverse();
    information.block<3,3>(3,3) = rotNoise.inverse();

    // Gaussian noise generators
    transSampler.setDistribution(transNoise);
    rotSampler.setDistribution(rotNoise);

    // 使用传入的确定性 RNG（对应 DR 或子图 通道），确保可复现性
    transSampler.seed(rng());
    rotSampler.seed(rng());
    return information;
}

/**
 * @brief 为子地图添加噪声，模拟传感器或位姿估计中的不确定性
 * 
 * 该函数通过添加高斯噪声来扰动子地图的位姿变换（平移和旋转），
 * 并使用扰动后的位姿对点云进行变换，从而生成带有噪声的子地图。
 * 
 * @param transSampler 平移噪声生成器，用于生成平移方向上的高斯噪声样本（当前未启用）
 * @param rotSampler 旋转噪声生成器，用于生成旋转方向上的高斯噪声样本
 * @param submap 子地图对象，包含原始点云和位姿变换，函数将直接修改其内容
 */
void addNoiseToSubmap(GaussianGen& transSampler,
                      GaussianGen& rotSampler,
                      SubmapObj& submap){

    // 提取子地图当前的旋转四元数和平移向量
    Eigen::Quaterniond gtQuat = (Eigen::Quaterniond)submap.submap_tf_.linear().cast<double>();
    Eigen::Vector3d gtTrans = submap.submap_tf_.translation().cast<double>();

    // 从旋转采样器中生成一个旋转噪声样本，并构造扰动四元数
    Eigen::Vector3d quatXYZ = rotSampler.generateSample();
    double qw = 1.0 - quatXYZ.norm();
    if (qw < 0) {
        qw = 0.;
        cerr << "x"; // 表示四元数归一化失败
    }

    // 当前代码中未启用平移噪声，而是引入了一个偏置在 yaw 方向的微小旋转噪声（子图专用 RNG）
    std::mt19937& gen = getSubmapNoiseRNG();
    std::normal_distribution<> d{0,0.05}; // yaw噪声 原参数0.1弧度(5.73°)

    // 构造 yaw 方向的小角度旋转作为扰动
    double roll = 0.0, pitch = 0.0, yaw = /*0.001*/ d(gen);
    Matrix3d m;
    m = AngleAxisd(roll, Vector3d::UnitX())
        * AngleAxisd(pitch, Vector3d::UnitY())
        * AngleAxisd(yaw, Vector3d::UnitZ());
    Eigen::Quaterniond rot(m);

    // 当前未启用平移噪声
    Eigen::Vector3d trans;
    trans.setZero();

    // 将噪声叠加到原始位姿上：先叠加平移，再叠加旋转
    trans = gtTrans + trans;
    rot = gtQuat * rot;

    // 构造带噪声的位姿变换
    Eigen::Isometry3d noisyMeasurement = (Eigen::Isometry3d) rot;
    noisyMeasurement.translation() = trans;

    // 使用带噪声的位姿对点云进行变换，并更新子地图的位姿
    pcl::transformPointCloud(submap.submap_pcl_, submap.submap_pcl_,
                             (noisyMeasurement.cast<float>() * submap.submap_tf_.inverse()).matrix());

    submap.submap_tf_ = noisyMeasurement.cast<float>();
}

/**
 * @brief 【未使用】为子地图序列添加噪声，模拟传感器或运动过程中的不确定性。
 *
 * 该函数对输入的子地图集合（submap_set）中除第一个外的每一个子地图，
 * 根据前一个子地图的位姿，计算当前子地图相对变换，并加入高斯噪声，
 * 然后更新当前子地图的点云和位姿。
 *
 * @param transSampler 用于平移噪声采样的高斯分布生成器（未使用）
 * @param rotSampler   用于旋转噪声采样的高斯分布生成器（未使用）
 * @param submap_set   子地图集合，每个子地图包含点云和位姿信息
 */
void addNoiseToMap(GaussianGen& transSampler,
                   GaussianGen& rotSampler,
                   SubmapsVec& submap_set){

    // 打印每个子地图与其前一个子地图之间的变换矩阵（调试用途）
    for (size_t i =1; i < submap_set.size(); i++){
        std::cout << i << " -------" << std::endl;
        std::cout << submap_set.at(i-1).submap_tf_.matrix() << std::endl;
    }

    // 遍历所有子地图，从第二个开始，为其添加噪声
    for (size_t i =1; i < submap_set.size(); i++){
        // 获取前一个子地图的位姿
        Eigen::Isometry3f tf_prev = submap_set.at(i-1).submap_tf_;

        // 计算当前子地图相对于前一个子地图的真实变换
        Eigen::Isometry3f meas_i = tf_prev.inverse() * submap_set.at(i).submap_tf_;
        Eigen::Quaternionf gtQuat = (Eigen::Quaternionf)meas_i.linear();
        Eigen::Vector3f gtTrans = meas_i.translation();

        // 初始化随机数生成器和正态分布（标准差为0.5）
        std::mt19937& gen = getSubmapNoiseRNG();
        std::normal_distribution<> d{0,0.5};

        // 仅在偏航角（yaw）方向添加噪声，roll 和 pitch 保持为 0
        float roll = 0.0, pitch = 0.0, yaw = /*0.5*/ d(gen);
        Matrix3f m;
        m = AngleAxisf(roll, Vector3f::UnitX())
            * AngleAxisf(pitch, Vector3f::UnitY())
            * AngleAxisf(yaw, Vector3f::UnitZ());
        Eigen::Quaternionf rot(m);

        // 将噪声旋转与真实旋转结合
        rot = gtQuat * rot;

        // 构造带噪声的相对变换（平移部分保持不变）
        meas_i = (Eigen::Isometry3f) rot;
        meas_i.translation() = gtTrans;

        // 计算加入噪声后的当前子地图估计位姿
        Eigen::Isometry3f estimate_i = tf_prev * meas_i;

        // 输出调试信息
        std::cout << i << " -------" << std::endl;
        std::cout << tf_prev.matrix() << std::endl;
        std::cout << meas_i.matrix() << std::endl;
        std::cout << estimate_i.matrix() << std::endl;

        // 使用估计位姿对当前子地图的点云进行变换
        pcl::transformPointCloud(submap_set.at(i).submap_pcl_, submap_set.at(i).submap_pcl_,
                                 (estimate_i * submap_set.at(i).submap_tf_.inverse()).matrix());

        // 更新当前子地图的位姿为加入噪声后的估计值
        submap_set.at(i).submap_tf_ = estimate_i.cast<float>();
    }
}
