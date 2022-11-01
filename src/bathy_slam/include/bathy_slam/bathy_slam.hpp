#ifndef BATHY_SLAM_HPP
#define BATHY_SLAM_HPP

#include "submaps_tools/submaps.hpp"

#include "graph_optimization/graph_construction.hpp"
#include "graph_optimization/utils_g2o.hpp"

#include "registration/gicp_reg.hpp"

class BathySlam{

public:

    GraphConstructor* graph_obj_;
    SubmapRegistration* gicp_reg_;

    BathySlam(GraphConstructor& graph_obj, SubmapRegistration& gicp_reg);
    ~BathySlam();

    SubmapsVec runOffline(SubmapsVec &submaps_gt, GaussianGen &transSampler, GaussianGen &rotSampler, bool add_gaussian_noise,
    double overlap_coverage, const DRNoise& dr_noise);
};


#endif //BATHY_SLAM_HPP
