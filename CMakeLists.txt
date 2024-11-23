cmake_minimum_required(VERSION 2.8)

project(bathymetric_slam)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置了 Debug 编译类型
set(CMAKE_BUILD_TYPE "Debug")

# 设置 OpenGL_GL_PREFERENCE 策略
set(OpenGL_GL_PREFERENCE GLVND)

# 核心依赖
find_package(Ceres 2.1.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.14.0 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(g2o REQUIRED)
find_package(yaml-cpp)
find_package(Boost REQUIRED COMPONENTS date_time)

# AUVLIB相关依赖
# find_package(LIBIGL REQUIRED QUIET)
find_package(LibXTF REQUIRED)
find_package(LibGSF REQUIRED)
find_package(EigenCereal REQUIRED)
find_package(DataTools REQUIRED)
find_package(SonarTracing REQUIRED)
find_package(AuvlibGlad REQUIRED)
find_package(BathyMaps REQUIRED)


include_directories(
    # 项目内部头文件
    src/submaps_tools/include 
    src/registration/include
    src/meas_models/include
    src/graph_optimization/include
    src/bathy_slam/include
    # 外部依赖头文件
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS} 
    ${G2O_INCLUDE_DIRS} 
    ${OPENCV_INCLUDE_DIRS}
    ${yaml_cpp_INCLUDE_DIRS}
) 


#PCL
add_definitions(${PCL_DEFINITIONS})

#ceres
add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})

#lib自定义库创建
set(CUSTOM_LIB
    # 添加各个模块的源文件
    # bathy_slam
    src/bathy_slam/src/bathy_slam.cpp
    # graph_optimization
    src/graph_optimization/src/ceres_optimizer.cpp
    src/graph_optimization/src/graph_construction.cpp
    src/graph_optimization/src/read_g2o.cpp
    src/graph_optimization/src/types.cpp
    src/graph_optimization/src/utils_g2o.cpp
    # meas_models
    src/meas_models/src/multibeam_simple.cpp
    # registration
    src/registration/src/gicp_reg.cpp
    src/registration/src/utils_registration.cpp
    src/registration/src/utils_visualization.cpp
    # submaps_tools
    src/submaps_tools/src/submaps.cpp
)

LIST(APPEND G2O_LIBS
    cxsparse
    cholmod
    g2o_cli g2o_ext_freeglut_minimal g2o_simulator
    g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d
    g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only
    g2o_types_sba g2o_types_slam3d g2o_csparse_extension
    g2o_opengl_helper g2o_solver_dense g2o_stuff
    g2o_types_sclam2d g2o_parser g2o_solver_pcg
    g2o_types_data g2o_types_sim3
)

set(THRIDPARTY_LIB
    # 添加各个模块的源文件
    std_data
    navi_data
    data_transforms

    ${G2O_LIBS}
    ceres
    ${GFLAGS_LIBRARIES}

    benchmark
    ${YAML_CPP_LIBRARIES}
    ${PCL_LIBRARIES}

)


# APP
add_executable(bathy_slam_real
    src/apps/src/test_slam_real.cpp
    ${CUSTOM_LIB}
)

# 链接依赖库
target_link_libraries(bathy_slam_real

    ${THRIDPARTY_LIB}
)

# read_auv_data 数据读取工具
# add_executable(read_auv_data
#     src/apps/src/read_auv_data.cpp
# )

# target_link_libraries(read_auv_data
#     std_data 
#     navi_data 
#     gsf_data 
#     all_data 
#     xtf_data 
#     csv_data 
#     data_transforms 
#     benchmark 
#     submaps_tools 
#     utils_visualization 
#     ${VTK_LIBRARIES}
# )

