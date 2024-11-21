# 项目声明
project(slam_ws_apps)

# 设置编译选项
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/)

# 查找所有依赖包
find_package(PCL REQUIRED)
find_package(VTK REQUIRED COMPONENTS 
    vtkCommonCore
    vtkRenderingCore
    vtkRenderingOpenGL2
    vtkRenderingFreeType
    vtkInteractionStyle
)

# 添加编译选项
add_definitions(${PCL_DEFINITIONS})

# 设置包含目录
include_directories(include
    ${PCL_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
)

## APPS
# bathy_slam_real
add_executable(bathy_slam_real
    src/test_slam_real.cpp
)

target_link_libraries(bathy_slam_real
    gicp_registration
    utils_g2o
    submaps_tools
    graph_construction
    ceres_optimizer
    read_g2o
    data_transforms
    benchmark
    std_data
    slam_solver
    ${YAML_CPP_LIBRARIES}
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
)

# read_auv_data
add_executable(read_auv_data
    src/read_auv_data.cpp
)

target_link_libraries(read_auv_data
    std_data 
    navi_data 
    gsf_data 
    all_data 
    xtf_data 
    csv_data 
    data_transforms 
    benchmark 
    submaps_tools 
    utils_visualization 
    ${VTK_LIBRARIES}
)