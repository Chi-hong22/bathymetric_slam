cmake_minimum_required(VERSION 3.16)
project(submaps_tools)

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/submaps)


find_package(PCL 1.10 REQUIRED)


include_directories(include ${PCL_INCLUDE_DIRS})

add_library(submaps_tools
	src/submaps.cpp
)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(submaps_tools PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
    PRIVATE src)


target_link_libraries(submaps_tools
        std_data
        navi_data
        data_transforms
        ${PCL_LIBRARY_DIRS}
)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS submaps_tools EXPORT SubmapsToolsConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}) 
# This is for Windows
# install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT SubmapsToolsConfig DESTINATION share/SubmapsTools/cmake)

# This makes the project importable from the build directory
export(TARGETS submaps_tools FILE SubmapsToolsConfig.cmake)
