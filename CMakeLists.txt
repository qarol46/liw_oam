cmake_minimum_required(VERSION 3.16)
project(liw_oam)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find ament build dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

# Find ROS2 dependencies
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find PCL dependencies
find_package(PCL REQUIRED COMPONENTS common io filters registration kdtree search surface segmentation features)

# Find other dependencies
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

# Handle robin-map library
if(EXISTS ${PROJECT_SOURCE_DIR}/thirdLibrary/tessil-src)
    add_library(robin_map INTERFACE)
    target_include_directories(robin_map INTERFACE
        ${PROJECT_SOURCE_DIR}/thirdLibrary/tessil-src/include
    )
    message(STATUS "Using local robin-map library")
else()
    # Alternative: use FetchContent if library is not present locally
    include(FetchContent)
    FetchContent_Declare(
        tessil
        GIT_REPOSITORY https://github.com/Tessil/robin-map.git
        GIT_TAG v1.2.1
    )
    
    if(NOT tessil_POPULATED)
        FetchContent_Populate(tessil)
        add_library(robin_map INTERFACE)
        target_include_directories(robin_map INTERFACE
            ${tessil_SOURCE_DIR}/include
        )
    endif()
endif()

# Create alias for consistency
add_library(tsl::robin_map ALIAS robin_map)

# Auto-initialize ament
ament_auto_find_build_dependencies()

# Include directories
include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

# Add executable
ament_auto_add_executable(lio_optimization
    src/lioOptimization.cpp
    src/optimize.cpp
    src/cloudProcessing.cpp
    src/imuProcessing.cpp
    src/initialization.cpp
    src/utility.cpp
    src/parameters.cpp
    src/poseParameterization.cpp
    src/lidarFactor.cpp
    src/imuFactor.cpp
)

# Link libraries
target_link_libraries(lio_optimization
    ${rclcpp_LIBRARIES}
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}
    ${Eigen3_LIBRARIES}
    robin_map
)

# Add compile definitions for PCL
target_compile_definitions(lio_optimization PRIVATE
    ${PCL_DEFINITIONS}
)

# Include directories for target
target_include_directories(lio_optimization PRIVATE
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)

# Install targets
install(TARGETS lio_optimization
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if any
if(EXISTS ${PROJECT_SOURCE_DIR}/launch)
    install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
    )
endif()

# Install configuration files if any
if(EXISTS ${PROJECT_SOURCE_DIR}/config)
    install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
    )
endif()

# Export dependencies
ament_export_dependencies(
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_eigen
    PCL
    Eigen3
    Ceres
)

# Export include directories
ament_export_include_directories(include)

# Finalize ament
ament_package()