cmake_minimum_required(VERSION 3.10)
project(cubli_laptop_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find the installed Eigen3 package
find_package(Eigen3 REQUIRED)

# Define the standalone test binary
add_executable(test_kf_standalone
    src/test_kf_standalone.cpp
    src/kalman_filter.cpp
    src/imu_preprocessing.cpp
)

# Tell CMake to supply Eigen3's header paths directly to GCC
target_link_libraries(test_kf_standalone PRIVATE Eigen3::Eigen)