cmake_minimum_required(VERSION 3.14)
project(moteus_app CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG main   # pin to a commit hash for reproducibility
)
FetchContent_MakeAvailable(moteus)

# Open-loop motor smoke test: spins the wheel at a fixed velocity/torque,
# no IMU and no control law. Useful for confirming the fdcanusb link and
# moteus config work before trusting the balance controller with it.
add_executable(moteus_app moteus.cpp)
target_link_libraries(moteus_app moteus::cpp)

# Closed-loop LQR balance controller: reads theta_b/theta_b_dot from the
# ESP32 over USB serial and drives the moteus reaction-wheel motor.
add_executable(lqr_controller lqr.cpp serial_port.cpp Kalman.cpp)
target_link_libraries(lqr_controller moteus::cpp)