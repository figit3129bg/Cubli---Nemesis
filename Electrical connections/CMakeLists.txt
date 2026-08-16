cmake_minimum_required(VERSION 3.14)
project(moteus_drive)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG        main   # for a reproducible build, replace with a specific commit hash later
)
FetchContent_MakeAvailable(moteus)

add_executable(moteus_drive main.cpp)
target_link_libraries(moteus_drive moteus::cpp)
