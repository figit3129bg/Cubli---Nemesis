cmake_minimum_required(VERSION 3.14)
project(moteus_app CXX)

include(FetchContent)
FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG main   # pin to a commit hash for reproducibility
)
FetchContent_MakeAvailable(moteus)

add_executable(moteus_app moteus.cpp)
target_link_libraries(moteus_app moteus::cpp)