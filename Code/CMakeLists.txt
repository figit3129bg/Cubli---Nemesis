cmake_minimum_required(VERSION 3.14)
project(moteus_read)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG main
)
FetchContent_MakeAvailable(moteus)

add_executable(moteus_test src/moteus_n1.cpp)
target_link_libraries(moteus_test moteus::cpp)

