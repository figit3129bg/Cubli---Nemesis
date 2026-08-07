cmake_minimum_required(VERSION 3.16)
project(cubli_lqr CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT UNIX)
    message(FATAL_ERROR
        "lqr targets a POSIX serial port (/dev/ttyUSB0) via termios; "
        "build this from WSL or Linux, not natively on Windows.")
endif()

include(FetchContent)
FetchContent_Declare(
    moteus
    GIT_REPOSITORY https://github.com/mjbots/moteus.git
    GIT_TAG        cpp/v1.0.0
)
FetchContent_MakeAvailable(moteus)

add_executable(lqr
    lqr.cpp
    functions.cpp
    serial_port.cpp
)

target_link_libraries(lqr PRIVATE moteus::cpp)
