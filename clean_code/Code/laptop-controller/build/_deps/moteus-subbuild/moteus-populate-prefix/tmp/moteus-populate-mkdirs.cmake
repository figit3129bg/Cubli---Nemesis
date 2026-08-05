# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/ubuntu_varty/projects/2d_cubli/Code/laptop-controller/build/_deps/moteus-src")
  file(MAKE_DIRECTORY "/home/ubuntu_varty/projects/2d_cubli/Code/laptop-controller/build/_deps/moteus-src")
endif()
file(MAKE_DIRECTORY
  "/home/ubuntu_varty/projects/2d_cubli/Code/laptop-controller/build/_deps/moteus-build"
  "/home/ubuntu_varty/projects/2d_cubli/Code/laptop-controller/build/_deps/moteus-subbuild/moteus-populate-prefix"
  "/home/ubuntu_varty/projects/2d_cubli/Code/laptop-controller/build/_deps/moteus-subbuild/moteus-populate-prefix/tmp"
  "/home/ubuntu_varty/projects/2d_cubli/Code/laptop-controller/build/_deps/moteus-subbuild/moteus-populate-prefix/src/moteus-populate-stamp"
  "/home/ubuntu_varty/projects/2d_cubli/Code/laptop-controller/build/_deps/moteus-subbuild/moteus-populate-prefix/src"
  "/home/ubuntu_varty/projects/2d_cubli/Code/laptop-controller/build/_deps/moteus-subbuild/moteus-populate-prefix/src/moteus-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/ubuntu_varty/projects/2d_cubli/Code/laptop-controller/build/_deps/moteus-subbuild/moteus-populate-prefix/src/moteus-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/ubuntu_varty/projects/2d_cubli/Code/laptop-controller/build/_deps/moteus-subbuild/moteus-populate-prefix/src/moteus-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
