# project
cmake_minimum_required (VERSION 2.6)
project (libviso2)
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -msse3")

find_package( OpenCV REQUIRED )

# directories
set (LIBVISO2_SRC_DIR src)

# include directory
include_directories("${LIBVISO2_SRC_DIR}")

# sources
FILE(GLOB LIBVISO2_SRC_FILES "src/*.cpp")

# make release version
set(CMAKE_BUILD_TYPE Release)

# demo program
add_executable(slam_viso2 ${LIBVISO2_SRC_FILES})
target_link_libraries (slam_viso2 tiobj ${OpenCV_LIBS})

install(TARGETS slam_viso2 DESTINATION bin)
