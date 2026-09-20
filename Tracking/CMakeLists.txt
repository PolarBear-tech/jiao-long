cmake_minimum_required(VERSION 3.15)
project(tracking)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME} PRIVATE Ceres::ceres)

if (CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux" OR CMAKE_HOST_SYSTEM_NAME STREQUAL "Darwin")
	target_link_libraries(${PROJECT_NAME} PRIVATE X11 ${CMAKE_CURRENT_SOURCE_DIR}/lib/libtrace_target_${CMAKE_HOST_SYSTEM_PROCESSOR}.a)
elseif(CMAKE_HOST_SYSTEM_NAME STREQUAL "Windows")
	target_link_libraries(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib/libtrace_target.lib)
endif()
