#指定cmake最低版本
cmake_minimum_required(VERSION 3.9)

#设置工程名字
project(car_logging)

#指定语言版本  这里指定C++14
set(CMAKE_CXX_STANDARD 14)

# 输出测试
message("hello")
message(STATUS ${PROJECT_NAME})
add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY)

include_directories(
    ${PROJECT_SOURCE_DIR}/3rdparty/GeographicLib/include/
    ${PROJECT_SOURCE_DIR}/3rdparty/eigen/
    ${PROJECT_SOURCE_DIR}/include/
    ${PROJECT_SOURCE_DIR}/src/
)

file(GLOB_RECURSE GeographicLib_FILES ${PROJECT_SOURCE_DIR}/3rdparty/GeographicLib/src/*.cpp)

add_library(libLocalization SHARED
    ${GeographicLib_FILES}
    src/localization_impl.cpp
    src/loc_interface.cpp
    src/GdWorker.cpp
)



