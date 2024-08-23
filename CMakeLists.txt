#指定cmake最低版本
cmake_minimum_required(VERSION 3.9)

#设置工程名字
project(car_logging)

#指定语言版本  这里指定C++14
set(CMAKE_CXX_STANDARD 14)

# 查找 Boost 库
find_package(Boost REQUIRED COMPONENTS thread system filesystem)
find_package(yaml-cpp REQUIRED)

# 设置 Boost 库的版本号
set(Boost_USE_STATIC_LIBS OFF) # 设置为 ON 如果你需要静态库
set(Boost_USE_MULTITHREADED ON) # 确保这个是 ON，因为 thread 组件是多线程的
set(Boost_USE_STATIC_RUNTIME OFF) # 设置为 ON 如果你需要静态运行时

# 输出测试
message("hello")
message(STATUS ${PROJECT_NAME})

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/3rdparty/GeographicLib/include/
    ${PROJECT_SOURCE_DIR}/3rdparty/eigen/
    ${PROJECT_SOURCE_DIR}/include/
)

file(GLOB_RECURSE GeographicLib_FILES ${PROJECT_SOURCE_DIR}/3rdparty/GeographicLib/src/*.cpp)
add_library(libGeographiccc SHARED ${GeographicLib_FILES})


add_library(libLocalization SHARED
    include/localization_impl.hpp
    src/localization_impl.cpp)

#生成可执行文件
add_executable(log_test src/log_test.cpp)
target_link_libraries (log_test 
    libLocalization
    libGeographiccc
    ${Boost_LIBRARIES}
    yaml-cpp
)

add_executable(newlog src/newlog.cpp)
target_link_libraries (newlog 
    libLocalization
    libGeographiccc
    ${Boost_LIBRARIES}
    yaml-cpp
)


