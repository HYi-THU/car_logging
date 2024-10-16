#指定cmake最低版本
cmake_minimum_required(VERSION 3.9)

#设置工程名字
project(car_logging)

#指定语言版本  这里指定C++14
set(CMAKE_CXX_STANDARD 14)

# 输出测试
message("hello")
message(STATUS ${PROJECT_NAME})

include_directories(
    # ${YAML_CPP_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/3rdparty/GeographicLib/include/
    ${PROJECT_SOURCE_DIR}/3rdparty/eigen/
    ${PROJECT_SOURCE_DIR}/include/
    ${PROJECT_SOURCE_DIR}/src/
)

file(GLOB_RECURSE GeographicLib_FILES ${PROJECT_SOURCE_DIR}/3rdparty/GeographicLib/src/*.cpp)
add_library(libGeographiccc SHARED ${GeographicLib_FILES})

add_library(libLocalization SHARED 
    src/localization_impl.cpp
    src/GdWorker.cpp
)


# #生成可执行文件
# add_executable(log_test src/log_test.cpp)
# target_link_libraries (log_test 
#     libLocalization
#     libGeographiccc
#     # yaml-cpp
# )

# add_executable(newlog src/newlog.cpp)
# target_link_libraries (newlog 
#     libLocalization
#     libGeographiccc
#     # yaml-cpp
# )


