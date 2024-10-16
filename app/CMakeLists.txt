#指定cmake最低版本
cmake_minimum_required(VERSION 3.9)

#设置工程名字
project(car_logging_app)

#指定语言版本  这里指定C++14
set(CMAKE_CXX_STANDARD 14)

# 输出测试
message("hello")
message(STATUS ${PROJECT_NAME})

# 设置共享库的路径
set(LIBRARY_PATH "${PROJECT_SOURCE_DIR}/../build")

add_library(libGeographiccc SHARED IMPORTED)
add_library(libLocalization SHARED IMPORTED)

set_target_properties(libGeographiccc PROPERTIES IMPORTED_LOCATION "${LIBRARY_PATH}/liblibGeographiccc.so")
set_target_properties(libLocalization PROPERTIES IMPORTED_LOCATION "${LIBRARY_PATH}/liblibLocalization.so")

add_executable(run_offline run_offline.cpp)

include_directories(
    # ${YAML_CPP_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/../3rdparty/GeographicLib/include/
    ${PROJECT_SOURCE_DIR}/../3rdparty/eigen/
    ${PROJECT_SOURCE_DIR}/../include/
)

target_link_libraries(run_offline
    libLocalization
    libGeographiccc
)

# add_executable(run_offline run_offline.cpp)
# target_link_libraries(run_offline
#     ${PROJECT_NAME}
# )