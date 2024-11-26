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
set(LIBRARY_PATH "${PROJECT_SOURCE_DIR}/3rdparty/LocalizationLib/build")

add_library(libLocalization SHARED IMPORTED)

set_target_properties(libLocalization PROPERTIES IMPORTED_LOCATION "${LIBRARY_PATH}/liblibLocalization.so")

include_directories(
    # ${YAML_CPP_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/3rdparty/LocalizationLib/3rdparty/GeographicLib/include/
    ${PROJECT_SOURCE_DIR}/3rdparty/LocalizationLib/3rdparty/eigen/
    ${PROJECT_SOURCE_DIR}/3rdparty/LocalizationLib/include/
    ${PROJECT_SOURCE_DIR}/include/
)

add_executable(run_offline src/run_offline.cpp)
target_link_libraries(run_offline
    libLocalization
)

add_executable(run_offline_1030 src/run_offline_1030.cpp)
target_link_libraries(run_offline_1030
    libLocalization
)

add_executable(run_online src/run_online.cpp)
target_link_libraries(run_online
    libLocalization
)