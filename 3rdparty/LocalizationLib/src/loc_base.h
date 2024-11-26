#pragma once

#include <iostream>
#include <unistd.h>
#include <cstring>
#include <numeric>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <vector>
#include <deque>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

struct GPSDATA
{
    uint32_t Utctime = 0;
    double timestamp = 0.0;
    int vehicleheading = 0;
    double longitude = 0.0;
    double latitude = 0.0;
    double altitude = 0.0;
    double vehiclespeed = 0.0;
    std::string lockstatus;
};

struct IMUDATA
{
    int index;
    uint32_t Utctime = 0;
    double timestamp = 0.0;
    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;
    double wx = 0.0;
    double wy = 0.0;
    double wz = 0.0;
};

struct Pose
{
    uint32_t Utctime;
    double timestamp;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    double x = 0.0, y = 0.0, z = 0.0;
    Eigen::Matrix3f P;
    double vel;
    double phi;           // rad
    double heading = 0.0; // deg
    double latitude = 0.0;
    double longtitude = 0.0;
    double altitude = 0.0;
};

struct LocData
{
    uint32_t Utctime; // Utc时间, 相对于2000-01-01 00:00:00过去的秒数
    std::vector<IMUDATA> imu_data_queue;
    GPSDATA gps_data;
    bool gps_flag = false;
    bool imu_flag = false;
    bool update_flag = true;
};

// 模板化的重载 << 运算符，专门用于长度为3的 std::vector
template <typename T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &vec)
{
    if (vec.size() == 3)
    {
        os << "[" << vec[0] << ", " << vec[1] << ", " << vec[2] << "]";
    }
    else
    {
        os << "Vector size is not 3, cannot display.";
    }
    return os;
}

inline std::ostream &operator<<(std::ostream &os, const GPSDATA &gps)
{
    os << "TimeStamp: " << gps.Utctime
       << ", LockStatus: " << gps.lockstatus
       << ", Longitude: " << std::fixed << std::setprecision(6) << gps.longitude
       << ", Latitude: " << gps.latitude
       //    << ", Altitude: " << gps.altitude
       << ", Speed: " << std::fixed << std::setprecision(2) << gps.vehiclespeed << "m/s"
       << ", Heading: " << gps.vehicleheading;

    return os;
}

inline std::ostream &operator<<(std::ostream &os, const IMUDATA &imu)
{
    os << "TimeStamp: " << imu.timestamp << "(" << imu.Utctime << "), "
       << "acceleration: " << "[" << imu.ax << ", " << imu.ay << ", " << imu.az << "]" << ", \t"
       << "omega: " << "[" << imu.wx << ", " << imu.wy << ", " << imu.wz << "]";
    return os;
}

// inline void removeLastStaticData(std::deque<GPSDATA> &gps, std::deque<IMUDATA> &imu)
// {
//     double last_longitude, last_latitude;
//     int idx = 1;

//     while (idx <= 5)
//     {
//         last_longitude += gps[gps.size() - idx].longitude;
//         last_latitude += gps[gps.size() - idx].latitude;
//         ++idx;
//     }

//     last_longitude /= 5.0;
//     last_latitude /= 5.0;

//     std::cout << std::fixed << std::setprecision(6) << "last_longitude: " << last_longitude << "\n";
//     std::cout << "last_latitude: " << last_latitude << "\n";
//     std::cout << "Before GPSDATA size: " << gps.size() << "\n";
//     while (abs(gps[gps.size() - 5].longitude - last_longitude) < 1e-7 && abs(gps[gps.size() - 5].latitude - last_latitude) < 1e-7)
//     {
//         gps.pop_back();
//     }
//     std::cout << "After GPSDATA size: " << gps.size() << "\n";

//     std::cout << "Before IMUDATA size: " << imu.size() << "\n";
//     while (imu.back().timestamp != gps.back().timestamp)
//     {
//         imu.pop_back();
//     }
//     std::cout << "After IMUDATA size: " << imu.size() << "\n";
// }
