#pragma once

#include <memory>
#include <vector>

namespace GdApi
{
  // 六轴传感器数据
  struct Sensor
  {
    int AccX;           // x轴加速度值, 单位1/100g
    int AccY;           // y轴加速度值, 单位1/100g
    int AccZ;           // z轴加速度值, 单位1/100g
    int AngX;           // x轴角速度值, 单位1/100度/秒
    int AngY;           // y轴角速度值, 单位1/100度/秒
    int AngZ;           // z轴角速度值, 单位1/100度/秒
    uint64_t TimeStamp; // 时间戳, ms
    uint32_t UtcTime;   // Utc时间, 相对于2000-01-01 00:00:00过去的秒数
  };

  // 定位数据
  struct GnssInfo
  {
    double LatitudeDegree;  // 纬度(度)
    double LongitudeDegree; // 经度(度)
    double HdopValue;       // HDOP水平因子
    uint64_t TimeStamp;     // 时间戳, ms
    uint32_t AlarmFlag;     // 报警标志
    uint32_t Status;        // 状态位 0=未定位，1=单点解，2=差分定位，3=PPS解，4=RTK固定解，5=RTK浮点解
    uint32_t UtcTime;       // Utc时间, 相对于2000-01-01 00:00:00过去的秒数
    uint16_t GpsAltitude;   // 海拔高度(米)
    uint16_t GpsSpeed;      // 速度 1/10km/h
    uint16_t GpsAngle;      // 方向 0-359, 正北为0, 顺时针
    uint8_t Satellite;      // 卫星颗数
  };

  // 定位结果
  struct LocalizationResult
  {
    // WGS-84 输出
    double LatitudeDegree;  // 纬度(度)
    double LongitudeDegree; // 经度(度)

    // 方向 0-359, 正北为0, 顺时针
    double Heading;

    // 看需求ENU 输出(东北天坐标系) 
    double pos_E;
    double pos_N;
    double pos_U;

    uint64_t TimeStamp; // 时间戳, ms
    uint32_t UtcTime;   // Utc时间, 相对于2000-01-01 00:00:00过去的秒数
  };
} // namespace GdApi