#pragma once

#include "GdApi.h"

class LocInterface
{
private:
    /* data */
public:
    static std::shared_ptr<LocInterface> make();
    virtual ~LocInterface() = default;

public:
    // 0.1 使用前需要设定Gnss可用解的状态标志位
    virtual void set_Gnss_Locked_Status(const uint32_t status) = 0;

    // 0.2 使用前需要设定Sensor的数据频率
    virtual void set_Imu_Frequency(const uint16_t frequency) = 0;

    // 1.传递六轴传感器数据
    virtual void feed_imu_queue(const std::vector<GdApi::Sensor> &arrSensor) = 0;

    // 2.传递GPS数据
    virtual void feed_gnss(const GdApi::GnssInfo &sInfo) = 0;

    // 3.获取定位结果
    virtual void GetPose(GdApi::LocalizationResult &result) = 0;
};

