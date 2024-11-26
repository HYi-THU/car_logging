#ifndef GD_DATA_TRANSMITTER_H
#define GD_DATA_TRANSMITTER_H

#include "GdApi.h"
#include "loc_interface.h"

namespace GdApi
{
  class GdWorker
  {
  public:
    GdWorker(void);
    ~GdWorker(void);

  public:
    // 0.1 使用前需要设定Gnss可用解的状态标志位
    void SetGnssLockStatus(const uint32_t status);

    // 0.2 使用前需要设定Sensor的数据频率
    void SetSensorFrequency(const uint16_t frequency);

    // 0.3 使用前需要设定保存Data的目录
    void set_Data_Dir(const std::string dir);

    // 1.传递六轴传感器数据
    void SetSensor(const std::vector<Sensor> &arrSensor);

    // 2.传递GPS数据
    void SetGnss(const GnssInfo &sInfo);

    // 3.获取定位结果
    void GetResult(LocalizationResult &result);

  private:
    std::shared_ptr<LocInterface> loc_ptr;
  };
} // namespace GdApi

#endif // GD_DATA_TRANSMITTER_H