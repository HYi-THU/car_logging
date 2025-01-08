#include "GdWorker.h"
#include "loc_interface.h"

namespace GdApi
{
  GdWorker::GdWorker(void)
  {
    loc_ptr = LocInterface::make();
  }

  GdWorker::~GdWorker(void){
    loc_ptr = nullptr;
  }

  void GdWorker::SetGnssLockStatus(const uint32_t Status){
    loc_ptr->set_Gnss_Locked_Status(Status);
  }

  void GdWorker::SetSensorFrequency(const uint16_t frequency){
    loc_ptr->set_Imu_Frequency(frequency);
  }

  void GdWorker::set_Data_Dir(const std::string dir){
    loc_ptr->set_Data_Dir(dir);
  }

  void GdWorker::SetSensor(const std::vector<Sensor> &arrSensor){
    loc_ptr->feed_imu_queue(arrSensor);
  }

  void GdWorker::SetGnss(const GnssInfo &sInfo){
    loc_ptr->feed_gnss(sInfo);
  }

  void GdWorker::GetResult(LocalizationResult &result){
    loc_ptr->GetPose(result);
  }

}
