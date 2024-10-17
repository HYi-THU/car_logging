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
    // loc_ptr->set_Gnss_Locked_Status(std::to_string(Status));
    loc_ptr->set_Gnss_Locked_Status(Status);
  }

  void GdWorker::SetSensorFrequency(const uint16_t frequency){
    loc_ptr->set_Imu_Frequency(frequency);
  }

  void GdWorker::SetSensor(const std::vector<Sensor> &arrSensor){
    loc_ptr->feed_imu_queue(arrSensor);
    // int idx = 0;
    // std::vector<IMUDATA> imu_queue;

    // for(auto &sen : arrSensor) {
    //   IMUDATA imudata;

    //   imudata.index = idx++;
    //   imudata.Utctime = sen.UtcTime;
    //   imudata.timestamp = 1e-3 * sen.TimeStamp;
    //   imudata.ax= 0.0978 * sen.AccX ;
    //   imudata.ay= 0.0978 * sen.AccY ;
    //   imudata.az= 0.0978 * sen.AccZ ;
    //   imudata.wx = 0.01 * M_PI / 180.0 * sen.AngX;
    //   imudata.wy = 0.01 * M_PI / 180.0 * sen.AngY;
    //   imudata.wz = 0.01 * M_PI / 180.0 * sen.AngZ;

    //   imu_queue.push_back(imudata);
    // }
    
    // loc_ptr->feed_imu_queue(imu_queue);
  }

  void GdWorker::SetGnss(const GnssInfo &sInfo){
    loc_ptr->feed_gnss(sInfo);
    // GPSDATA gpsdata;
    // gpsdata.Utctime = sInfo.UtcTime;
    // gpsdata.timestamp = 1e-3 * sInfo.TimeStamp;
    // gpsdata.lockstatus = std::to_string(sInfo.Status);
    // gpsdata.latitude = sInfo.LatitudeDegree;
    // gpsdata.longitude = sInfo.LongitudeDegree;
    // gpsdata.altitude = 1.0 * sInfo.GpsAltitude;
    // gpsdata.vehiclespeed = 1.0 * sInfo.GpsSpeed / 3.6;

    // loc_ptr->feed_gnss(gpsdata);
  }

  void GdWorker::GetResult(LocalizationResult &result){
    loc_ptr->GetPose(result);

    // Pose current_pose;
    // loc_ptr->GetPose(current_pose);

    // result.UtcTime = current_pose.Utctime;
    // result.LatitudeDegree = current_pose.latitude;
    // result.LongitudeDegree = current_pose.longtitude;

    // result.TimeStamp = (uint64_t)current_pose.timestamp * 1000;
  }

}
