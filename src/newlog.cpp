#include <sstream>
#include <deque>
#include "utils.hpp"
#include "localization_impl.hpp"

// #include "utm_projection.h"
// #include <proj_api.h>
// #include "/home/lhy/codecplus/test/proj_api.h"
// #include "include/Geocentric/LocalCartesian.hpp"

using namespace std;

int main()
{
  std::ifstream file("../data/陀螺仪数据_20240730.txt");

  if (!file.is_open())
  {
    std::cerr << "Error opening file." << std::endl;
    return 1;
  }

  std::deque<IMUDATA> imudeque_;
  std::deque<GPSDATA> gpsdeque_;
  std::vector<string> tokens_;

  std::string line, time;
  int idx = 0;
  int timestamp;

  while (std::getline(file, line))
  {
    if (line.substr(0, 4) == "MEDI")
    {
      tokens_ = SplitStringByComma(line);
      idx = 0;
      time = tokens_[3].substr(tokens_[3].find_last_of('=') + 1);
      timestamp = convertStringToIntTime(time);

      // std::cout << time << ", "
      //           << ExtractNumbersAndDots(tokens_[4]) << ", "
      //           << ExtractNumbersAndDots(tokens_[5]) << ", "
      //           << ExtractNumbersAndDots(tokens_[6]) << ", "
      //           << ExtractNumbersAndDots(tokens_[7]) << ", "
      //           << tokens_[8].substr(tokens_[8].find_first_of('=') + 1, 8)
      //           << std::endl;

      GPSDATA gpsdata;
      gpsdata.time = time;
      gpsdata.timestamp = timestamp;
      gpsdata.lockstatus = tokens_[8].substr(tokens_[8].find_first_of('=') + 1, 8);
      gpsdata.longitude = stod(ExtractNumbersAndDots(tokens_[4]));
      gpsdata.latitude = stod(ExtractNumbersAndDots(tokens_[5]));
      gpsdata.vehiclespeed = stod(ExtractNumbersAndDots(tokens_[6])) / 3.6;
      gpsdata.altitude = stod(ExtractNumbersAndDots(tokens_[7]));
      // gpsdata.vehicleheading = stoi(ExtractNumbersAndDots(tokens_[10]));

      gpsdeque_.emplace_back(gpsdata);
      // std::cout << gpsdata << std::endl;
    }
    else
    {
      line = line.substr(line.find_first_of(':') + 2);
      tokens_ = SplitStringByComma(line);
      tokens_[0] = tokens_[0].substr(tokens_[0].find_first_of('=') + 1);
      tokens_[0] = tokens_[0].substr(0, tokens_[0].find_first_of('(') - 1);
      tokens_[1] = tokens_[1].substr(tokens_[1].find_first_of('=') + 1);
      tokens_[1] = tokens_[1].substr(0, tokens_[1].find_first_of('(') - 1);

      // std::cout << tokens_[0] << "\t" << tokens_[1] << std::endl;

      IMUDATA imudata;
      imudata.index = idx++;
      imudata.timestamp = timestamp;
      // 参考深圳的重力加速度
      str2vector(tokens_[0], imudata.acceleration, 0.0978);
      str2vector(tokens_[1], imudata.anglespeed, 0.01 * M_PI / 180.0);

      imudeque_.emplace_back(imudata);
      // std::cout << imudata << std::endl;
    }
  }

  file.close();

  // 处理掉末尾的重复静止数据
  removeLastStaticData(gpsdeque_, imudeque_);

  // // 第一段行程 1722321074
  // while(gpsdeque_.back().timestamp > 1722321074) {
  //   gpsdeque_.pop_back();
  // }
  // while(imudeque_.back().timestamp > 1722321074) {
  //   imudeque_.pop_back();
  // }

  // while(imudeque_.front().timestamp > 1722321074) {
  //   imudeque_.pop_front();
  // }
  // while(imudeque_.back().timestamp > 1722324991) {
  //   imudeque_.pop_back();
  // }
  // std::cout << imudeque_.size() << std::endl;
  // double sum_ax = 0.0, sum_ay = 0.0, sum_wz = 0.0;
  // for(int i = 0; i < imudeque_.size(); ++i){
  //   sum_ax += imudeque_[i].acceleration[0];
  //   sum_ay += imudeque_[i].acceleration[1];
  //   sum_wz += imudeque_[i].anglespeed[2];
  // }

  // std::cout << sum_ax / imudeque_.size() << std::endl;
  // std::cout << sum_ay / imudeque_.size() << std::endl;
  // std::cout << sum_wz / imudeque_.size() << std::endl;

  std::shared_ptr<Localization> loc_ptr;
  loc_ptr = std::make_shared<Localization>();
  loc_ptr->set_Localization_Flag("0x786435");

  int initial_timestamp;
  int sz1 = gpsdeque_.size();

  // // TODO:
  while (gpsdeque_.front().timestamp < 1722319100)
  {
    gpsdeque_.pop_front();
  }

  while (!loc_ptr->Is_initial())
  {
    GPSDATA current_GPS = gpsdeque_.front();
    gpsdeque_.pop_front();
    loc_ptr->Initializer(current_GPS, gpsdeque_.front());
  }
  initial_timestamp = gpsdeque_.front().timestamp;
  std::cout << initial_timestamp << std::endl;
  gpsdeque_.pop_front();

  int sz2 = imudeque_.size();
  while (imudeque_.front().timestamp <= initial_timestamp)
  {
    if (imudeque_.front().timestamp == initial_timestamp)
    {
      loc_ptr->Predictor(imudeque_.front());
    }
    imudeque_.pop_front();
  }

  std::cout << sz1 << ", " << sz2 << std::endl;
  std::cout << gpsdeque_.size() << ", " << imudeque_.size() << std::endl;
  std::cout << sz1 - gpsdeque_.size() << ", " << sz2 - imudeque_.size() << std::endl;

  while (!gpsdeque_.empty())
  {
    // TODO
    if (gpsdeque_.front().timestamp > 1722320630)
    {
      break;
    }

    // update
    loc_ptr->Update(gpsdeque_.front());

    // predict
    while (!imudeque_.empty() && imudeque_.front().timestamp <= gpsdeque_.front().timestamp)
    {
      loc_ptr->Predictor(imudeque_.front());
      imudeque_.pop_front();
    }

    gpsdeque_.pop_front();
  }

  std::cout << gpsdeque_.size() << ", " << imudeque_.size() << std::endl;

  return 0;
}
