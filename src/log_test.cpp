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
  std::ifstream file1("../7月1日_Trck.log");
  std::ifstream file2("../7月1日_陀螺仪数据.log");

  if (!file1.is_open() || !file2.is_open())
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

  while (std::getline(file1, line))
  {
    tokens_ = SplitStringByComma(line);
    // std::cout << tokens_[1].substr(7, tokens_[1].size() - 7) << ", " << tokens_[3].substr(5, 4) << ", "
    //           << ExtractNumbersAndDots(tokens_[7]) << ", "
    //           << ExtractNumbersAndDots(tokens_[8]) << ", "
    //           << ExtractNumbersAndDots(tokens_[9]) << ", "
    //           << ExtractNumbersAndDots(tokens_[10]) << std::endl;

    time = tokens_[1].substr(7, tokens_[1].size() - 7);
    timestamp = convertStringToIntTime(time);
    // 加格林威治时间差， UTC -> UTC+8 东八区的时差
    timestamp += 28800;

    GPSDATA gpsdata;
    gpsdata.time = time;
    gpsdata.timestamp = timestamp;
    gpsdata.lockstatus = tokens_[3].substr(5, 4);
    gpsdata.longitude = stod(ExtractNumbersAndDots(tokens_[7]));
    gpsdata.latitude = stod(ExtractNumbersAndDots(tokens_[8]));
    gpsdata.vehiclespeed = stod(ExtractNumbersAndDots(tokens_[9])) / 3.6;
    gpsdata.vehicleheading = stoi(ExtractNumbersAndDots(tokens_[10]));
    gpsdata.altitude = 0.0;

    gpsdeque_.emplace_back(gpsdata);
    // std::cout << gpsdata << std::endl;
  }

  while (std::getline(file2, line))
  {
    if (line.substr(0, 7) == "LocTime")
    {
      idx = 0;
      time = line.substr(8, 19);
      timestamp = convertStringToIntTime(time);

      // GPSDATA gpsdata;
      // gpsdata.time = time;
      // gpsdata.timestamp = timestamp;
      // gpsdata.longitude = stod(line.substr(34, 4) + '0' + line.substr(38, 5));
      // gpsdata.latitude = stod(line.substr(49, 9));
      // gpsdata.vehiclespeed = stod(line.substr(65, line.find_first_of('(') - 65)) / 3.6;
      // gpsdata.altitude = stod(
      //     line.substr(line.find_last_of('=') + 1,
      //                 line.find_last_of('(') - line.find_last_of('=') - 1));

      // gpsdeque_.emplace_back(gpsdata);
      // std::cout << gpsdata << std::endl;
    }
    else
    {
      IMUDATA imudata;
      imudata.index = idx++;
      imudata.timestamp = timestamp;

      string substr1 = line.substr(line.find_first_of('=') + 1, line.size() - line.find_first_of('=') - 1);
      string substr2 = substr1.substr(substr1.find_first_of('=') + 1, substr1.find_last_of('(') - substr1.find_first_of('=') - 2);
      substr1 = substr1.substr(0, substr1.find_first_of('(') - 1);

      str2vector(substr1, imudata.acceleration, 0.0980);
      str2vector(substr2, imudata.anglespeed, 0.01 * M_PI / 180.0);

      imudeque_.emplace_back(imudata);
      // std::cout << imudata << std::endl;
    }
  }

  file1.close();
  file2.close();

  // 处理掉末尾的重复静止数据
  removeLastStaticData(gpsdeque_, imudeque_);

  while(gpsdeque_.front().timestamp < 1719812429) {
    gpsdeque_.pop_front();
  }
  while(imudeque_.front().timestamp < 1719812429) {
    imudeque_.pop_front();
  }

  std::shared_ptr<Localization> loc_ptr;
  loc_ptr = std::make_shared<Localization>();

  int initial_timestamp;
  int sz1 = gpsdeque_.size();
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
    // update
    loc_ptr->Update(gpsdeque_.front());

    if (abs(gpsdeque_.front().timestamp - 1719801742) < 5)
    {
      std::cout << gpsdeque_.front() << std::endl;
    }

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
