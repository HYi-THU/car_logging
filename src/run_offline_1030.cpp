#include <sstream>
#include <fstream>

#include "utils.hpp"
#include "GdWorker.h"

using namespace std;

int main()
{
  // 读取当前文件夹目录, 用于存放log
  std::string current_path_;
  char buffer[1024];
  if (getcwd(buffer, sizeof(buffer)) != nullptr)
  {
    std::string current_path_string(buffer);
    current_path_ = std::string(buffer);
    std::cout << "Current path is: " << current_path_string << std::endl;
  }
  else
  {
    std::cerr << "Failed to get current directory" << std::endl;
  }

  std::ifstream file("../data/20241030_100hz.txt");

  if (!file.is_open())
  {
    std::cerr << "Error opening file." << std::endl;
    return 1;
  }

  std::vector<GdApi::Sensor> Sensor_queue_;
  std::vector<GdApi::LocalizationResult> loc_result_queue_;
  std::vector<string> tokens_;

  std::string line, time;
  int idx = 0;
  uint32_t utc_time;

  std::shared_ptr<GdApi::GdWorker> work_ptr_;
  work_ptr_ = std::make_shared<GdApi::GdWorker>();
  work_ptr_->SetGnssLockStatus(4);
  work_ptr_->SetSensorFrequency(100);
  work_ptr_->set_Data_Dir(std::string("../data/"));

  while (std::getline(file, line))
  {
    if (line.substr(0, 5) == "GpsTm")
    {
      tokens_ = SplitStringByComma(line);
      idx = 0;
      time = tokens_[0].substr(tokens_[0].find_last_of('=') + 1);
      utc_time = convertStringToIntTime(time); 

      GdApi::GnssInfo gnssdata;
      gnssdata.UtcTime = utc_time;
      gnssdata.TimeStamp = (uint64_t)utc_time * 1000;
      gnssdata.LongitudeDegree = stod(ExtractNumbersAndDots(tokens_[2]));
      gnssdata.LatitudeDegree = stod(ExtractNumbersAndDots(tokens_[3]));
      gnssdata.GpsSpeed = 10 * stoi(ExtractNumbersAndDots(tokens_[4]));
      gnssdata.GpsAngle = stoi(ExtractNumbersAndDots(tokens_[5]));

      std::string status_str = tokens_[1].substr(tokens_[1].find_first_of('=') + 1);
      if (status_str == "0x83")
      {
        gnssdata.Status = 4;
      }
      else
      {
        gnssdata.Status = 0;
      }

      work_ptr_->SetSensor(Sensor_queue_);
      work_ptr_->SetGnss(gnssdata);

      GdApi::LocalizationResult loc_res;
      work_ptr_->GetResult(loc_res);
      loc_result_queue_.push_back(loc_res);

      Sensor_queue_.clear();
    }
    else
    {
      tokens_ = SplitStringByComma(line);
      tokens_[0] = tokens_[0].substr(tokens_[0].find_first_of('=') + 1);
      tokens_[0] = tokens_[0].substr(0, tokens_[0].find_first_of('(') - 1);
      tokens_[1] = tokens_[1].substr(tokens_[1].find_first_of('=') + 1);
      tokens_[1] = tokens_[1].substr(0, tokens_[1].find_first_of('(') - 1);

      std::vector<double> acceleration, anglespeed;
      str2vector(tokens_[0], acceleration, 1.0);
      str2vector(tokens_[1], anglespeed, 1.0);

      GdApi::Sensor sensor;
      sensor.UtcTime = utc_time;
      sensor.TimeStamp = (uint64_t)utc_time * 1000 + idx * 10;
      sensor.AccX = int(acceleration[0]);
      sensor.AccY = int(acceleration[1]);
      sensor.AccZ = int(acceleration[2]);
      sensor.AngX = int(anglespeed[0]);
      sensor.AngY = int(anglespeed[1]);
      sensor.AngZ = int(anglespeed[2]);

      Sensor_queue_.push_back(sensor);
      idx++;
    }
  }

  file.close();

  std::string loc_result_path_;
  std::ofstream loc_result_fout_;

  loc_result_path_ = current_path_ + "/../log/gps_points.txt";
  loc_result_fout_.open(loc_result_path_, std::ios::trunc);

  // 保存经纬度信息
  for (auto &loc_res : loc_result_queue_)
  {
    if(loc_res.LatitudeDegree > 0.1 && loc_res.LongitudeDegree > 0) {
      loc_result_fout_ << std::setprecision(9)
                     << loc_res.LatitudeDegree << ", "
                     << loc_res.LongitudeDegree << std::endl;
    }
  }
  std::cout << "finish!" << std::endl;

  return 0;
}
