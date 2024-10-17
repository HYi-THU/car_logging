#include "localization_impl.h"

Localization::Localization()
{
  // std::ifstream fin_config("/home/lhy/codecplus/test/config.yaml");
  // if (!fin_config.is_open())
  // {
  //     std::cerr << "Error: Unable to open file 'config.yaml'." << std::endl;
  // }

  // config = YAML::Load(fin_config);

  // bias_ax = config["bias_ax"].as<double>();
  // bias_ay = config["bias_ay"].as<double>();
  // bias_wz = config["bias_wz"].as<double>();
  bias_ax = -0.374946;
  bias_ay = -0.415610;
  bias_wz = -0.015;

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

  // Current path is: /home/lhy/codecplus/car_logging/build
  // Current working directory: /home/lhy/codecplus/car_logging/build
  std::cout << "Current working directory: " << current_path_ << std::endl;

  log_path_ = current_path_ + "/../log/log.txt";
  log_fout_.open(log_path_, std::ios::trunc);

  egopose_path_ = current_path_ + "/../log/egopose.txt";
  drpose_path_ = current_path_ + "/../log/drpose.txt";
  gpspose_path_ = current_path_ + "/../log/gpspose.txt";
  velocity_path_ = current_path_ + "/../log/veldata.txt";
  imudata_path_ = current_path_ + "/../log/imudata.txt";
  gpsdata_path_ = current_path_ + "/../log/gpsdata.txt";

  if (save_flag)
  {
    egopose_fout_.open(egopose_path_, std::ios::trunc);
    drpose_fout_.open(drpose_path_, std::ios::trunc);
    gpspose_fout_.open(gpspose_path_, std::ios::trunc);
    imudata_fout_.open(imudata_path_, std::ios::trunc);
    gpsdata_fout_.open(gpsdata_path_, std::ios::trunc);
    velocity_fout_.open(velocity_path_, std::ios::trunc);
    velocity_fout_ << "timestamp gps_vel pred_vel diff_with_last_update delta_vel update_vel pred_P update_P"
                   << std::endl;
  }
}

Localization::~Localization()
{
  log_fout_.close();
  egopose_fout_.close();
  drpose_fout_.close();
  gpspose_fout_.close();
  velocity_fout_.close();
}

void Localization::set_Gnss_Locked_Status(const uint32_t status)
{
  gnss_locked_status_ = std::to_string(status);
}

void Localization::set_Imu_Frequency(const uint16_t frequency)
{
  imu_frequency_ = frequency;
  imu_dt_ = 1.0 / imu_frequency_;
}

void Localization::feed_imu_queue(const std::vector<GdApi::Sensor> &arrSensor)
{
  int idx = 0;
  std::vector<IMUDATA> imu_queue;

  for (auto &sen : arrSensor)
  {
    IMUDATA imudata;

    imudata.index = idx++;
    imudata.Utctime = sen.UtcTime;
    imudata.timestamp = 1e-3 * sen.TimeStamp;
    imudata.ax = 0.0978 * sen.AccX;
    imudata.ay = 0.0978 * sen.AccY;
    imudata.az = 0.0978 * sen.AccZ;
    imudata.wx = 0.01 * M_PI / 180.0 * sen.AngX;
    imudata.wy = 0.01 * M_PI / 180.0 * sen.AngY;
    imudata.wz = 0.01 * M_PI / 180.0 * sen.AngZ;

    imu_queue.push_back(imudata);
  }

  this->feed_imu_queue(imu_queue);
}

void Localization::feed_gnss(const GdApi::GnssInfo &sInfo)
{
  GPSDATA gpsdata;
  gpsdata.Utctime = sInfo.UtcTime;
  gpsdata.timestamp = 1e-3 * sInfo.TimeStamp;
  gpsdata.lockstatus = std::to_string(sInfo.Status);
  gpsdata.latitude = sInfo.LatitudeDegree;
  gpsdata.longitude = sInfo.LongitudeDegree;
  gpsdata.altitude = 1.0 * sInfo.GpsAltitude;
  gpsdata.vehiclespeed = 1.0 * sInfo.GpsSpeed / 3.6;

  this->feed_gnss(gpsdata);
}

void Localization::GetPose(GdApi::LocalizationResult &result)
{
  double lat, lon, alt;
  geoConverter.Reverse(egopose.x, egopose.y, egopose.z, lat, lon, alt);

  result.UtcTime = egopose.Utctime;
  result.LatitudeDegree = lat;
  result.LongitudeDegree = lon;
  result.Heading = egopose.phi / M_PI * 180.0;

  result.TimeStamp = (uint64_t)egopose.timestamp * 1000;
}

void Localization::Initializer(const GPSDATA &p1, const GPSDATA &p2)
{
  if (p1.lockstatus != gnss_locked_status_ || p2.lockstatus != gnss_locked_status_)
    return;

  double x, y, z;
  geoConverter.Reset(p1.latitude, p1.longitude, 0.0);
  geoConverter.Forward(p2.latitude, p2.longitude, 0.0, x, y, z);

  double distance = sqrt(x * x + y * y);

  if ((abs(x) > 2 || abs(y) > 2) && distance > 5.0 && abs(x) < 40.0 && abs(y) < 40.0)
  {
    std::cout << "----------- Initializer Finish -----------" << std::endl;
    std::cout << p1 << std::endl;
    std::cout << p2 << std::endl;
    std::cout << "------------------------------------------" << std::endl;

    // 弧度单位
    double phi = atan2(y, x);

    gpspose.timestamp = egopose.timestamp = drpose.timestamp = p1.timestamp;
    egopose.vel = drpose.vel = distance;
    egopose.phi = drpose.phi = phi;

    Eigen::Quaterniond Q = convertPhiToQuaterniond(phi);
    egopose.qx = drpose.qx = Q.x();
    egopose.qy = drpose.qy = Q.y();
    egopose.qz = drpose.qz = Q.z();
    egopose.qw = drpose.qw = Q.w();
    save_log(egopose_fout_, egopose);
    save_log(drpose_fout_, drpose);
    save_log(gpspose_fout_, gpspose);

    gpspose.timestamp = egopose.timestamp = drpose.timestamp = p2.timestamp;
    gpspose.x = egopose.x = drpose.x = x;
    gpspose.y = egopose.y = drpose.y = y;
    gpspose.z = egopose.z = drpose.z = z;

    egopose.P = Eigen::Matrix3f::Zero();
    egopose.P.row(0).col(0) << 0.09;
    egopose.P.row(1).col(1) << 0.09;
    egopose.P.row(2).col(2) << 4 / 180 * M_PI / 180 * M_PI;

    save_log(egopose_fout_, egopose);
    save_log(drpose_fout_, drpose);
    save_log(gpspose_fout_, gpspose);

    initial_flag = true;
    initial_timestamp = p1.timestamp;
  }
}

bool Localization::Is_initial()
{
  return initial_flag;
}

void Localization::save_log(std::ofstream &fout, const Pose &pose)
{
  if (save_flag)
  {
    fout << std::fixed << std::setprecision(2) << pose.timestamp << " "
         << std::fixed << std::setprecision(4) << pose.x << " "
         << pose.y << " " << pose.z << " "
         << pose.qx << " " << pose.qy << " "
         << pose.qz << " " << pose.qw << std::endl;
  }
}

void Localization::correct_Phi(double &phi)
{
  if (phi > M_PI)
  {
    phi -= 2 * M_PI;
  }
  else if (phi < -M_PI)
  {
    phi += 2 * M_PI;
  }
}

Eigen::Quaterniond Localization::convertPhiToQuaterniond(const double phi)
{
  Eigen::AngleAxisd rotation_vector = Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond Q(rotation_vector);
  return Q;
}

void Localization::CalibrateBias(const IMUDATA &imu)
{
  static_imu_ax_.emplace_back(imu.ax);
  static_imu_ay_.emplace_back(imu.ay);
  static_imu_wz_.emplace_back(imu.wz);

  // std::cout << static_imu_ax_.size() << ", "
  //           << std::fixed << std::setprecision(4)
  //           << std::accumulate(static_imu_ax_.begin(), static_imu_ax_.end(), 0.0) / static_imu_ax_.size() << ", "
  //           << std::accumulate(static_imu_ay_.begin(), static_imu_ay_.end(), 0.0) / static_imu_ay_.size() << ", "
  //           << std::accumulate(static_imu_wz_.begin(), static_imu_wz_.end(), 0.0) / static_imu_wz_.size() << std::endl;

  if (static_imu_ax_.size() > 50)
  {
    // bias_ax = std::accumulate(static_imu_ax_.begin(), static_imu_ax_.end(), 0.0) / static_imu_ax_.size();
    // bias_ay = std::accumulate(static_imu_ay_.begin(), static_imu_ay_.end(), 0.0) / static_imu_ay_.size();
    // bias_wz = std::accumulate(static_imu_wz_.begin(), static_imu_wz_.end(), 0.0) / static_imu_wz_.size();
    imu_calibration = true;
  }

  while (static_imu_ax_.size() >= 200)
  {
    static_imu_ax_.pop_front();
    static_imu_ay_.pop_front();
    static_imu_wz_.pop_front();
  }
}

bool Localization::GnssFilter(const GPSDATA &gps)
{
  // 0. 数据收集阶段只根据数据标志进行简单筛选
  // 1. 在GPS对应的时间戳处插值形成IMU的队列
  // 2. 20个GPS节点 和 20个drpose节点进行粗匹配
  // 3. 超过3sigma误差的进行过滤

  // if (gps.lockstatus != gnss_locked_status_)
  // {
  //   gps_queue_.pop_back();
  //   return false;
  // }

  // if (gps_queue_.size() < 10)
  // {
  //   return true;
  // }

  return false;
}

void Localization::Predictor(const IMUDATA &imu)
{
  // if (imu.timestamp != (int)drpose.timestamp)
  //     return;
  if (save_flag)
  {
    imudata_fout_ << imu << std::endl;
  }

  // 通过重力加速度判断当前imu加速度计是否为无效数据
  if (abs(imu.az - 0.0) < 0.01)
  {
    // 无效时采用上一帧IMU数据
    current_imu_ = last_imu_;
    current_imu_.timestamp = imu.timestamp;
    current_imu_.index = imu.index;
  }
  else
  {
    current_imu_ = imu;
    if (vehicle_static)
    {
      CalibrateBias(imu);
      return;
    }
  }

  // 速度预测, 未标定按0.1g加速度误差计算，标定后按0.05g计算
  velocity += (current_imu_.ax - bias_ax) * 0.1;
  P_vel += imu_dt_ * imu_dt_ * (imu_calibration ? 0.49 : 0.98);

  double dS = abs(velocity * imu_dt_);
  moving_distance_.emplace_back(dS);
  while (moving_distance_.size() > 5)
  {
    moving_distance_.pop_front();
  }

  log_fout_ << "velocity: " << velocity << ", dS: " << dS << std::endl;
  // double dS = drpose.vel * imu_dt_ + imu.ax * 0.04;

  double nobias_wz = current_imu_.wz - bias_wz;

  drpose.timestamp = 1.0 * current_imu_.timestamp + imu_dt_ * (current_imu_.index + 1);
  drpose.x += cos(drpose.phi + nobias_wz * imu_dt_) * dS;
  drpose.y += sin(drpose.phi + nobias_wz * imu_dt_) * dS;
  drpose.phi += nobias_wz * imu_dt_;
  correct_Phi(drpose.phi);

  Eigen::Quaterniond q = convertPhiToQuaterniond(drpose.phi);
  drpose.qx = q.x();
  drpose.qy = q.y();
  drpose.qz = q.z();
  drpose.qw = q.w();

  save_log(drpose_fout_, drpose);

  egopose.timestamp = 1.0 * current_imu_.timestamp + imu_dt_ * (current_imu_.index + 1);
  egopose.x += cos(egopose.phi + nobias_wz * imu_dt_) * dS;
  egopose.y += sin(egopose.phi + nobias_wz * imu_dt_) * dS;
  egopose.phi += nobias_wz * imu_dt_;
  correct_Phi(egopose.phi);

  q = convertPhiToQuaterniond(egopose.phi);
  egopose.qx = q.x();
  egopose.qy = q.y();
  egopose.qz = q.z();
  egopose.qw = q.w();

  F = Eigen::Matrix3f::Identity();
  F.row(0).col(2) << -sin(egopose.phi + nobias_wz * imu_dt_) * dS;
  F.row(1).col(2) << cos(egopose.phi + nobias_wz * imu_dt_) * dS;

  L = Eigen::Matrix<float, 3, 2>::Zero();
  L.row(0).col(0) << cos(egopose.phi + nobias_wz * imu_dt_);
  L.row(0).col(1) << -sin(egopose.phi + nobias_wz * imu_dt_) * dS;
  L.row(1).col(0) << sin(egopose.phi + nobias_wz * imu_dt_);
  L.row(1).col(1) << cos(egopose.phi + nobias_wz * imu_dt_) * dS;
  L.row(2).col(1) << imu_dt_;

  Q = Eigen::Matrix2f::Zero();
  Q.row(0).col(0) << 0.09 * dS * dS;
  Q.row(1).col(1) << 0.015 * 0.015;

  log_fout_ << std::fixed << std::setprecision(2) << "------------------ idx "
            << current_imu_.timestamp + imu_dt_ * (current_imu_.index + 1) << " ------------------" << std::endl;

  log_fout_ << std::fixed << std::setprecision(4) << "F: \n"
            << F << std::endl;
  log_fout_ << "L: \n"
            << L << std::endl;
  log_fout_ << "Q: \n"
            << Q << std::endl;

  egopose.P = F * egopose.P * F.transpose() + L * Q * L.transpose();

  if (current_imu_.index != 4)
  {
    save_log(egopose_fout_, egopose);
  }

  // std::cout << std::fixed << std::setprecision(4)
  //           << "vel: " << drpose.vel << ", phi: " << drpose.phi
  //           << ", q: [" << Q.x() << ", " << Q.y() << ", "
  //           << Q.z() << ", " << Q.w() << "]" << std::endl;
}

void Localization::Update(const GPSDATA &gps)
{
  if (save_flag)
  {
    gpsdata_fout_ << gps << std::endl;
  }

  double x, y, z;
  geoConverter.Forward(gps.latitude, gps.longitude, 0.0, x, y, z);

  double delta_x = x - gpspose.x;
  double delta_y = y - gpspose.y;
  double delta_phi = atan2(delta_y, delta_x);

  // 速度更新
  double dS_gps = sqrt(pow(x - gpspose.x, 2) + pow(y - gpspose.y, 2));
  double dS_dr = std::accumulate(moving_distance_.begin(), moving_distance_.end(), 0.0);

  gps_moving_.emplace_back(dS_gps);
  while (gps_moving_.size() > 3)
  {
    gps_moving_.pop_front();
  }

  double mean_gps_moving = std::accumulate(gps_moving_.begin(), gps_moving_.end(), 0.0) / gps_moving_.size();

  if (abs(mean_gps_moving) < 0.5 && abs(dS_gps) < 0.1)
  {
    velocity = 0.0;
    vehicle_static = true;
  }
  else
  {
    vehicle_static = false;
  }

  // if (dS_dr > 0 && abs(dS_gps - dS_dr) / dS_dr > 2.0)
  //     return;

  // 对 dS 求导得到 1/dt, dt 为 1.0
  double H_vel = 1.0;
  // 对 dS 的观测精度分析, 假设前后两个点都具有 0.4m 的精度误差
  double R_vel = 0.16;
  // 当车速较高时，假设前后两个点都具有 0.2m 的精度误差

  if (dS_gps > 10.0 && dS_gps > 2.0 * dS_dr)
  {
    R_vel = 100;
  }
  if (velocity > 20.0)
  {
    R_vel = 0.04;
  }

  double K_vel = P_vel * H_vel * (1 / (H_vel * P_vel * H_vel + R_vel));
  double delta_vel = K_vel * (dS_gps - velocity);

  if (gps.vehiclespeed > 2.0)
  {
    velocity = gps.vehiclespeed;
  }
  // 当速度较高的情况下, 更新的速度量在原速度量的50%以内时进行更新
  else if (
      // abs(velocity) > 40.0 || abs(velocity) <= 5.0 ||
      (abs(velocity) > 5.0 && abs(delta_vel / velocity) > 0.5) ||
      std::isnan(delta_vel))
  {
    velocity = dS_gps;
  }
  else
  {
    if (save_flag)
    {
      velocity_fout_ << std::fixed << std::setprecision(1) << gps.timestamp
                     << std::fixed << std::setprecision(4)
                     << " " << gps.vehiclespeed
                     << " " << velocity
                     << " " << velocity - last_update_velocity
                     << " " << delta_vel
                     << " " << velocity + delta_vel
                     << " " << P_vel
                     << " " << (1 - K_vel * H_vel) * P_vel
                     << std::endl;
    }

    velocity += delta_vel;
    last_update_velocity = velocity;
    P_vel = (1 - K_vel * H_vel) * P_vel;
  }

  egopose.vel = drpose.vel = velocity;

  gpspose.timestamp = gps.timestamp;
  gpspose.x = x;
  gpspose.y = y;
  gpspose.z = z;

  if (round(mean_gps_moving * 100.0) > 1.0)
  {
    save_log(gpspose_fout_, gpspose);
  }

  // egopose 更新
  R = Eigen::Matrix2f::Zero();
  if (gps.lockstatus == gnss_locked_status_)
  {
    R.row(0).col(0) << 0.15;
    R.row(1).col(1) << 0.15;
  }
  else
  {
    R.row(0).col(0) << 100.0;
    R.row(1).col(1) << 100.0;
  }
  log_fout_ << "R: \n"
            << R << std::endl;

  H = Eigen::Matrix<float, 2, 3>::Zero();
  H.row(0).col(0) << 1;
  H.row(1).col(1) << 1;

  K = egopose.P * H.transpose() * (H * egopose.P * H.transpose() + R).inverse();
  log_fout_ << "K: \n"
            << K << std::endl;

  Eigen::Matrix<float, 2, 1> prior_delta;
  prior_delta.row(0).col(0) << x - egopose.x;
  prior_delta.row(1).col(0) << y - egopose.y;

  Eigen::Matrix<float, 3, 1> delta;
  delta = K * prior_delta;
  log_fout_ << "delta: \n"
            << delta.transpose() << std::endl;

  // if (abs(delta(0, 0)) > 100.0 || abs(delta(1, 0)) > 100.0)
  // {
  //     delta = Eigen::Matrix<float, 3, 1>::Zero();
  //     log_fout_ << "correct delta: \n"
  //               << delta.transpose() << std::endl;
  // }

  egopose.Utctime = gps.Utctime;
  egopose.P = (Eigen::Matrix3f::Identity() - K * H) * egopose.P;
  egopose.x += delta(0, 0);
  egopose.y += delta(1, 0);
  egopose.phi += delta(2, 0);
  correct_Phi(egopose.phi);

  log_fout_ << "x: " << egopose.x << ", y: " << egopose.y << ", phi: " << egopose.phi << std::endl;

  log_fout_ << "*****************************************************" << std::endl;

  Eigen::Quaterniond q = convertPhiToQuaterniond(egopose.phi);
  egopose.qx = q.x();
  egopose.qy = q.y();
  egopose.qz = q.z();
  egopose.qw = q.w();

  save_log(egopose_fout_, egopose);
}

void Localization::feed_imu_queue(std::vector<IMUDATA> imu_queue)
{
  for (auto &imu : imu_queue)
  {
    // std::cout << imu << std::endl;
    imu_buffer_.push_back(imu);
  }

  if (loc_data_queue_.empty() ||
      loc_data_queue_.back().Utctime < imu_buffer_.front().Utctime)
  {
    LocData loc_data;
    loc_data.Utctime = imu_buffer_.front().Utctime;
    while (!imu_buffer_.empty())
    {
      if (imu_buffer_.front().Utctime == loc_data.Utctime)
      {
        loc_data.imu_data_queue.push_back(imu_buffer_.front());
        if (loc_data.imu_data_queue.size() == imu_frequency_)
        {
          loc_data.imu_flag = true;
        }
        imu_buffer_.pop_front();
      }
      else
      {
        break;
      }
    }

    loc_data_queue_.push_back(loc_data);
  }
  else
  {

    for (auto &loc_data : loc_data_queue_)
    {
      while (!imu_buffer_.empty())
      {
        if (imu_buffer_.front().Utctime > loc_data.Utctime)
        {
          loc_data.imu_flag = true;
          break;
        }
        else if (imu_buffer_.front().Utctime == loc_data.Utctime)
        {
          loc_data.imu_data_queue.push_back(imu_buffer_.front());
          if (loc_data.imu_data_queue.size() == imu_frequency_)
          {
            loc_data.imu_flag = true;
          }
          imu_buffer_.pop_front();
        }
        else
        {
          imu_buffer_.pop_front();
        }
      }
    }
  }

  this->Run();

  // while (initial_flag && imu_buffer_.size() > 0)
  // {
  //   this->Predictor(imu_buffer_.front());
  //   imu_buffer_.pop_front();
  // }
}

void Localization::feed_gnss(const GPSDATA &gps)
{
  std::cout << gps << std::endl;

  if (loc_data_queue_.empty() ||
      loc_data_queue_.back().Utctime < gps.Utctime)
  {
    LocData loc_data;
    loc_data.Utctime = gps.Utctime;
    loc_data.gps_data = gps;
    loc_data.gps_flag = true;

    loc_data_queue_.push_back(loc_data);
  }
  else
  {
    for (auto &loc_data : loc_data_queue_)
    {
      if (loc_data.Utctime == gps.Utctime)
      {
        loc_data.gps_data = gps;
        loc_data.gps_flag = true;
        break;
      }
    }
  }

  this->Run();
}

void Localization::Run()
{
  // std::cout << loc_data_queue_.size() << "  " << loc_data_queue_.back().Utctime << std::endl;
  if (!initial_flag && loc_data_queue_.size() > 2)
  {
    GPSDATA BLH0 = loc_data_queue_.front().gps_data;
    loc_data_queue_.pop_front();
    this->Initializer(BLH0, loc_data_queue_.front().gps_data);
    if (initial_flag)
    {
      for (auto &imu : loc_data_queue_.front().imu_data_queue)
      {
        this->Predictor(imu);
      }
      loc_data_queue_.pop_front();
    }

    return;
  }

  if (initial_flag)
  {

    for (auto &loc_data : loc_data_queue_)
    {
      std::cout << "Header UtcTime: " << loc_data.Utctime << ", "
                // << "imu UtcTime: [" << loc_data.imu_data_queue.front().timestamp
                // << "," << loc_data.imu_data_queue.back().timestamp << "], "
                << loc_data.imu_flag << loc_data.gps_flag << "(" << imu_buffer_.size() << ")" << std::endl;
    }
  }

  while (initial_flag && !loc_data_queue_.empty())
  {
    LocData curr_loc_data = loc_data_queue_.front();
    if (curr_loc_data.imu_flag && curr_loc_data.gps_flag)
    {
      std::cout << "Header UtcTime: " << curr_loc_data.Utctime
                << ", imu UtcTime: [" << curr_loc_data.imu_data_queue.front().timestamp
                << "," << curr_loc_data.imu_data_queue.back().timestamp << "], "
                << curr_loc_data.imu_flag << curr_loc_data.gps_flag << "(" << imu_buffer_.size() << ")" << std::endl;

      // Filter

      // Update
      this->Update(curr_loc_data.gps_data);

      // Predict
      for (auto &imu : curr_loc_data.imu_data_queue)
      {
        this->Predictor(imu);
      }

      loc_data_queue_.pop_front();
    }
    else
    {
      break;
    }
  }
}

