#pragma once
#include "loc_base.h"
#include "GdApi.h"
#include "loc_interface.h"

#include <Geocentric.hpp>
#include <LocalCartesian.hpp>

class Localization : public LocInterface
{
public:


private:
    GeographicLib::LocalCartesian geoConverter;

    uint32_t initial_timestamp;
    bool initial_flag = false;      // 定位初始化
    bool vehicle_static = false;    // 车辆静止
    bool imu_calibration = false;   // imu标定

    // 给定 IMU 默认频率
    uint16_t imu_frequency_ = 5;
    double imu_dt_ = 0.2;
    
    std::string gnss_locked_status_;
    std::string current_path_;
    std::string log_path_;
    std::string data_dir_;
    std::ofstream log_fout_;
    std::ofstream data_fout_;

    IMUDATA current_imu_, last_imu_;
    std::deque<IMUDATA> imu_buffer_;
    std::deque<GPSDATA> gnss_filtered_queue_;
    std::deque<LocData> loc_data_queue_;

    std::deque<double> moving_distance_;
    std::deque<double> gps_moving_;
    std::deque<double> static_imu_ax_;
    std::deque<double> static_imu_ay_;
    std::deque<double> static_imu_wz_;

    double velocity = 0.0;
    double last_update_velocity = 0.0;
    double P_vel = 0.0;
    double bias_ax = 0.0;
    double bias_ay = 0.0;
    double bias_wz = 0.0;

    Pose egopose;
    Pose drpose;
    Pose gpspose;

    bool save_data_ = false;        // 设定存储数据的flag
    bool init_save_flag_ = false;   // 设定GPS存储在前, 因需要确认已经开始存入GPS

    // 存储中间数据
    bool save_flag = false;
    std::string egopose_path_;
    std::string drpose_path_;
    std::string gpspose_path_;
    std::string velocity_path_;
    std::string imudata_path_;
    std::string gpsdata_path_;

    std::ofstream egopose_fout_;
    std::ofstream drpose_fout_;
    std::ofstream gpspose_fout_;
    std::ofstream velocity_fout_;
    std::ofstream imudata_fout_;
    std::ofstream gpsdata_fout_;

    // Eigen::Matrix4f T1, T2;
    Eigen::Matrix3f F;
    Eigen::Matrix2f Q, R;
    Eigen::Matrix<float, 3, 2> L, K;
    Eigen::Matrix<float, 2, 3> H;

public:
    Localization(/* args */);
    ~Localization();

public:
    // 0.1 使用前需要设定Gnss可用解的状态标志位
    virtual void set_Gnss_Locked_Status(const uint32_t status);

    // 0.2 使用前需要设定Sensor的数据频率
    virtual void set_Imu_Frequency(const uint16_t frequency);

    // 0.3 使用前需要设定保存Data的目录
    virtual void set_Data_Dir(const std::string dir);

    // 1.传递六轴传感器数据
    virtual void feed_imu_queue(const std::vector<GdApi::Sensor> &arrSensor);

    // 2.传递GPS数据
    virtual void feed_gnss(const GdApi::GnssInfo &sInfo);

    // 3.获取定位结果
    virtual void GetPose(GdApi::LocalizationResult &result);

public:
    void Initializer(const GPSDATA &p1, const GPSDATA &p2);

    bool Is_initial();

    void save_log(std::ofstream &fout, const Pose &pose);

    void CalibrateBias(const IMUDATA &imu);

    bool GnssFilter(const GPSDATA &gps);

    void Predictor(const IMUDATA &imu);

    void Update(const GPSDATA &gps);

    void feed_imu_queue(std::vector<IMUDATA> imu_queue);

    void feed_gnss(const GPSDATA &gps);

    void Run();

private:
    void correct_Phi(double &phi);

    Eigen::Quaterniond convertPhiToQuaterniond(const double phi);
};
