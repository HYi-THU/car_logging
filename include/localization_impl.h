#pragma once
#include "utils.hpp"

class Localization
{
private:
    GeographicLib::LocalCartesian geoConverter;
    // YAML::Node config;

    uint32_t initial_timestamp;
    bool save_flag = false;
    bool initial_flag = false;
    bool vehicle_static = false;
    bool imu_calibration = false;

    uint16_t imu_frequency_ = 5;
    double imu_dt_ = 0.2;

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

    std::string gnss_locked_status_;
    std::string current_path_;
    std::string log_path_;
    std::string egopose_path_;
    std::string drpose_path_;
    std::string gpspose_path_;
    std::string velocity_path_;
    std::string imudata_path_;
    std::string gpsdata_path_;

    std::ofstream log_fout_;
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

    void set_Gnss_Locked_Status(const std::string &str);

    void set_Imu_Frequency(const uint16_t frequency);

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

    void GetPose(Pose &current_pose);

private:
    void correct_Phi(double &phi);

    Eigen::Quaterniond convertPhiToQuaterniond(const double phi);
};
