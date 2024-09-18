#pragma once

#include "utils.hpp"

class Localization
{
private:
    GeographicLib::LocalCartesian geoConverter;
    // YAML::Node config;
    
    bool initial_flag = false;
    bool vehicle_static = false;
    bool imu_calibration = false;

    std::deque<double> moving_distance_;
    std::deque<double> gps_moving_;
    std::deque<double> static_imu_ax_;
    std::deque<double> static_imu_ay_;
    std::deque<double> static_imu_wz_;

    IMUDATA current_imu_, last_imu_;

    double velocity = 0.0;
    double last_update_velocity = 0.0;
    double P_vel = 0.0;
    double bias_ax = 0.0;
    double bias_ay = 0.0;
    double bias_wz = 0.0;

    Pose egopose;
    Pose drpose;
    Pose gpspose;

    std::ofstream log_fout_;
    std::ofstream egopose_fout_;
    std::ofstream drpose_fout_;
    std::ofstream gpspose_fout_;
    std::ofstream velocity_fout_;
    std::ofstream imudata_fout_;
    std::ofstream gpsdata_fout_;
    
    std::string localization_flag_;
    std::string current_path_;
    std::string imudata_path_;
    std::string gpsdata_path_;

    std::string log_path_;
    std::string egopose_path_;
    std::string drpose_path_;
    std::string gpspose_path_;
    std::string velocity_path_;

    // Eigen::Matrix4f T1, T2;
    Eigen::Matrix3f F;
    Eigen::Matrix2f Q, R;
    Eigen::Matrix<float, 3, 2> L, K;
    Eigen::Matrix<float, 2, 3> H;

public:
    Localization(/* args */);
    ~Localization();

    void set_Localization_Flag(const std::string & str);

    void Initializer(const GPSDATA &p1, const GPSDATA &p2);

    bool Is_initial();

    void save_log(std::ofstream &fout, const Pose &pose);

    void Predictor(const IMUDATA &imu);

    void Update(const GPSDATA &gps);

    void CalibrateBias(const IMUDATA &imu);
};
