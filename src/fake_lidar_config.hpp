#ifndef FAKE_LIDAR_HPP
#define FAKE_LIDAR_HPP

#include <utility>
#include "rclcpp/rclcpp.hpp"

class FakeLidarConfig
{
public:
    constexpr static const char* PARAM_MIN_RANGE = "min_range";
    constexpr static const char* PARAM_MAX_RANGE = "max_range";
    constexpr static const char* PARAM_MIN_ANGLE = "min_angle";
    constexpr static const char* PARAM_MAX_ANGLE = "max_angle";
    constexpr static const char* PARAM_SAMPLE_COUNT = "sample_count";
    constexpr static const char* PARAM_SAMPLING_FREQUENCY = "sampling_frequency";
    constexpr static const char* PARAM_SIGMA = "sigma";

    constexpr static const double DEFAULT_MIN_RANGE = 0.2;
    constexpr static const double DEFAULT_MAX_RANGE = 2.0;
    constexpr static const double DEFAULT_MIN_ANGLE = -M_PI;
    constexpr static const double DEFAULT_MAX_ANGLE = M_PI;
    constexpr static const int DEFAULT_SAMPLE_COUNT = 360;
    constexpr static const double DEFAULT_SAMPLING_FREQUENCY = 10.0;
    constexpr static const double DEFAULT_SIGMA = 1.0;

    constexpr static const char* DESCRIPTION_MIN_RANGE =
    "minimum range value [m]";
    constexpr static const char* DESCRIPTION_MAX_RANGE =
    "maximum range value [m]";
    constexpr static const char* DESCRIPTION_MIN_ANGLE =
    "start angle of the scan [rad]";
    constexpr static const char* DESCRIPTION_MAX_ANGLE =
    "end angle of the scan [rad]";
    constexpr static const char* DESCRIPTION_SAMPLE_COUNT =
    "Number of samples per full laser scan";
    constexpr static const char* DESCRIPTION_SAMPLING_FREQUENCY =
    "Number of full Scans per second.";
    constexpr static const char* DESCRIPTION_SIGMA =
    "Standard deviation of obstacle (it is essentialy normal distribution pushed into circle).";

    constexpr static const double epsilon = 0.00001;

public:
    std::pair<double,double> range;
    std::pair<double,double> angle;
    int sample_count;
    double sampling_frequency;
    double sigma;

public:
    void declare(rclcpp::Node *node);
    void update(rclcpp::Node *node);
    void print(rclcpp::Node *node);

    double get_scan_step() const;
    int get_scan_period_ms() const;
    double get_scaled_sample(double scale) const;

};

#endif // FAKE_LIDAR_HPP