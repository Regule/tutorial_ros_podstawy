#include "tutorial_ros_podstawy/fake_sensor_utils.hpp"

namespace fake_sensors
{

CyclicObstacle::CyclicObstacle(int center, double sigma, int limit)
{
    _center = center;
    _sigma = sigma;
    _limit = limit;
}

double CyclicObstacle::operator[](int idx)
{
    return idx/(double)_limit;
}

}