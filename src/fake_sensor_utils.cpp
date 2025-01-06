#include "tutorial_ros_podstawy/fake_sensor_utils.hpp"

namespace fake_sensors
{

CyclicObstacle::CyclicObstacle(int center, double sigma, int limit)
{
    _center = center; // mi
    _sigma = sigma;
    _limit = limit;
}

double CyclicObstacle::operator[](int idx)
{
    if(idx>_center+_limit/2)
    {
        idx = idx - _center - _limit;
    }
    else if(idx < _center - _limit/2)
    {
        idx = _limit - _center + idx + 1;
    }
    else{
        idx = idx - _center;
    }

    return 1.0 - exp(0.0 - pow(idx-_center, 2) / (2*pow(_sigma, 2)));
}

}
