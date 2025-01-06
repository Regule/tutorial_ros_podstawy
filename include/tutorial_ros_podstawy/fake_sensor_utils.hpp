#ifndef FAKE_SENSORS_UTILS_HPP
#define FAKE_SENSORS_UTILS_HPP

#include <cmath>

namespace fake_sensors
{

class CyclicObstacle
{
public:
    CyclicObstacle(int center, double sigma, int limit);
    double operator[](int idx);
private:
    int _center;
    int _sigma;
    int _limit;
};

}

#endif //FAKE_SENSORS_UTILS_HPP
