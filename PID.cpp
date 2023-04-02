#include "PID.hpp"

const double pid(double prevError, double error, const double kp, const double kd) {
    float dError = error - prevError;
    return error * kp + dError * kd;
}
