#pragma once

// PID
//  - prevError
//  - value:  the current value
//  - target: the target value
//  - kp:     factor for poportional
//  - kd:     factor for derivative
const double pid(double prevError, double error, const double kp, const double kd);

