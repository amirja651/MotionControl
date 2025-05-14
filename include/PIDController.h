#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <cmath>

struct PIDConfig
{
    double Kp;
    double Ki;
    double Kd;
};

class PIDController
{
public:
    PIDController(PIDConfig config);
    double input;   // Process variable (current position)
    double output;  // Controller output

    void   begin();
    void   setTarget(double target);
    double getTarget() const;
    double getPositionError(double currentPosition, bool isRotational = false) const;
    void   setInput(float input);
    double getOutput() const;
    void   setOutputLimits(double min, double max);

    double setpoint;  // Target position

private:
};

#endif  // PID_CONTROLLER_H