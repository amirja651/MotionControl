#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <PID_v1.h>
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
    PIDController(PIDConfig config, unsigned long sampleTime = 100);
    double input;   // Process variable (current position)
    double output;  // Controller output
    PID*   pid;     // PID controller instance

    void   begin();
    void   setTarget(double target);
    double getTarget() const;
    double getPositionError(double currentPosition) const;
    void   setInput(float input);
    double getOutput() const;

    double setpoint;  // Target position

private:
};

#endif  // PID_CONTROLLER_H