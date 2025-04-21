#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <PID_v1.h>
#include "MAE3Encoder.h"
#include "MotorController.h"

struct PIDConfig
{
    double Kp;
    double Ki;
    double Kd;
};

class PIDController
{
public:
    PIDController(MotorController* motor, MAE3Encoder* encoder, PIDConfig config, unsigned long sampleTime = 100);
    double input;   // Process variable (current position)
    double output;  // Controller output
    PID*   pid;     // PID controller instance

    void   begin();
    bool   update();
    void   setTarget(double target);
    double getTarget() const;
    void   setGains(double Kp, double Ki, double Kd);
    void   enable();
    void   disable();
    bool   isEnabled() const;
    void   setOutputLimits(double min, double max);
    void   setSampleTime(unsigned long sampleTime);
    void   setPositionThreshold(float threshold);
    float  getPositionThreshold() const;
    bool   isAtTarget() const;

private:
    double           setpoint;           // Target position
    unsigned long    sampleTime;         // Sample time in milliseconds
    bool             enabled;            // Controller state
    float            positionThreshold;  // Position threshold in degrees
    unsigned long    lastUpdateTime;     // Last update timestamp
    MotorController* motor;              // Pointer to motor controller
    MAE3Encoder*     encoder;            // Pointer to encoder
};

#endif  // PID_CONTROLLER_H