#include "PIDController.h"

PIDController::PIDController(PIDConfig config, unsigned long sampleTime) : input(0.0), output(0.0), setpoint(0.0)
{
    pid = new PID(&input, &output, &setpoint, config.Kp, config.Ki, config.Kd, DIRECT);
}

void PIDController::begin()
{
    pid->SetMode(AUTOMATIC);
    pid->SetOutputLimits(-100.0, 100.0);  // Set default output limits (-100 to 100 for speed control)
}

void PIDController::setTarget(double target)
{
    setpoint = target;
}

double PIDController::getTarget() const
{
    return setpoint;
}

double PIDController::getPositionError(double currentPosition) const
{
    double positionError = abs(currentPosition - setpoint);

    // Handle wrap-around at 0/360 degrees
    if (positionError > 180.0f)
    {
        positionError = 360.0f - positionError;
    }

    return positionError;
}

void PIDController::setInput(float input)
{
    this->input = input;
}

double PIDController::getOutput() const
{
    return this->output;
}
