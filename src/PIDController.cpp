#include "PIDController.h"

PIDController::PIDController(PIDConfig config) : input(0.0), output(0.0), setpoint(0.0)
{
    pid = new PID(&input, &output, &setpoint, config.Kp, config.Ki, config.Kd, DIRECT);
}

void PIDController::begin()
{
    pid->SetMode(AUTOMATIC);
}

void PIDController::setTarget(double target)
{
    setpoint = target;
}

double PIDController::getTarget() const
{
    return setpoint;
}

double PIDController::getPositionError(double currentPosition, bool isRotational) const
{
    double error = setpoint - currentPosition;

    if (isRotational)
    {
        error = fmod(error + 180.0, 360.0) - 180.0;
        if (error < -180.0)
            error += 360.0;
        if (error > 180.0)
            error -= 360.0;
    }

    return error;
}

void PIDController::setInput(float input)
{
    this->input = input;
}

double PIDController::getOutput() const
{
    return this->output;
}

void PIDController::setOutputLimits(double min, double max)
{
    pid->SetOutputLimits(min, max);
}
