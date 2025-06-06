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
    double positionError = 0.0;

    if (isRotational)
    {
        positionError = fabs(currentPosition - setpoint);

        // Handle wrap-around at 0/360 degrees
        if (positionError > 180.0f && isRotational)
        {
            positionError = 360.0f - positionError;
        }
    }
    else
    {
        positionError = currentPosition - setpoint;
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

void PIDController::setOutputLimits(double min, double max)
{
    pid->SetOutputLimits(min, max);
}
