#include "PIDController.h"

PIDController::PIDController(MotorController* motor, MAE3Encoder* encoder, PIDConfig config, unsigned long sampleTime)
    : motor(motor),
      encoder(encoder),
      input(0.0),
      output(0.0),
      setpoint(0.0),
      sampleTime(sampleTime),
      enabled(false),
      positionThreshold(0.3f),  // Default position threshold in degrees
      lastUpdateTime(0)
{
    // Create PID instance with default parameters
    pid = new PID(&input, &output, &setpoint, config.Kp, config.Ki, config.Kd, DIRECT);
}

void PIDController::begin()
{
    if (pid)
    {
        pid->SetMode(AUTOMATIC);
        pid->SetSampleTime(sampleTime);
        pid->SetOutputLimits(-100.0, 100.0);  // Set default output limits (-100 to 100 for speed control)
        enabled = true;
    }
}

bool PIDController::update()
{
    if (!enabled || !pid || !motor || !encoder)
    {
        return false;
    }

    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime >= sampleTime)
    {
        // Update input from encoder
        input = encoder->getPositionDegrees();

        // Check if we're already at target
        if (isAtTarget())
        {
            motor->stop();
            lastUpdateTime = currentTime;
            return true;
        }

        // Compute PID
        bool computed = pid->Compute();

        if (computed)
        {
            // Apply output to motor
            if (output > 0)
            {
                motor->moveForward();
            }
            else if (output < 0)
            {
                motor->moveReverse();
            }
            else
            {
                motor->stop();
            }

            lastUpdateTime = currentTime;
            return true;
        }
    }
    return false;
}

void PIDController::setTarget(double target)
{
    setpoint = target;
}

double PIDController::getTarget() const
{
    return setpoint;
}

void PIDController::setGains(double Kp, double Ki, double Kd)
{
    if (pid)
    {
        pid->SetTunings(Kp, Ki, Kd);
    }
}

void PIDController::enable()
{
    if (pid)
    {
        pid->SetMode(AUTOMATIC);
        enabled = true;
    }
}

void PIDController::disable()
{
    if (pid)
    {
        pid->SetMode(MANUAL);
        enabled = false;
        motor->stop();
    }
}

bool PIDController::isEnabled() const
{
    return enabled;
}

void PIDController::setOutputLimits(double min, double max)
{
    if (pid)
    {
        pid->SetOutputLimits(min, max);
    }
}

void PIDController::setSampleTime(unsigned long sampleTime)
{
    this->sampleTime = sampleTime;
    if (pid)
    {
        pid->SetSampleTime(sampleTime);
    }
}

void PIDController::setPositionThreshold(float threshold)
{
    positionThreshold = threshold;
}

float PIDController::getPositionThreshold() const
{
    return positionThreshold;
}

bool PIDController::isAtTarget() const
{
    if (!encoder)
    {
        return false;
    }

    float currentPosition = encoder->getPositionDegrees();
    float positionError   = abs(currentPosition - setpoint);

    // Handle wrap-around at 0/360 degrees
    if (positionError > 180.0f)
    {
        positionError = 360.0f - positionError;
    }

    return positionError <= positionThreshold;
}