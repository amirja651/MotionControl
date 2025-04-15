#include "PIDController.h"

PIDController::PIDController(MotorController* motor, MAE3Encoder* encoder, double Kp, double Ki, double Kd,
                             unsigned long sampleTime)
    : motor(motor),
      encoder(encoder),
      input(0.0),
      output(0.0),
      setpoint(0.0),
      Kp(Kp),
      Ki(Ki),
      Kd(Kd),
      sampleTime(sampleTime),
      enabled(false),
      lastUpdateTime(0)
{
    // Create PID instance with default parameters
    pid = new PID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
}

void PIDController::begin()
{
    if (pid)
    {
        pid->SetMode(AUTOMATIC);
        pid->SetSampleTime(sampleTime);
        // Set default output limits (-100 to 100 for speed control)
        pid->SetOutputLimits(-100.0, 100.0);
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
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

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