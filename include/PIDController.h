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
    /**
     * @brief Constructor for PIDController
     * @param motor Pointer to the motor controller instance
     * @param encoder Pointer to the encoder instance
     * @param config PID configuration
     * @param sampleTime Sample time in milliseconds
     */
    PIDController(MotorController* motor, MAE3Encoder* encoder, PIDConfig config, unsigned long sampleTime = 100);

    /**
     * @brief Initialize the PID controller
     */
    void begin();

    /**
     * @brief Update the PID controller
     * @return true if the controller updated successfully
     */
    bool update();

    /**
     * @brief Set the target position in degrees
     * @param target Target position in degrees
     */
    void setTarget(double target);

    /**
     * @brief Get the current target position
     * @return Current target position in degrees
     */
    double getTarget() const;

    /**
     * @brief Set PID gains
     * @param Kp Proportional gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     */
    void setGains(double Kp, double Ki, double Kd);

    /**
     * @brief Enable the PID controller
     */
    void enable();

    /**
     * @brief Disable the PID controller
     */
    void disable();

    /**
     * @brief Check if the PID controller is enabled
     * @return true if enabled, false otherwise
     */
    bool isEnabled() const;

    /**
     * @brief Set the output limits
     * @param min Minimum output value
     * @param max Maximum output value
     */
    void setOutputLimits(double min, double max);

    /**
     * @brief Set the sample time in milliseconds
     * @param sampleTime Sample time in milliseconds
     */
    void setSampleTime(unsigned long sampleTime);

    /**
     * @brief Set the position threshold in degrees
     * @param threshold Position threshold in degrees
     */
    void setPositionThreshold(float threshold);

    /**
     * @brief Get the current position threshold
     * @return Current position threshold in degrees
     */
    float getPositionThreshold() const;

    /**
     * @brief Check if the current position is within threshold of the target
     * @return true if within threshold, false otherwise
     */
    bool isAtTarget() const;

private:
    MotorController* motor;    // Pointer to motor controller
    MAE3Encoder*     encoder;  // Pointer to encoder
    PID*             pid;      // PID controller instance

    // PID variables
    double        input;              // Process variable (current position)
    double        output;             // Controller output
    double        setpoint;           // Target position
    unsigned long sampleTime;         // Sample time in milliseconds
    bool          enabled;            // Controller state
    float         positionThreshold;  // Position threshold in degrees

    // Timing variables
    unsigned long lastUpdateTime;  // Last update timestamp
};

#endif  // PID_CONTROLLER_H