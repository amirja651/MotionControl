#include "MotorController.h"
#include <ArduinoLog.h>
#include "Config/Motor_Specs.h"
#include "Config/SPI.h"
#include "Config/TMC5160T_Driver.h"

MotorController::MotorController(String name, DriverConfig dc)
    : driver(dc.csPin, 0.075, dc.link_index),

      csPin(dc.csPin),
      stepPin(dc.stepPin),
      dirPin(dc.dirPin),
      enPin(dc.enPin),

      runCurrent(200),    // Default 1000mA
      holdCurrent(100),   // Default 500mA
      speed(500),         // Default 1000 steps/sec
      acceleration(500),  // Default 1000 steps/sec²
      maxSpeed(500),
      maxAcceleration(1000),
      maxDeceleration(1000),

      instanceName(name),
      isMoving(false),

      motorType(dc.motorType)
{
}

void MotorController::begin()
{
    // Setup pins
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);

    pinMode(stepPin, OUTPUT);
    digitalWrite(stepPin, LOW);

    pinMode(dirPin, OUTPUT);
    digitalWrite(dirPin, LOW);

    pinMode(enPin, OUTPUT);
    disable();

    driver.begin();

    configureDriver();

    if (motorType == MotorType::ROTATIONAL)
    {
        optimizeForPancake();
    }
    else
    {
        // optimizeFor11HS13_1004H();
        optimize2();
    }
}

void MotorController::optimizeForPancake()
{
    // Set optimal parameters for pancake motor
    driver.intpol(true);     // Enable microstep interpolation
    driver.sfilt(true);      // Enable StallGuard filter
    driver.sgt(10);          // Set StallGuard threshold
    driver.TCOOLTHRS(1000);  // Set CoolStep threshold

    // Configure for high precision
    driver.microsteps(16);  // Maximum microstepping for smooth motion
    driver.intpol(true);    // Enable microstep interpolation

    // Optimize current control
    driver.ihold(holdCurrent * 32 / 32);  // Set hold current
    driver.irun(runCurrent * 32 / 32);    // Set run current

    driver.iholddelay(6);  // Set hold current delay

    // Configure motion control
    driver.RAMPMODE(0);  // Positioning mode for precise control
    driver.VMAX(500);    // Set maximum speed
    driver.a1(500);      // Set maximum acceleration
    driver.d1(500);      // Set maximum deceleration
}

void MotorController::optimize2()
{
    // Set optimal parameters for pancake motor
    driver.intpol(true);     // Enable microstep interpolation
    driver.sfilt(true);      // Enable StallGuard filter
    driver.sgt(10);          // Set StallGuard threshold
    driver.TCOOLTHRS(1000);  // Set CoolStep threshold

    // Configure for high precision
    driver.microsteps(16);  // Maximum microstepping for smooth motion
    driver.intpol(true);    // Enable microstep interpolation

    // Optimize current control
    driver.ihold(holdCurrent * 32 / 32);  // Set hold current
    driver.irun(runCurrent * 32 / 32);    // Set run current

    driver.iholddelay(6);  // Set hold current delay

    // Configure motion control
    driver.RAMPMODE(0);  // Positioning mode for precise control
    driver.VMAX(500);    // Set maximum speed
    driver.a1(500);      // Set maximum acceleration
    driver.d1(500);      // Set maximum deceleration
}

void MotorController::optimizeFor11HS13_1004H()
{
    // First, ensure the driver is enabled
    enable();

    // Configure current settings with proper scaling
    driver.rms_current(1000);  // 1.0A in mA
    driver.ihold(16);          // 0.5A (50% of run current)
    driver.irun(32);           // 1.0A (full current)
    driver.iholddelay(6);      // Set hold current delay

    // Verify current settings
    uint32_t irun  = driver.irun();
    uint32_t ihold = driver.ihold();
    Log.noticeln("Current Settings Verification:");
    Log.noticeln("  IRUN: %d (should be 32)", irun);
    Log.noticeln("  IHOLD: %d (should be 16)", ihold);

    // Configure microstepping and interpolation
    driver.microsteps(16);  // Use 16 microsteps for smooth motion
    driver.intpol(true);    // Enable microstep interpolation

    // Configure StallGuard and CoolStep
    driver.sfilt(true);      // Enable StallGuard filter
    driver.sgt(10);          // Set StallGuard threshold
    driver.TCOOLTHRS(1000);  // Set CoolStep threshold

    // Configure stealthChop for quiet operation
    driver.TPWMTHRS(500);  // Switch to SpreadCycle above 500 steps/sec
    driver.pwm_autoscale(true);
    driver.pwm_autograd(true);
    driver.pwm_ofs(36);
    driver.pwm_grad(14);
    driver.pwm_freq(1);

    // Configure chopper for optimal performance
    driver.toff(3);              // Set turn-off time
    driver.blank_time(24);       // Set blank time
    driver.hysteresis_start(5);  // Set hysteresis start
    driver.hysteresis_end(3);    // Set hysteresis end

    // Configure motion control
    driver.RAMPMODE(0);                                                   // Positioning mode for precise control
    driver.VMAX(CONFIG::Motor11HS13_1004H::Operation::MAX_SPEED);         // Set maximum speed
    driver.AMAX(CONFIG::Motor11HS13_1004H::Operation::MAX_ACCELERATION);  // Set maximum acceleration
    driver.DMAX(CONFIG::Motor11HS13_1004H::Operation::MAX_DECELERATION);  // Set maximum deceleration
    driver.a1(CONFIG::Motor11HS13_1004H::Operation::ACCELERATION);        // Set acceleration
    driver.v1(CONFIG::Motor11HS13_1004H::Operation::SPEED);               // Set speed
    driver.d1(CONFIG::Motor11HS13_1004H::Operation::MAX_DECELERATION);    // Set deceleration
    driver.VSTART(0);                                                     // Set start velocity
    driver.VSTOP(10);                                                     // Set stop velocity

    // Configure GCONF register for optimal performance
    uint32_t gconf = 0;
    gconf |= (1 << 0);  // Enable internal RSense
    gconf |= (1 << 2);  // Enable stealthChop
    gconf |= (1 << 3);  // Enable microstep interpolation
    gconf |= (1 << 4);  // Enable double edge step
    gconf |= (1 << 6);  // Enable multistep filtering
    driver.GCONF(gconf);

    // Set motion parameters with verification
    driver.VMAX(1000);  // Set a reasonable speed
    driver.AMAX(1000);  // Set a reasonable acceleration
    driver.DMAX(1000);  // Set a reasonable deceleration

    // Verify motion parameters
    uint32_t vmax = driver.VMAX();
    uint32_t amax = driver.AMAX();
    Log.noticeln("Motion Parameters Verification:");
    Log.noticeln("  VMAX: %d steps/sec", vmax);
    Log.noticeln("  AMAX: %d steps/sec²", amax);

    // Check driver status
    uint32_t status = driver.DRV_STATUS();
    Log.noticeln("Driver Status: 0x%08X", status);

    // Check specific status bits
    if (status & (1 << 13))
        Log.errorln("Motor stalled");
    if (status & (1 << 12))
        Log.errorln("Motor overtemperature");
    if (status & (1 << 11))
        Log.errorln("Motor short to ground");
    if (status & (1 << 10))
        Log.errorln("Motor open load");

    Log.noticeln("Motor Configuration:");
    Log.noticeln("  Run Current: %d mA", driver.irun() * 31.25);
    Log.noticeln("  Hold Current: %d mA", driver.ihold() * 31.25);
    Log.noticeln("  Speed: %d steps/sec", driver.VMAX());
    Log.noticeln("  Acceleration: %d steps/sec²", driver.AMAX());
}

void MotorController::moveForward()
{
    isMoving = true;
    digitalWrite(dirPin, HIGH);
    enable();
}

void MotorController::moveReverse()
{
    isMoving = true;
    digitalWrite(dirPin, LOW);
    enable();
}

void MotorController::stop()
{
    // Set target velocity to zero
    driver.VMAX(0);

    // Wait for standstill
    uint32_t       status;
    const uint32_t timeout = 1000;  // 1000 ms
    uint32_t       elapsed = 0;
    do
    {
        status = driver.DRV_STATUS();
        vTaskDelay(1);
        elapsed++;
        if (elapsed > timeout)
        {
            // Timeout handling: maybe log or break
            break;
        }
    } while (!(status & (1 << 31)));

    driver.ihold(holdCurrent);  // Reduce to hold current
    isMoving = false;
    disable();
}

void MotorController::update() {}

void MotorController::toggleStealthChop()
{
    uint32_t currentThreshold = driver.TPWMTHRS();
    if (currentThreshold == 0)
    {
        // Currently in StealthChop mode, switch to SpreadCycle
        driver.TPWMTHRS(500);  // Switch to SpreadCycle above 500 steps/sec
    }
    else
    {
        // Currently in SpreadCycle mode, switch to StealthChop
        driver.TPWMTHRS(0);  // Enable StealthChop mode
    }
}

void MotorController::setStealthChopMode(bool enable)
{
    if (enable)
    {
        // Enable StealthChop mode
        driver.TPWMTHRS(0);  // Enable StealthChop for all velocities

        // Configure PWM for StealthChop
        driver.pwm_autoscale(true);  // Enable automatic current scaling
        driver.pwm_autograd(true);   // Enable automatic gradient adaptation
        driver.pwm_ofs(36);          // Default PWM offset
        driver.pwm_grad(14);         // Default PWM gradient
        driver.pwm_freq(1);          // 1 = 23.4kHz PWM frequency

        // Configure chopper for StealthChop
        driver.toff(3);              // Minimum time for slow decay phase
        driver.hysteresis_start(1);  // Hysteresis start value
        driver.hysteresis_end(2);    // Hysteresis end value
        driver.blank_time(24);       // Blanking time
    }
    else
    {
        // Switch to SpreadCycle mode
        driver.TPWMTHRS(0xFFFFF);  // Disable StealthChop (very high threshold)

        // Configure for SpreadCycle
        driver.toff(5);              // Standard time for slow decay phase
        driver.hysteresis_start(4);  // Standard hysteresis start
        driver.hysteresis_end(1);    // Standard hysteresis end
        driver.blank_time(24);       // Standard blanking time
    }
}

bool MotorController::testCommunication()
{
    if (driver.version() == 0xFF || driver.version() == 0)
    {
        return false;
    }

    if (0)
    {
        digitalWrite(csPin, LOW);

        uint32_t gconf  = driver.GCONF();
        uint32_t status = driver.DRV_STATUS();

        if (gconf == 0xFFFFFFFF || status == 0xFFFFFFFF)
        {
            return false;
        }
    }

    return true;
}

// Private Methods
void MotorController::configureDriver()
{
    // Configure GCONF register for optimal performance
    uint32_t gconf = 0;
    gconf |= (1 << 0);  // Enable internal RSense
    gconf |= (1 << 2);  // Enable stealthChop
    gconf |= (1 << 3);  // Enable microstep interpolation
    gconf |= (1 << 4);  // Enable double edge step
    gconf |= (1 << 6);  // Enable multistep filtering
    driver.GCONF(gconf);

    // Set current control parameters
    driver.rms_current(runCurrent);  // Set motor RMS current
    driver.ihold(holdCurrent);       // Set hold current
    driver.irun(runCurrent);         // Set run current
    driver.iholddelay(6);            // Set hold delay
    driver.TPOWERDOWN(10);           // Set power down time

    // Configure microstepping
    driver.microsteps(16);  // Set microsteps
    driver.intpol(true);    // Set microstep interpolation

    // Configure CoolStep
    driver.TCOOLTHRS(1000);  // Set CoolStep threshold
    driver.sgt(10);          // Set StallGuard threshold
    driver.sfilt(true);      // Set StallGuard filter
    driver.sgt(10);          // Set StallGuard threshold

    // Configure stealthChop
    driver.TPWMTHRS(0);          // Enable stealthChop by default
    driver.pwm_autoscale(true);  // Enable PWM autoscale
    driver.pwm_autograd(true);   // Enable PWM autograd
    driver.pwm_ofs(36);          // Set PWM offset
    driver.pwm_grad(14);         // Set PWM gradient
    driver.pwm_freq(1);          // Set PWM frequency

    // Configure spreadCycle
    driver.en_pwm_mode(1);       // 0 for spread cycle, 1 for stealthChop
    driver.toff(3);              // Set turn-off time
    driver.blank_time(24);       // Set blank time
    driver.hysteresis_start(5);  // Set hysteresis start
    driver.hysteresis_end(3);    // Set hysteresis end

    // Configure motion control
    driver.RAMPMODE(0);            // Set ramp mode
    driver.VMAX(maxSpeed);         // Set maximum speed
    driver.AMAX(maxAcceleration);  // Set maximum acceleration
    driver.DMAX(maxDeceleration);  // Set maximum deceleration
    driver.a1(maxAcceleration);    // Set minimum acceleration
    driver.v1(maxSpeed / 2);       // Set minimum speed
    driver.d1(maxDeceleration);    // Set minimum deceleration
    driver.VSTART(0);              // Set start velocity
    driver.VSTOP(10);              // Set stop velocity
}

void MotorController::step()
{
    if (isMoving)
    {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(160);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(160);
    }
}

void MotorController::enable()
{
    digitalWrite(enPin, LOW);
}

void MotorController::disable()
{
    digitalWrite(enPin, HIGH);
}

bool MotorController::isRotational()
{
    return motorType == MotorType::ROTATIONAL;
}
