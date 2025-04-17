#include "MotorController.h"
#include <ArduinoLog.h>
#include "Config/SPI.h"
#include "Config/System_Config.h"
#include "Config/TMC5160T_Driver.h"

MotorController::MotorController(String name, DriverConfig dc)
    : driver(dc.csPin, 0.075),
      csPin(dc.csPin),
      stepPin(dc.stepPin),
      dirPin(dc.dirPin),
      enPin(dc.enPin),

      isMoving(false),
      instanceName(name),

      runCurrent(CONFIG::MotorSpecs::Operation::RUN_CURRENT),    // Default 1000mA
      holdCurrent(CONFIG::MotorSpecs::Operation::HOLD_CURRENT),  // Default 500mA

      speed(CONFIG::MotorSpecs::Operation::SPEED),                // Default 1000 steps/sec
      acceleration(CONFIG::MotorSpecs::Operation::ACCELERATION),  // Default 1000 steps/sec²

      coolStepThreshold(CONFIG::SYSTEM::TCOOLTHRS),
      stallGuardThreshold(CONFIG::SYSTEM::SGTHRS),
      stallGuardFilter(true),
      spreadCycleEnabled(false),
      microstepInterpolation(true),

      currentHoldDelay(CONFIG::SYSTEM::IHOLDDELAY),

      rampMode(0),  // Default to positioning mode
      maxSpeed(CONFIG::MotorSpecs::Operation::SPEED),
      maxAcceleration(CONFIG::MotorSpecs::Operation::ACCELERATION),
      maxDeceleration(CONFIG::MotorSpecs::Operation::ACCELERATION)
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
    digitalWrite(enPin, HIGH);

    configureDriver();

    digitalWrite(enPin, LOW);

    optimizeForPancake();
}

void MotorController::optimizeForPancake()
{
    // Set optimal parameters for pancake motor
    driver.intpol(true);                          // Enable microstep interpolation
    driver.sfilt(true);                           // Enable StallGuard filter
    driver.sgt(CONFIG::SYSTEM::SGTHRS);           // Set StallGuard threshold
    driver.TCOOLTHRS(CONFIG::SYSTEM::TCOOLTHRS);  // Set CoolStep threshold

    // Configure for high precision
    driver.microsteps(16);  // Maximum microstepping for smooth motion
    driver.intpol(true);    // Enable microstep interpolation

    // Optimize current control
    driver.ihold(holdCurrent * CONFIG::SYSTEM::CURRENT_SCALING / 32);  // Set hold current
    driver.irun(runCurrent * CONFIG::SYSTEM::CURRENT_SCALING / 32);    // Set run current

    driver.iholddelay(CONFIG::SYSTEM::IHOLDDELAY);  // Set hold current delay

    // Configure motion control
    driver.RAMPMODE(0);                                          // Positioning mode for precise control
    driver.VMAX(CONFIG::MotorSpecs::Operation::MAX_SPEED);       // Set maximum speed
    driver.a1(CONFIG::MotorSpecs::Operation::MAX_ACCELERATION);  // Set maximum acceleration
    driver.d1(CONFIG::MotorSpecs::Operation::MAX_DECELERATION);  // Set maximum deceleration
}

void MotorController::moveForward()
{
    isMoving = true;
    digitalWrite(dirPin, HIGH);
}

void MotorController::moveReverse()
{
    isMoving = true;
    digitalWrite(dirPin, LOW);

    //              acceleration);
}

void MotorController::stop()
{
    // Set target velocity to zero
    driver.VMAX(0);

    // Wait for standstill
    uint32_t status;
    do
    {
        status = driver.DRV_STATUS();
        delay(1);
    } while (!((status & (1 << 31))));  // Wait for standstill bit

    // Reduce to hold current
    driver.ihold(holdCurrent);

    // Update state
    isMoving = false;

    // Log.noticeln(F("%s - Motor stopped"), instanceName);
}

void MotorController::update()
{
    if (isMoving)
    {
        step();
    }
}

void MotorController::increaseRunCurrent()
{
    if (runCurrent < MAX_RUN_CURRENT)
    {
        runCurrent += CURRENT_STEP;
        driver.rms_current(runCurrent);
        Log.noticeln(F("%s - Run current increased to: %d mA (Max: 1000mA)"), instanceName, runCurrent);
    }
    else
    {
        Log.warningln(F("%s - Run current at maximum (1000mA)"), instanceName);
    }
}

void MotorController::decreaseRunCurrent()
{
    if (runCurrent > MIN_CURRENT)
    {
        runCurrent -= CURRENT_STEP;
        driver.rms_current(runCurrent);
        Log.noticeln(F("%s - Run current decreased to: %d mA (Min: 100mA)"), instanceName, runCurrent);
    }
    else
    {
        Log.warningln(F("Run current at minimum (100mA)"));
    }
}

void MotorController::increaseHoldCurrent()
{
    if (holdCurrent < MAX_HOLD_CURRENT)
    {
        holdCurrent += CURRENT_STEP;
        driver.ihold(holdCurrent);
        Log.noticeln(F("%s - Hold current increased to: %d mA (Max: 500mA)"), instanceName, holdCurrent);
    }
    else
    {
        Log.warningln(F("Hold current at maximum (500mA)"));
    }
}

void MotorController::decreaseHoldCurrent()
{
    if (holdCurrent > MIN_CURRENT)
    {
        holdCurrent -= CURRENT_STEP;
        driver.ihold(holdCurrent);
        Log.noticeln(F("%s - Hold current decreased to: %d mA (Min: 100mA)"), instanceName, holdCurrent);
    }
    else
    {
        Log.warningln(F("Hold current at minimum (100mA)"));
    }
}

void MotorController::increaseSpeed()
{
    if (speed < MAX_SPEED)
    {
        speed += SPEED_STEP;
        Log.noticeln(F("%s - Speed increased to: %d steps/sec"), instanceName, speed);
    }
    else
    {
        Log.warningln(F("Speed at maximum (10000 steps/sec)"));
    }
}

void MotorController::decreaseSpeed()
{
    if (speed > MIN_SPEED)
    {
        speed -= SPEED_STEP;
        Log.noticeln(F("%s - Speed decreased to: %d steps/sec"), instanceName, speed);
    }
    else
    {
        Log.warningln(F("Speed at minimum (100 steps/sec)"));
    }
}

void MotorController::increaseAcceleration()
{
    if (acceleration < MAX_ACCEL)
    {
        acceleration += ACCEL_STEP;
        driver.AMAX(acceleration);
        Log.noticeln(F("%s - Acceleration increased to: %d steps/sec²"), instanceName, acceleration);
    }
    else
    {
        Log.warningln(F("Acceleration at maximum (10000 steps/sec²)"));
    }
}

void MotorController::decreaseAcceleration()
{
    if (acceleration > MIN_ACCEL)
    {
        acceleration -= ACCEL_STEP;
        driver.AMAX(acceleration);
        Log.noticeln(F("%s - Acceleration decreased to: %d steps/sec²"), instanceName, acceleration);
    }
    else
    {
        Log.warningln(F("Acceleration at minimum (100 steps/sec²)"));
    }
}

void MotorController::toggleStealthChop()
{
    uint32_t currentThreshold = driver.TPWMTHRS();
    if (currentThreshold == 0)
    {
        // Currently in StealthChop mode, switch to SpreadCycle
        driver.TPWMTHRS(500);  // Switch to SpreadCycle above 500 steps/sec
        // Log.info(F("Switched to SpreadCycle mode (more power, more noise)"), LogModule::MOTOR, String(instanceName));
    }
    else
    {
        // Currently in SpreadCycle mode, switch to StealthChop
        driver.TPWMTHRS(0);  // Enable StealthChop mode
        // Log.info(F("Switched to StealthChop mode (silent operation)"), instanceName);
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

        Log.noticeln(F("%s - StealthChop mode enabled"), instanceName);
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

        Log.noticeln(F("%s - SpreadCycle mode enabled"), instanceName);
    }
}

bool MotorController::testCommunication()
{
    digitalWrite(csPin, LOW);
    ;

    uint32_t gconf  = driver.GCONF();
    uint32_t status = driver.DRV_STATUS();

    if (gconf == 0xFFFFFFFF || status == 0xFFFFFFFF)
    {
        return false;
    }

    return true;
}

String MotorController::motorName() const
{
    return instanceName;
}

// Private Methods
void MotorController::configureDriver()
{
    driver.begin();

    if (driver.version() == 0xFF || driver.version() == 0)
    {
        Serial.println("Driver communication error");
        while (true)
            ;
    }

    Serial.print("Driver firmware version: ");
    Serial.println(driver.version());

    if (driver.sd_mode())
    {
        Serial.println("Driver is hardware configured for Step & Dir mode");
        // while (true)
        //     ;
    }
    if (driver.drv_enn())
    {
        Serial.println("Driver is not hardware enabled");
        // while (true)
        //     ;
    }

    // Configure GCONF register for optimal performance
    uint32_t gconf = 0;
    gconf |= (1 << 0);  // Enable internal RSense
    gconf |= (1 << 2);  // Enable stealthChop
    gconf |= (1 << 3);  // Enable microstep interpolation
    gconf |= (1 << 4);  // Enable double edge step
    gconf |= (1 << 6);  // Enable multistep filtering
    driver.GCONF(gconf);

    // Set current control parameters
    driver.rms_current(runCurrent);                 // Set motor RMS current
    driver.ihold(holdCurrent);                      // Set hold current
    driver.irun(runCurrent);                        // Set run current
    driver.iholddelay(currentHoldDelay);            // Set hold delay
    driver.TPOWERDOWN(CONFIG::SYSTEM::TPOWERDOWN);  // Set power down time

    // Configure microstepping
    driver.microsteps(CONFIG::SYSTEM::MICROSTEPS);  // Set microsteps
    driver.intpol(microstepInterpolation);          // Set microstep interpolation

    // Configure CoolStep
    driver.TCOOLTHRS(coolStepThreshold);  // Set CoolStep threshold
    driver.sgt(stallGuardThreshold);      // Set StallGuard threshold
    driver.sfilt(stallGuardFilter);       // Set StallGuard filter
    driver.sgt(stallGuardThreshold);      // Set StallGuard threshold

    // Configure stealthChop
    driver.TPWMTHRS(0);                         // Enable stealthChop by default
    driver.pwm_autoscale(true);                 // Enable PWM autoscale
    driver.pwm_autograd(true);                  // Enable PWM autograd
    driver.pwm_ofs(CONFIG::SYSTEM::PWM_OFS);    // Set PWM offset
    driver.pwm_grad(CONFIG::SYSTEM::PWM_GRAD);  // Set PWM gradient
    driver.pwm_freq(CONFIG::SYSTEM::PWM_FREQ);  // Set PWM frequency

    // Configure spreadCycle
    driver.en_pwm_mode(!spreadCycleEnabled);         // 0 for spread cycle, 1 for stealthChop
    driver.toff(CONFIG::SYSTEM::TOFF);               // Set turn-off time
    driver.blank_time(CONFIG::SYSTEM::BLANK_TIME);   // Set blank time
    driver.hysteresis_start(CONFIG::SYSTEM::HSTRT);  // Set hysteresis start
    driver.hysteresis_end(CONFIG::SYSTEM::HEND);     // Set hysteresis end

    // Configure motion control
    driver.RAMPMODE(rampMode);     // Set ramp mode
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
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(160);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(160);
}
