#include "MotorController.h"
#include <ArduinoLog.h>
#include "Config/SPI.h"
#include "Config/System_Config.h"
#include "Config/TMC5160T_Driver.h"
// driver.TCOOLTHRS(threshold);

// Constructor initializes motor driver and sets default parameters
MotorController::MotorController(String name, DriverConfig dc)
    : driver(dc.csPin, CONFIG::SPI::MOSI, CONFIG::SPI::MISO, CONFIG::SPI::SCK),
      isMoving(false),
      direction(true),
      stepDelay(STEP_DELAY),
      lastStepTime(0),
      stepCounter(0),
      csPin(dc.csPin),
      stepPin(dc.stepPin),
      dirPin(dc.dirPin),
      enPin(dc.enPin),
      runCurrent(CONFIG::MotorSpecs::Operation::RUN_CURRENT),     // Default 1000mA
      holdCurrent(CONFIG::MotorSpecs::Operation::HOLD_CURRENT),   // Default 500mA
      speed(CONFIG::MotorSpecs::Operation::SPEED),                // Default 1000 steps/sec
      acceleration(CONFIG::MotorSpecs::Operation::ACCELERATION),  // Default 1000 steps/sec²
      lastTempPrintTime(0),
      lastTemperature(0),
      instanceName(name),
      diagnosticsEnabled(false),
      coolStepThreshold(CONFIG::SYSTEM::TCOOLTHRS),
      stallGuardThreshold(CONFIG::SYSTEM::SGTHRS),
      stallGuardFilter(true),
      spreadCycleEnabled(false),
      microstepInterpolation(true),
      currentScaling(CONFIG::SYSTEM::CURRENT_SCALING),
      currentHoldDelay(CONFIG::SYSTEM::IHOLDDELAY),
      currentRunDelay(CONFIG::SYSTEM::IRUNDELAY),
      rampMode(0),  // Default to positioning mode
      maxSpeed(CONFIG::MotorSpecs::Operation::SPEED),
      maxAcceleration(CONFIG::MotorSpecs::Operation::ACCELERATION),
      maxDeceleration(CONFIG::MotorSpecs::Operation::ACCELERATION)
{
}

// Initialize motor controller and driver
void MotorController::begin()
{
    setupPins();
    disableSPI();
    resetDriverState();
    configureDriver();
    optimizeForPancake();
}

// Configure GPIO pins for motor control
void MotorController::setupPins()
{
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);
    pinMode(csPin, OUTPUT);
    delay(5);
}

// Deactivates the SPI device by setting CS pin high
void MotorController::disableSPI()
{
    digitalWrite(csPin, HIGH);
    delay(5);
}

// Activates the SPI device by setting CS pin low
void MotorController::enableSPI()
{
    digitalWrite(csPin, LOW);
    delay(5);
}

void MotorController::resetDriverState()
{
    disableDriver();
    enableDriver();
}

// Configure TMC5160 driver parameters for medical-grade precision
void MotorController::configureDriver()
{
    driver.begin();
    delay(5);

    // Configure GCONF register for optimal performance
    uint32_t gconf = 0;
    gconf |= (1 << 0);  // Enable internal RSense
    gconf |= (1 << 2);  // Enable stealthChop
    gconf |= (1 << 3);  // Enable microstep interpolation
    gconf |= (1 << 4);  // Enable double edge step
    gconf |= (1 << 6);  // Enable multistep filtering
    driver.GCONF(gconf);
    delay(5);

    // Set current control parameters
    driver.rms_current(runCurrent);
    driver.ihold(holdCurrent);
    driver.irun(runCurrent);
    driver.iholddelay(currentHoldDelay);
    driver.TPOWERDOWN(CONFIG::SYSTEM::TPOWERDOWN);

    // Configure microstepping
    driver.microsteps(CONFIG::SYSTEM::MICROSTEPS);
    driver.intpol(microstepInterpolation);

    // Configure CoolStep
    driver.TCOOLTHRS(coolStepThreshold);
    driver.sgt(stallGuardThreshold);
    driver.sfilt(stallGuardFilter);
    driver.sgt(stallGuardThreshold);

    // Configure stealthChop
    driver.TPWMTHRS(0);  // Enable stealthChop by default
    driver.pwm_autoscale(true);
    driver.pwm_autograd(true);
    driver.pwm_ofs(CONFIG::SYSTEM::PWM_OFS);
    driver.pwm_grad(CONFIG::SYSTEM::PWM_GRAD);
    driver.pwm_freq(CONFIG::SYSTEM::PWM_FREQ);

    // Configure spreadCycle
    driver.en_pwm_mode(!spreadCycleEnabled);  // 0 for spread cycle, 1 for stealthChop
    driver.toff(CONFIG::SYSTEM::TOFF);
    driver.blank_time(CONFIG::SYSTEM::BLANK_TIME);
    driver.hysteresis_start(CONFIG::SYSTEM::HSTRT);
    driver.hysteresis_end(CONFIG::SYSTEM::HEND);

    // Configure motion control
    driver.RAMPMODE(rampMode);
    driver.VMAX(maxSpeed);
    driver.AMAX(maxAcceleration);
    driver.DMAX(maxDeceleration);
    driver.a1(maxAcceleration);
    driver.v1(maxSpeed / 2);
    driver.d1(maxDeceleration);
    driver.VSTART(0);
    driver.VSTOP(10);

    enableDriver();
    delay(5);
}

// Handle power loss by reinitializing driver
void MotorController::handlePowerLoss()
{
    disableDriver();
    disableSPI();
    enableSPI();

    // Reconfigure driver
    configureDriver();
}

// Check driver status and reinitialize if needed
bool MotorController::checkAndReinitializeDriver()
{
    uint32_t status = driver.DRV_STATUS();

    // Check for communication errors
    if (status == 0 || status == 0xFFFFFFFF)
    {
        Log.errorln(F("%s - Driver communication error detected"), instanceName);
        handlePowerLoss();
        return true;
    }

    // Check for critical error conditions
    if (status & (1 << 25))  // Overtemperature shutdown
    {
        Log.errorln(F("%s - Driver overtemperature shutdown detected"), instanceName);
        handlePowerLoss();
        return true;
    }

    if (status & (1 << 27) || status & (1 << 28))  // Short to ground
    {
        Log.errorln(F("%s - Driver short to ground detected"), instanceName);
        handlePowerLoss();
        return true;
    }

    if (status & (1 << 12) || status & (1 << 13))  // Short to supply
    {
        Log.errorln(F("%s - Driver short to supply detected"), instanceName);
        handlePowerLoss();
        return true;
    }

    // Check for StallGuard stall condition
    if (((status >> 10) & 0x3FF) < 50)  // Very low StallGuard value
    {
        Log.warningln(F("%s - Driver stall condition detected"), instanceName);
        handlePowerLoss();
        return true;
    }

    return false;
}

void MotorController::setMovementDirection(bool forward)
{
    isMoving  = true;
    direction = forward;
    digitalWrite(dirPin, direction ? HIGH : LOW);
    delay(5);
    // Log.noticeln(F("%s - Direction %s"), forward ? F("Forward") : F("Reverse"), instanceName);
}

// Start motor movement in forward direction
void MotorController::moveForward()
{
    if (!testCommunication() || !diagnoseTMC5160())
    {
        Log.errorln(F("%s - Motor will not move."), instanceName);
        return;
    }
    setMovementDirection(true);
}

// Start motor movement in reverse direction
void MotorController::moveReverse()
{
    // Check driver status and communication
    if (!testCommunication() || !diagnoseTMC5160())
    {
        Log.errorln(F("%s - Motor will not move due to driver error."), instanceName);
        return;
    }

    // Configure motion parameters
    driver.RAMPMODE(1);         // Velocity mode
    driver.VMAX(speed);         // Set target velocity
    driver.AMAX(acceleration);  // Set acceleration
    driver.DMAX(acceleration);  // Set deceleration

    // Set direction and start movement
    setMovementDirection(false);

    // Enable driver and start motion
    enableDriver();
    driver.VSTART(0);  // Start from zero velocity

    // Log.noticeln(F("%s - Moving reverse at %d steps/sec, acceleration %d steps/sec²"), instanceName, speed,
    //              acceleration);
}

// Stop motor movement
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

    Log.noticeln(F("%s - Motor stopped"), instanceName);
}

// Execute a single step
void MotorController::step()
{
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPin, LOW);
    lastStepTime = micros();

    if (++stepCounter >= STATUS_PRINT_INTERVAL)
    {
        stepCounter = 0;
    }
}

// Main update loop for motor control
void MotorController::update()
{
    if (isMoving)
    {
        // Check driver status and reinitialize if needed
        if (checkAndReinitializeDriver())
        {
            Log.errorln(F("%s - Driver reinitialized due to error"), instanceName);
            return;
        }

        // Get current motion status
        uint32_t status = driver.DRV_STATUS();

        // Check for motion completion
        if (status & (1 << 29))  // Position reached
        {
            isMoving = false;
            Log.noticeln(F("%s - Target position reached"), instanceName);
            return;
        }

        // Update diagnostics periodically
        static unsigned long lastDiagnosticTime = 0;
        if (diagnosticsEnabled && millis() - lastDiagnosticTime >= 100)
        {
            updateDiagnostics();
            lastDiagnosticTime = millis();
        }

        // Check load and optimize current periodically
        /*static unsigned long lastLoadCheckTime = 0;
        if (millis() - lastLoadCheckTime >= 50)
        {
            checkLoad();
            optimizeCurrent();
            lastLoadCheckTime = millis();
        }*/

        // Monitor temperature periodically
        static unsigned long lastTempCheckTime = 0;
        if (millis() - lastTempCheckTime >= 200)
        {
            int temp = (status >> 16) & 0xFF;  // Temperature from DRV_STATUS
            if (temp >= 120)                   // Overtemperature pre-warning
            {
                Log.warningln(F("%s - High temperature detected: %d°C"), instanceName, temp);
                uint16_t reducedCurrent = runCurrent * 0.8;
                driver.rms_current(reducedCurrent);
            }
            lastTempCheckTime = millis();
        }

        // Adjust microstepping based on speed periodically
        static unsigned long lastMicrostepCheckTime = 0;
        if (millis() - lastMicrostepCheckTime >= 100)
        {
            adjustMicrostepping();
            lastMicrostepCheckTime = millis();
        }
    }
}

uint32_t MotorController::getDriverStatus()
{
    return driver.DRV_STATUS();
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

uint16_t MotorController::getRunCurrent() const
{
    return runCurrent;
}

uint16_t MotorController::getHoldCurrent() const
{
    return holdCurrent;
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

uint16_t MotorController::getSpeed() const
{
    return speed;
}

uint16_t MotorController::getAcceleration() const
{
    return acceleration;
}

void MotorController::printStatusRegister(uint32_t status)
{
    Log.noticeln(F("%s - Raw Driver Status: 0x%X"), instanceName, status);
    printErrorFlags(status);
    printStallGuardStatus(status);
    printDriverState(status);
}

void MotorController::printErrorFlags(uint32_t status)
{
    Log.noticeln(F("%s - Overtemperature Pre-warning: %s"), instanceName, (status & 0x00000001) ? F("Yes") : F("No"));
    Log.noticeln(F("%s - Overtemperature Shutdown: %s"), instanceName, (status & 0x00000002) ? F("Yes") : F("No"));
    Log.noticeln(F("%s - Short to Ground A: %s"), instanceName, (status & 0x00000004) ? F("Yes") : F("No"));
    Log.noticeln(F("%s - Short to Ground B: %s"), instanceName, (status & 0x00000008) ? F("Yes") : F("No"));
    Log.noticeln(F("%s - Open Load A: %s"), instanceName, (status & 0x00000010) ? F("Yes") : F("No"));
    Log.noticeln(F("%s - Open Load B: %s"), instanceName, (status & 0x00000020) ? F("Yes") : F("No"));
}

void MotorController::printStallGuardStatus(uint32_t status)
{
    Log.noticeln(F("%s - StallGuard Value: %s"), instanceName, String((status >> 10) & 0x3FF));
    Log.noticeln(F("%s - Stall Detected: %s"), instanceName, (status & 0x01000000) ? F("Yes") : F("No"));
}

void MotorController::printDriverState(uint32_t status)
{
    Log.noticeln(F("%s - Standstill: %s"), instanceName, (status & 0x80000000) ? F("Yes") : F("No"));
    Log.noticeln(F("%s - Velocity Reached: %s"), instanceName, (status & 0x40000000) ? F("Yes") : F("No"));
    Log.noticeln(F("%s - Position Reached: %s"), instanceName, (status & 0x20000000) ? F("Yes") : F("No"));
}

void MotorController::printDriverStatus()
{
    uint32_t status = driver.DRV_STATUS();

    Log.noticeln(F("%s - DRV_STATUS Report:"), instanceName);
    Log.noticeln(F("%s - Standstill: %s"), instanceName, (status & (1 << 31)) ? F("Yes") : F("No"));
    Log.noticeln(F("%s - Open Load B: %s"), instanceName, (status & (1 << 30)) ? F("Yes") : F("No"));
    Log.noticeln(F("%s - Open Load A: %s"), instanceName, (status & (1 << 29)) ? F("Yes") : F("No"));
    Log.noticeln(F("%s - Short to GND B: %s"), instanceName, (status & (1 << 28)) ? F("Yes") : F("No"));
    Log.noticeln(F("%s - Short to GND A: %s"), instanceName, (status & (1 << 27)) ? F("Yes") : F("No"));
    Log.noticeln(F("%s - Overtemperature Pre-warning: %s"), instanceName, (status & (1 << 26)) ? F("Yes") : F("No"));
    Log.noticeln(F("%s - Overtemperature Shutdown: %s"), instanceName, (status & (1 << 25)) ? F("Yes") : F("No"));
    Log.noticeln(F("%s - Stall Detected: %s"), instanceName, (status & (1 << 24)) ? F("Yes") : F("No"));
    Log.noticeln(F("%s - StallGuard Value: %d"), instanceName, (status >> 10) & 0x3FF);

    // Load interpretation based on StallGuard value
    uint16_t sg_value = status & 0x3FF;
    if (sg_value < 100)
    {
        Log.noticeln(F("%s - Load Status: Possible stall condition!"), instanceName);
    }
    else if (sg_value < 500)
    {
        Log.noticeln(F("%s - Load Status: Moderate load"), instanceName);
    }
    else
    {
        Log.noticeln(F("%s - Load Status: Light load"), instanceName);
    }
}

bool MotorController::diagnoseTMC5160()
{
    uint32_t status = driver.DRV_STATUS();

    if (status & (1 << 27))
    {
        Log.errorln(F("%s - Short to GND on Phase A (s2ga)"), instanceName);
        return false;
    }

    if (status & (1 << 28))
    {
        Log.errorln(F("%s - Short to GND on Phase B (s2gb)"), instanceName);
        return false;
    }

    if (status & (1 << 12))
    {
        Log.errorln(F("%s - Short to supply on Phase A (s2vsa)"), instanceName);
        return false;
    }

    if (status & (1 << 13))
    {
        Log.errorln(F("%s - Short to supply on Phase B (s2vsb)"), instanceName);
        return false;
    }

    if (status & (1 << 25))
    {
        Log.errorln(F("%s - Overtemperature shutdown active (ot)"), instanceName);
        return false;
    }

    if (status & (1 << 24))
    {
        Log.warningln(F("%s - Motor stall detected (StallGuard)"), instanceName);
        return false;
    }

    // Log.info(F("%s - All driver diagnostics OK."), instanceName);

    return true;
}

void MotorController::printDriverConfig()
{
    Log.noticeln(F("%s - Driver Configuration:"), instanceName);

    // Current settings
    Log.noticeln(F("%s - Run Current: %d mA"), instanceName, runCurrent);
    Log.noticeln(F("%s - Hold Current: %d mA"), instanceName, holdCurrent);

    // Microstepping configuration
    Log.noticeln(F("%s - Microsteps: %d"), instanceName, driver.microsteps());
    Log.noticeln(F("%s - Microstep Interpolation: %s"), instanceName, driver.intpol() ? F("Enabled") : F("Disabled"));

    // Motion control
    Log.noticeln(F("%s - Speed: %d steps/sec"), instanceName, speed);
    Log.noticeln(F("%s - Acceleration: %d steps/sec²"), instanceName, acceleration);

    // Global configuration
    Log.noticeln(F("%s - GCONF: 0x%X"), instanceName, driver.GCONF());

    // Power management
    Log.noticeln(F("%s - TPOWERDOWN: %d tclk"), instanceName, driver.TPOWERDOWN());
    Log.noticeln(F("%s - TPOWERDOWN: %d ms"), instanceName, (driver.TPOWERDOWN() * 2) / 1000);  // Convert to ms

    // StealthChop configuration
    Log.noticeln(F("%s - TPWMTHRS: %d tclk"), instanceName, driver.TPWMTHRS());
    Log.noticeln(F("%s - TCOOLTHRS: %d tclk"), instanceName, driver.TCOOLTHRS());
    Log.noticeln(F("%s - THIGH: %d tclk"), instanceName, driver.THIGH());

    // StallGuard configuration
    Log.noticeln(F("%s - SGTHRS: %d"), instanceName, driver.sgt());
    Log.noticeln(F("%s - SFILT: %s"), instanceName, driver.sfilt() ? F("Enabled") : F("Disabled"));

    // Chopper configuration
    Log.noticeln(F("%s - TOFF: %d"), instanceName, driver.toff());
    Log.noticeln(F("%s - HSTRT: %d"), instanceName, driver.hysteresis_start());
    Log.noticeln(F("%s - HEND: %d"), instanceName, driver.hysteresis_end());
    Log.noticeln(F("%s - TBL: %d"), instanceName, driver.blank_time());

    // PWM configuration
    Log.noticeln(F("%s - PWM_OFS: %d"), instanceName, driver.pwm_ofs());
    Log.noticeln(F("%s - PWM_GRAD: %d"), instanceName, driver.pwm_grad());
    Log.noticeln(F("%s - PWM_FREQ: %d"), instanceName, driver.pwm_freq());
    Log.noticeln(F("%s - PWM_AUTOSCALE: %s"), instanceName, driver.pwm_autoscale() ? F("Enabled") : F("Disabled"));
    Log.noticeln(F("%s - PWM_AUTOGRAD: %s"), instanceName, driver.pwm_autograd() ? F("Enabled") : F("Disabled"));

    // Direct coil control
    Log.noticeln(F("%s - XDIRECT: 0x%X"), instanceName, driver.XDIRECT());
}

int MotorController::getTemperature()
{
    uint32_t status  = driver.DRV_STATUS();
    int      rawTemp = (status >> 16) & 0xFF;  // Temperature is in bits 16-23
    return rawTemp;                            // Direct temperature reading in °C (1°C steps)
}

void MotorController::printTemperature()
{
    uint32_t status = driver.DRV_STATUS();

    // Temperature is in bits 16-23 of DRV_STATUS
    int temp = (status >> 16) & 0xFF;

    if (temp != lastTemperature)
    {
        // Report temperature with warning levels
        // TMC5160A datasheet:
        // - Overtemperature shutdown at 150°C
        // - Overtemperature pre-warning at 120°C
        if (temp >= 150)  // Overtemperature shutdown threshold
        {
            Log.errorln(F("%s - Temperature: %d°C (SHUTDOWN)"), instanceName, temp);
        }
        else if (temp >= 120)  // Overtemperature pre-warning threshold
        {
            Log.warningln(F("%s - Temperature: %d°C (WARNING)"), instanceName, temp);
        }
        else
        {
            Log.noticeln(F("%s - Temperature: %d°C"), instanceName, temp);
        }

        lastTemperature = temp;
    }
}

// Check for motor stall condition
void MotorController::checkStall()
{
    uint32_t status = driver.DRV_STATUS();

    // Check StallGuard value (bits 10-19) and Stall flag (bit 24)
    uint16_t sg_value       = (status >> 10) & 0x3FF;
    bool     stall_detected = status & 0x01000000;

    if (stall_detected || sg_value < stallGuardThreshold)
    {
        Log.warningln(F("%s - Stall detected! SG Value: %d"), instanceName, sg_value);

        // Reduce current temporarily to prevent damage
        uint16_t originalCurrent = runCurrent;
        driver.rms_current(runCurrent * 0.7);

        // Print detailed stall information
        printStallGuardStatus(status);

        // Wait for recovery
        delay(100);

        // Restore current
        driver.rms_current(originalCurrent);
    }
}

// Toggle between StealthChop and SpreadCycle modes
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

// Performs a basic SPI communication test by sending a test pattern
bool MotorController::testCommunication()
{
    enableSPI();

    uint32_t gconf  = driver.GCONF();
    uint32_t status = driver.DRV_STATUS();

    if (gconf == 0xFFFFFFFF || status == 0xFFFFFFFF)
    {
        return false;
    }

    return true;
}

// Performs a single byte SPI transfer
uint8_t MotorController::transfer(uint8_t data)
{
    return SPI.transfer(data);
}

void MotorController::enableDriver()
{
    digitalWrite(enPin, LOW);
}

void MotorController::disableDriver()
{
    digitalWrite(enPin, HIGH);
}

void MotorController::stepHigh()
{
    digitalWrite(stepPin, HIGH);
}

void MotorController::stepLow()
{
    digitalWrite(stepPin, LOW);
}

void MotorController::dirHigh()
{
    digitalWrite(dirPin, HIGH);
}

void MotorController::dirLow()
{
    digitalWrite(dirPin, LOW);
}

String MotorController::motorName() const
{
    return instanceName;
}

// Optimize for pancake motor
void MotorController::optimizeForPancake()
{
    // Set optimal parameters for pancake motor
    setMicrostepInterpolation(true);
    setStallGuardFilter(true);
    setStallGuardThreshold(CONFIG::SYSTEM::SGTHRS);
    setCoolStepThreshold(CONFIG::SYSTEM::TCOOLTHRS);

    // Configure for high precision
    driver.microsteps(256);  // Maximum microstepping for smooth motion
    driver.intpol(true);     // Enable microstep interpolation

    // Optimize current control
    setCurrentScaling(CONFIG::SYSTEM::CURRENT_SCALING);
    setCurrentHoldDelay(CONFIG::SYSTEM::IHOLDDELAY);
    setCurrentRunDelay(CONFIG::SYSTEM::IRUNDELAY);

    // Configure motion control
    setRampMode(0);  // Positioning mode for precise control
    setMaxSpeed(CONFIG::MotorSpecs::Operation::MAX_SPEED);
    setMaxAcceleration(CONFIG::MotorSpecs::Operation::MAX_ACCELERATION);
    setMaxDeceleration(CONFIG::MotorSpecs::Operation::MAX_DECELERATION);
}

// Advanced motor control methods
void MotorController::setCoolStepThreshold(uint32_t threshold)
{
    coolStepThreshold = threshold;
    driver.TCOOLTHRS(threshold);
}

void MotorController::setStallGuardThreshold(int8_t threshold)
{
    stallGuardThreshold = threshold;
    driver.sgt(threshold);
}

void MotorController::setStallGuardFilter(bool enable)
{
    stallGuardFilter = enable;
    driver.sfilt(enable);
}

void MotorController::setSpreadCycle(bool enable)
{
    spreadCycleEnabled = enable;
    driver.en_pwm_mode(!enable);  // 0 for spread cycle, 1 for stealthChop
}

void MotorController::setMicrostepInterpolation(bool enable)
{
    microstepInterpolation = enable;
    driver.intpol(enable);
}

// Advanced current control
void MotorController::setCurrentScaling(uint8_t scaling)
{
    currentScaling = scaling;
    driver.ihold(holdCurrent * scaling / 32);
    driver.irun(runCurrent * scaling / 32);
}

void MotorController::setCurrentHoldDelay(uint8_t delay)
{
    currentHoldDelay = delay;
    driver.iholddelay(delay);
}

void MotorController::setCurrentRunDelay(uint8_t delay)
{
    currentRunDelay = delay;
    // Run current delay is handled by irun(runCurrent)
}

// Motion control
void MotorController::setRampMode(uint8_t mode)
{
    rampMode = mode;
    driver.RAMPMODE(mode);
}

void MotorController::setMaxSpeed(uint32_t speed)
{
    maxSpeed = speed;
    driver.VMAX(speed);
}

void MotorController::setMaxAcceleration(uint32_t accel)
{
    maxAcceleration = accel;
    driver.a1(accel);
}

void MotorController::setMaxDeceleration(uint32_t decel)
{
    maxDeceleration = decel;
    driver.d1(decel);
}

// Advanced diagnostics
void MotorController::enableDiagnostics()
{
    diagnosticsEnabled = true;
}

void MotorController::disableDiagnostics()
{
    diagnosticsEnabled = false;
}

uint32_t MotorController::getLoadValue()
{
    return driver.sg_result();
}

bool MotorController::isStalled()
{
    return (driver.sg_result() < stallGuardThreshold);
}

// Update diagnostic information
void MotorController::updateDiagnostics()
{
    if (!diagnosticsEnabled)
        return;

    // Get current status
    uint32_t status   = driver.DRV_STATUS();
    uint16_t sg_value = (status >> 10) & 0x3FF;  // StallGuard value
    int      temp     = (status >> 16) & 0xFF;   // Temperature

    // Report load status based on StallGuard value
    String load_status;
    if (sg_value < 100)
    {
        load_status = F("Possible stall condition!");
    }
    else if (sg_value < 500)
    {
        load_status = F("Moderate load");
    }
    else
    {
        load_status = F("Light load");
    }

    // Report temperature status
    String temp_status;
    if (temp >= 150)
    {
        temp_status = F("SHUTDOWN");
    }
    else if (temp >= 120)
    {
        temp_status = F("WARNING");
    }
    else
    {
        temp_status = F("Normal");
    }

    // Log detailed diagnostics
    Log.noticeln(F("%s - Diagnostics:"), instanceName);
    Log.noticeln(F("%s - Load: %d (SG Value) - %s"), instanceName, sg_value, load_status);
    Log.noticeln(F("%s - Temperature: %d°C - %s"), instanceName, temp, temp_status);

    // Check for stall condition
    if (isStalled())
    {
        handleStall();
    }
}

// Handle stall condition
void MotorController::handleStall()
{
    //    String message = F("Stall detected:");
    //  Log.warningln(message, instanceName);

    // Reduce current temporarily
    uint16_t originalCurrent = runCurrent;
    driver.rms_current(runCurrent * 0.7);

    // Wait for recovery
    delay(100);

    // Restore current
    driver.rms_current(originalCurrent);
}

// Optimize current based on load
void MotorController::optimizeCurrent()
{
    uint32_t status   = driver.DRV_STATUS();
    uint16_t sg_value = (status >> 10) & 0x3FF;  // StallGuard value from bits 10-19

    // Optimize current based on StallGuard value
    if (sg_value < 100)  // Possible stall condition
    {
        // Increase current by 30% but not above MAX_RUN_CURRENT
        uint16_t newCurrent = static_cast<uint16_t>(
            std::min(static_cast<double>(runCurrent * 1.3), static_cast<double>(MAX_RUN_CURRENT)));
        driver.rms_current(newCurrent);
        runCurrent = newCurrent;
        Log.noticeln(F("%s - Current increased to %d mA (StallGuard: %d)"), instanceName, newCurrent, sg_value);
    }
    else if (sg_value < 300)  // High load
    {
        // Increase current by 15% but not above MAX_RUN_CURRENT
        uint16_t newCurrent = static_cast<uint16_t>(
            std::min(static_cast<double>(runCurrent * 1.15), static_cast<double>(MAX_RUN_CURRENT)));
        driver.rms_current(newCurrent);
        runCurrent = newCurrent;
        Log.noticeln(F("%s - Current increased to %d mA (StallGuard: %d)"), instanceName, newCurrent, sg_value);
    }
    else if (sg_value > 800)  // Very light load
    {
        // Decrease current by 20% but not below MIN_CURRENT
        uint16_t newCurrent =
            static_cast<uint16_t>(std::max(static_cast<double>(runCurrent * 0.8), static_cast<double>(MIN_CURRENT)));
        driver.rms_current(newCurrent);
        runCurrent = newCurrent;
        Log.noticeln(F("%s - Current decreased to %d mA (StallGuard: %d)"), instanceName, newCurrent, sg_value);
    }
    else if (sg_value > 500)  // Light load
    {
        // Decrease current by 10% but not below MIN_CURRENT
        uint16_t newCurrent =
            static_cast<uint16_t>(std::max(static_cast<double>(runCurrent * 0.9), static_cast<double>(MIN_CURRENT)));
        driver.rms_current(newCurrent);
        runCurrent = newCurrent;
        Log.noticeln(F("%s - Current decreased to %d mA (StallGuard: %d)"), instanceName, newCurrent, sg_value);
    }
}

// Check motor load
void MotorController::checkLoad()
{
    uint32_t load = getLoadValue();
    if (load > CONFIG::SYSTEM::LOAD_WARNING_THRESHOLD)
    {
        Log.warningln(F("%s - High load detected: $d"), instanceName, load);
        optimizeCurrent();
    }
}

// Adjust microstepping based on speed
void MotorController::adjustMicrostepping()
{
    // TMC5160A supports microstepping from 1 to 256
    // Adjust based on speed and precision requirements
    /*if (speed > 20000)  // Very high speed
    {
        driver.microsteps(4);  // Minimum microstepping for high speed
        Log.noticeln(F("%s - Microstepping set to 4 (Very High Speed: %d steps/sec)"), instanceName, speed);
    }
    else if (speed > 10000)  // High speed
    {
        driver.microsteps(8);  // Reduced microstepping for high speed
        Log.noticeln(F("%s - Microstepping set to 8 (High Speed: %d steps/sec)"), instanceName, speed);
    }
    else if (speed > 5000)  // Medium speed
    {
        driver.microsteps(16);  // Balanced microstepping for medium speed
        Log.noticeln(F("%s - Microstepping set to 16 (Medium Speed: %d steps/sec)"), instanceName, speed);
    }
    else if (speed > 1000)  // Low speed
    {
        driver.microsteps(32);  // Higher microstepping for low speed
        Log.noticeln(F("%s - Microstepping set to 32 (Low Speed: %d steps/sec)"), instanceName, speed);
    }
    else  // Very low speed
    {
        driver.microsteps(64);  // Maximum microstepping for precision
        Log.noticeln(F("%s - Microstepping set to 64 (Very Low Speed: %d steps/sec)"), instanceName, speed);
    }*/

    driver.microsteps(16);
    // Enable microstep interpolation for smoother motion
    driver.intpol(true);
}