#include "MotorController.h"
// driver.TCOOLTHRS(threshold);

// Constructor initializes motor driver and sets default parameters
MotorController::MotorController(String name, uint8_t csPin, uint8_t stepPin, uint8_t dirPin, uint8_t enPin,
                                 uint8_t mosiPin, uint8_t misoPin, uint8_t sckPin)
    : driver(csPin, mosiPin, misoPin, sckPin),
      isMoving(false),
      direction(true),
      stepDelay(Config::MotorController::STEP_DELAY),
      lastStepTime(0),
      stepCounter(0),
      csPin(csPin),
      stepPin(stepPin),
      dirPin(dirPin),
      enPin(enPin),
      mosiPin(mosiPin),
      misoPin(misoPin),
      sckPin(sckPin),
      runCurrent(Config::MotorSpecs::Operation::RUN_CURRENT),     // Default 1000mA
      holdCurrent(Config::MotorSpecs::Operation::HOLD_CURRENT),   // Default 500mA
      speed(Config::MotorSpecs::Operation::SPEED),                // Default 1000 steps/sec
      acceleration(Config::MotorSpecs::Operation::ACCELERATION),  // Default 1000 steps/sec²
      lastTempPrintTime(0),
      lastTemperature(0),
      instanceName(name),
      diagnosticsEnabled(false),
      coolStepThreshold(Config::TMC5160T_Driver::TCOOLTHRS),
      stallGuardThreshold(Config::TMC5160T_Driver::SGTHRS),
      stallGuardFilter(true),
      spreadCycleEnabled(false),
      microstepInterpolation(true),
      currentScaling(Config::TMC5160T_Driver::CURRENT_SCALING),
      currentHoldDelay(Config::TMC5160T_Driver::IHOLDDELAY),
      currentRunDelay(Config::TMC5160T_Driver::IRUNDELAY),
      rampMode(0),  // Default to positioning mode
      maxSpeed(Config::MotorSpecs::Operation::SPEED),
      maxAcceleration(Config::MotorSpecs::Operation::ACCELERATION),
      maxDeceleration(Config::MotorSpecs::Operation::ACCELERATION)
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
    driver.TPOWERDOWN(Config::TMC5160T_Driver::TPOWERDOWN);

    // Configure microstepping
    driver.microsteps(Config::TMC5160T_Driver::MICROSTEPS);
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
    driver.pwm_ofs(Config::TMC5160T_Driver::PWM_OFS);
    driver.pwm_grad(Config::TMC5160T_Driver::PWM_GRAD);
    driver.pwm_freq(Config::TMC5160T_Driver::PWM_FREQ);

    // Configure spreadCycle
    driver.en_pwm_mode(!spreadCycleEnabled);  // 0 for spread cycle, 1 for stealthChop
    driver.toff(Config::TMC5160T_Driver::TOFF);
    driver.blank_time(Config::TMC5160T_Driver::BLANK_TIME);
    driver.hysteresis_start(Config::TMC5160T_Driver::HSTRT);
    driver.hysteresis_end(Config::TMC5160T_Driver::HEND);

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
    if (status == 0 || status == 0xFFFFFFFF)
    {
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
    String message = F("Direction ");
    message.concat(forward ? F("Forward") : F("Reverse"));
    logger->info(message, LogModule::MOTOR, motorName());
}

// Start motor movement in forward direction
void MotorController::moveForward()
{
    if (!diagnoseTMC5160())
    {
        logger->error(F("Motor will not move."), LogModule::MOTOR, motorName());
        return;
    }
    setMovementDirection(true);
}

// Start motor movement in reverse direction
void MotorController::moveReverse()
{
    if (!diagnoseTMC5160())
    {
        logger->error(F("Motor will not move."), LogModule::MOTOR, motorName());
        return;
    }
    setMovementDirection(false);
}

// Stop motor movement
void MotorController::stop()
{
    isMoving = false;
    driver.ihold(100);  // Reduce to ultra low hold current
}

// Execute a single step
void MotorController::step()
{
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPin, LOW);
    lastStepTime = micros();

    if (++stepCounter >= Config::MotorController::STATUS_PRINT_INTERVAL)
    {
        stepCounter = 0;
    }
}

// Main update loop for motor control
void MotorController::update()
{
    if (isMoving)
    {
        // Handle step timing
        if (micros() - lastStepTime >= (1000000 / speed))
        {
            step();
        }

        // Update diagnostics less frequently
        static unsigned long lastDiagnosticTime = 0;
        if (diagnosticsEnabled && millis() - lastDiagnosticTime >= 100)
        {
            updateDiagnostics();
            lastDiagnosticTime = millis();
        }

        // Check load and optimize current less frequently
        static unsigned long lastLoadCheckTime = 0;
        if (millis() - lastLoadCheckTime >= 50)
        {
            checkLoad();
            optimizeCurrent();
            lastLoadCheckTime = millis();
        }

        // Monitor temperature less frequently
        static unsigned long lastTempCheckTime = 0;
        if (millis() - lastTempCheckTime >= 200)
        {
            int temp = getTemperature();
            if (temp > Config::TMC5160T_Driver::TEMP_WARNING_THRESHOLD)
            {
                String message = F("High temperature detected: ");
                message.concat(String(temp));
                logger->warning(message, LogModule::MOTOR, motorName());
                uint16_t reducedCurrent = runCurrent * 0.8;
                driver.rms_current(reducedCurrent);
            }
            lastTempCheckTime = millis();
        }

        // Print temperature at configured interval
        if (millis() - lastTempPrintTime >= Config::TMC5160T_Driver::TEMP_PRINT_INTERVAL)
        {
            printTemperature();
            lastTempPrintTime = millis();
        }

        // Adjust microstepping based on speed
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
    if (runCurrent < Config::MotorController::MAX_RUN_CURRENT)
    {
        runCurrent += Config::MotorController::CURRENT_STEP;
        driver.rms_current(runCurrent);
        String message = F("Run current increased to: ");
        message.concat(String(runCurrent));
        message.concat(F("mA (Max: 1000mA)"));
        logger->info(message, LogModule::MOTOR, motorName());
    }
    else
    {
        logger->warning(F("Run current at maximum (1000mA)"));
    }
}

void MotorController::decreaseRunCurrent()
{
    if (runCurrent > Config::MotorController::MIN_CURRENT)
    {
        runCurrent -= Config::MotorController::CURRENT_STEP;
        driver.rms_current(runCurrent);
        String message = F("Run current decreased to: ");
        message.concat(String(runCurrent));
        message.concat(F("mA (Min: 100mA)"));
        logger->info(message, LogModule::MOTOR, motorName());
    }
    else
    {
        logger->warning(F("Run current at minimum (100mA)"));
    }
}

void MotorController::increaseHoldCurrent()
{
    if (holdCurrent < Config::MotorController::MAX_HOLD_CURRENT)
    {
        holdCurrent += Config::MotorController::CURRENT_STEP;
        driver.ihold(holdCurrent);
        String message = F("Hold current increased to: ");
        message.concat(String(holdCurrent));
        message.concat(F("mA (Max: 500mA)"));
        logger->info(message, LogModule::MOTOR, motorName());
    }
    else
    {
        logger->warning(F("Hold current at maximum (500mA)"));
    }
}

void MotorController::decreaseHoldCurrent()
{
    if (holdCurrent > Config::MotorController::MIN_CURRENT)
    {
        holdCurrent -= Config::MotorController::CURRENT_STEP;
        driver.ihold(holdCurrent);
        String message = F("Hold current decreased to: ");
        message.concat(String(holdCurrent));
        message.concat(F("mA (Min: 100mA)"));
        logger->info(message, LogModule::MOTOR, motorName());
    }
    else
    {
        logger->warning(F("Hold current at minimum (100mA)"));
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
    if (speed < Config::MotorController::MAX_SPEED)
    {
        speed += Config::MotorController::SPEED_STEP;
        String message = F("Speed increased to: ");
        message.concat(String(speed));
        message.concat(F(" steps/sec"));
        logger->info(message, LogModule::MOTOR, motorName());
    }
    else
    {
        logger->warning(F("Speed at maximum (10000 steps/sec)"));
    }
}

void MotorController::decreaseSpeed()
{
    if (speed > Config::MotorController::MIN_SPEED)
    {
        speed -= Config::MotorController::SPEED_STEP;
        String message = F("Speed decreased to: ");
        message.concat(String(speed));
        message.concat(F(" steps/sec"));
        logger->info(message, LogModule::MOTOR, motorName());
    }
    else
    {
        logger->warning(F("Speed at minimum (100 steps/sec)"));
    }
}

void MotorController::increaseAcceleration()
{
    if (acceleration < Config::MotorController::MAX_ACCEL)
    {
        acceleration += Config::MotorController::ACCEL_STEP;
        driver.AMAX(acceleration);
        String message = F("Acceleration increased to: ");
        message.concat(String(acceleration));
        message.concat(F(" steps/sec²"));
        logger->info(message, LogModule::MOTOR, motorName());
    }
    else
    {
        logger->warning(F("Acceleration at maximum (10000 steps/sec²)"));
    }
}

void MotorController::decreaseAcceleration()
{
    if (acceleration > Config::MotorController::MIN_ACCEL)
    {
        acceleration -= Config::MotorController::ACCEL_STEP;
        driver.AMAX(acceleration);
        String message = F("Acceleration decreased to: ");
        message.concat(String(acceleration));
        message.concat(F(" steps/sec²"));
        logger->info(message, LogModule::MOTOR, motorName());
    }
    else
    {
        logger->warning(F("Acceleration at minimum (100 steps/sec²)"));
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
    String message = F("Driver Status Register:");
    message.concat(F("   Raw Status: 0x"));
    message.concat(String(status, HEX));
    logger->info(message, LogModule::MOTOR, motorName());
    printErrorFlags(status);
    printStallGuardStatus(status);
    printDriverState(status);
}

void MotorController::printErrorFlags(uint32_t status)
{
    String message = F("Over Temperature: ");
    if (status & 0x00000001)
    {
        message.concat(F("Yes"));
        logger->error(message, LogModule::MOTOR, motorName());
    }
    else
    {
        message.concat(F("No"));
        logger->info(message, LogModule::MOTOR, motorName());
    }

    message = "";
    message.concat(F("Short to Ground A: "));
    if (status & 0x00000002)
    {
        message.concat(F("Yes"));
        logger->error(message, LogModule::MOTOR, motorName());
    }
    else
    {
        message.concat(F("No"));
        logger->info(message, LogModule::MOTOR, motorName());
    }

    message = "";
    message.concat(F("Short to Ground B: "));
    if (status & 0x00000004)
    {
        message.concat(F("Yes"));
        logger->error(message, LogModule::MOTOR, motorName());
    }
    else
    {
        message.concat(F("No"));
        logger->info(message, LogModule::MOTOR, motorName());
    }

    message = "";
    message.concat(F("Open Load A: "));
    if (status & 0x00000008)
    {
        message.concat(F("Yes"));
        logger->error(message, LogModule::MOTOR, motorName());
    }
    else
    {
        message.concat(F("No"));
        logger->info(message, LogModule::MOTOR, motorName());
    }

    message = "";
    message.concat(F("Open Load B: "));
    if (status & 0x00000010)
    {
        message.concat(F("Yes"));
        logger->error(message, LogModule::MOTOR, motorName());
    }
    else
    {
        message.concat(F("No"));
        logger->info(message, LogModule::MOTOR, motorName());
    }
}

void MotorController::printStallGuardStatus(uint32_t status)
{
    String message = F("StallGuard Value: ");
    message.concat(String((status >> 10) & 0x3FF));
    logger->info(message, LogModule::MOTOR, motorName());

    message = "";
    message.concat(F("Stall Detected: "));
    if (status & 0x00000200)
    {
        message.concat(F("Yes"));
        logger->error(message, LogModule::MOTOR, motorName());
    }
    else
    {
        message.concat(F("No"));
        logger->info(message, LogModule::MOTOR, motorName());
    }
}

void MotorController::printDriverState(uint32_t status)
{
    String message = F("Standstill: ");
    if (status & 0x00000400)
    {
        message.concat(F("Yes"));
        logger->error(message, LogModule::MOTOR, motorName());
    }
    else
    {
        message.concat(F("No"));
        logger->info(message, LogModule::MOTOR, motorName());
    }

    message = "";
    message.concat(F("Velocity Reached: "));
    if (status & 0x00000800)
    {
        message.concat(F("Yes"));
        logger->error(message, LogModule::MOTOR, motorName());
    }
    else
    {
        message.concat(F("No"));
        logger->info(message, LogModule::MOTOR, motorName());
    }

    message = "";
    message.concat(F("Position Reached: "));
    if (status & 0x00001000)
    {
        message.concat(F("Yes"));
        logger->error(message, LogModule::MOTOR, motorName());
    }
    else
    {
        message.concat(F("No"));
        logger->info(message, LogModule::MOTOR, motorName());
    }
}

void MotorController::printDriverStatus()
{
    uint32_t status = driver.DRV_STATUS();

    String message = F("DRV_STATUS Report");

    message = (status & (1 << 31) ? F("Standstill (stst)") : F("Motor moving"));
    logger->info(message, LogModule::MOTOR, motorName());

    message = (status & (1 << 30) ? F("Open load on Phase B (olb)") : F("Phase B OK"));
    logger->info(message, LogModule::MOTOR, motorName());

    message = (status & (1 << 29) ? F("Open load on Phase A (ola)") : F("Phase A OK"));
    logger->info(message, LogModule::MOTOR, motorName());

    message = (status & (1 << 28) ? F("Short to GND on Phase B (s2gb)") : F("Phase B GND OK"));
    logger->info(message, LogModule::MOTOR, motorName());

    message = (status & (1 << 27) ? F("Short to GND on Phase A (s2ga)") : F("Phase A GND OK"));
    logger->info(message, LogModule::MOTOR, motorName());

    message = (status & (1 << 26) ? F("Overtemperature pre-warning (otpw)") : F("Temp OK"));
    logger->info(message, LogModule::MOTOR, motorName());

    message = (status & (1 << 25) ? F("Overtemperature shutdown (ot)") : F("Not overheated"));
    logger->info(message, LogModule::MOTOR, motorName());

    message = (status & (1 << 24) ? F("StallGuard: Stall detected!") : F("No stall"));
    logger->info(message, LogModule::MOTOR, motorName());

    message = (status & (1 << 15) ? F("Fullstep active (fsactive)") : F("Microstepping active"));
    logger->info(message, LogModule::MOTOR, motorName());

    message = (status & (1 << 14) ? F("StealthChop active (stealth)") : F("SpreadCycle active"));
    logger->info(message, LogModule::MOTOR, motorName());

    message = (status & (1 << 13) ? F("Short to V+ on Phase B (s2vbs)") : F("Phase B Supply OK"));
    logger->info(message, LogModule::MOTOR, motorName());

    message = (status & (1 << 12) ? F("Short to V+ on Phase A (s2vsa)") : F("Phase A Supply OK"));
    logger->info(message, LogModule::MOTOR, motorName());

    uint8_t cs_actual = (status >> 17) & 0x0F;
    message           = (F("CS_ACTUAL (current scaling): "));
    message.concat(cs_actual);
    logger->info(message, LogModule::MOTOR, motorName());

    float current_mA = cs_actual / 32.0 * driver.rms_current();
    message          = (F("Estimated actual current = "));
    message.concat(current_mA);
    message.concat(F(" mA"));
    logger->info(message, LogModule::MOTOR, motorName());

    uint16_t sg_result = status & 0x03FF;
    if (sg_result < 100)
    {
        message = (F(" Possible stall condition!"));
    }
    else if (sg_result < 500)
    {
        message = (F("Moderate load"));
    }
    else
    {
        message = (F("Light load"));
    }

    logger->info(message, LogModule::MOTOR, motorName());
}

bool MotorController::diagnoseTMC5160()
{
    uint32_t status = driver.DRV_STATUS();
    bool     ok     = true;

    if (status & (1 << 27))
    {
        logger->error(F("Short to GND on Phase A (s2ga)"), LogModule::MOTOR, motorName());
        ok = false;
    }

    if (status & (1 << 28))
    {
        logger->error(F("Short to GND on Phase B (s2gb)"), LogModule::MOTOR, motorName());
        ok = false;
    }

    if (status & (1 << 12))
    {
        logger->error(F("Short to supply on Phase A (s2vsa)"), LogModule::MOTOR, motorName());
        ok = false;
    }

    if (status & (1 << 13))
    {
        logger->error(F("Short to supply on Phase B (s2vsb)"), LogModule::MOTOR, motorName());
        ok = false;
    }

    if (status & (1 << 25))
    {
        logger->error(F("Overtemperature shutdown active (ot)"), LogModule::MOTOR, motorName());
        ok = false;
    }

    if (status & (1 << 24))
    {
        logger->warning(F("Motor stall detected (StallGuard)"), LogModule::MOTOR, motorName());
        ok = false;
    }

    if (ok)
    {
        logger->info(F("All driver diagnostics OK."), LogModule::MOTOR, motorName());
    }

    return ok;
}

void MotorController::printDriverConfig()
{
    String message = F("Run Current: ");
    message.concat(String(runCurrent));
    message.concat(F("mA"));
    logger->info(message, LogModule::MOTOR, motorName());

    message = F("Hold Current: ");
    message.concat(String(holdCurrent));
    message.concat(F("mA"));
    logger->info(message, LogModule::MOTOR, motorName());

    message = F("Microsteps: ");
    message.concat(String(16));
    logger->info(message, LogModule::MOTOR, motorName());

    message = F("Speed: ");
    message.concat(String(speed));
    message.concat(F(" steps/sec"));
    logger->info(message, LogModule::MOTOR, motorName());

    message = F("Acceleration: ");
    message.concat(String(acceleration));
    message.concat(F(" steps/sec²"));
    logger->info(message, LogModule::MOTOR, motorName());

    message = F("GCONF (Global Config): 0x");
    message.concat(String(driver.GCONF(), HEX));
    logger->info(message, LogModule::MOTOR, motorName());

    message = F("TPOWERDOWN (Power Down Time): ");
    message.concat(String(driver.TPOWERDOWN()));
    message.concat(F(" tclk"));
    logger->info(message, LogModule::MOTOR, motorName());

    message = F("TSTEP (Current Step Timing): ");
    message.concat(String(driver.TSTEP()));
    message.concat(F(" tclk"));
    logger->info(message, LogModule::MOTOR, motorName());

    message = F("TPWMTHRS (StealthChop Threshold): ");
    message.concat(String(driver.TPWMTHRS()));
    message.concat(F(" tclk"));
    logger->info(message, LogModule::MOTOR, motorName());

    message = F("THIGH (Step Pulse High Time): ");
    message.concat(String(driver.THIGH()));
    message.concat(F(" tclk"));
    logger->info(message, LogModule::MOTOR, motorName());

    message = F("XDIRECT (Direct Coil Control): 0x");
    message.concat(String(driver.XDIRECT(), HEX));
    logger->info(message, LogModule::MOTOR, motorName());
}

int MotorController::getTemperature()
{
    uint32_t status  = driver.DRV_STATUS();
    int      rawTemp = (status >> 16) & 0xFF;  // Temperature is in bits 16-23
    return rawTemp;                            // Direct temperature reading in °C (1°C steps)
}

void MotorController::printTemperature()
{
    int temp = getTemperature();
    if (temp != lastTemperature)
    {
        String message = F("Temperature: ");
        message.concat(String(temp));
        message.concat(F("°C"));
        logger->info(message, LogModule::MOTOR, motorName());
        lastTemperature = temp;
    }
}

// Check for motor stall condition
void MotorController::checkStall()
{
    uint32_t status = driver.DRV_STATUS();
    if (status & 0x00000200)
    {
        logger->warning(F("Stall detected!"), LogModule::MOTOR, motorName());
        stop();  // Stop motor on stall
        printStallGuardStatus(status);
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
        logger->info(F("Switched to SpreadCycle mode (more power, more noise)"), LogModule::MOTOR, String(motorName()));
    }
    else
    {
        // Currently in SpreadCycle mode, switch to StealthChop
        driver.TPWMTHRS(0);  // Enable StealthChop mode
        logger->info(F("Switched to StealthChop mode (silent operation)"), LogModule::MOTOR, motorName());
    }
}

void MotorController::setStealthChopMode(bool enable)
{
    if (enable)
    {
        driver.TPWMTHRS(0);  // Enable StealthChop full-time
        logger->info(F("StealthChop enabled"), LogModule::MOTOR, motorName());
    }
    else
    {
        driver.TPWMTHRS(300);  // Enable SpreadCycle above threshold
        logger->info(F("SpreadCycle enabled (TPWMTHRS = 300)"), LogModule::MOTOR, motorName());
    }
}

// Performs a basic SPI communication test by sending a test pattern
bool MotorController::testCommunication(bool enableMessage)
{
    String message = "";

    if (enableMessage)
    {
        message = F("Testing SPI communication with TMC5160: ");
    }

    enableSPI();

    uint32_t gconf  = driver.GCONF();
    uint32_t status = driver.DRV_STATUS();

    if (gconf == 0xFFFFFFFF || status == 0xFFFFFFFF)
    {
        if (enableMessage)
        {
            message.concat(F("Failed"));
        }

        logger->error(message, LogModule::MOTOR, motorName());
        return false;
    }

    if (enableMessage)
    {
        message.concat(F("Ok"));
    }

    logger->info(message, LogModule::MOTOR, motorName());
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
    setStallGuardThreshold(Config::TMC5160T_Driver::SGTHRS);
    setCoolStepThreshold(Config::TMC5160T_Driver::TCOOLTHRS);

    // Configure for high precision
    driver.microsteps(256);  // Maximum microstepping for smooth motion
    driver.intpol(true);     // Enable microstep interpolation

    // Optimize current control
    setCurrentScaling(Config::TMC5160T_Driver::CURRENT_SCALING);
    setCurrentHoldDelay(Config::TMC5160T_Driver::IHOLDDELAY);
    setCurrentRunDelay(Config::TMC5160T_Driver::IRUNDELAY);

    // Configure motion control
    setRampMode(0);  // Positioning mode for precise control
    setMaxSpeed(Config::MotorSpecs::Operation::MAX_SPEED);
    setMaxAcceleration(Config::MotorSpecs::Operation::MAX_ACCELERATION);
    setMaxDeceleration(Config::MotorSpecs::Operation::MAX_DECELERATION);
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

    uint32_t load_value = getLoadValue();
    int      temp       = getTemperature();

    String message = F("Diagnostics -> Load: ");
    message.concat(String(load_value));
    message.concat(F(", Temp: "));
    message.concat(String(temp));
    message.concat(F("°C"));

    logger->info(message, LogModule::MOTOR, motorName());

    if (isStalled())
    {
        handleStall();
    }
}

// Handle stall condition
void MotorController::handleStall()
{
    String message = F("Stall detected:");
    logger->warning(message, LogModule::MOTOR, motorName());

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
    uint32_t load = getLoadValue();
    if (load > Config::TMC5160T_Driver::LOAD_THRESHOLD)
    {
        // Increase current by 20% but not above MAX_RUN_CURRENT
        uint16_t newCurrent = static_cast<uint16_t>(std::min(
            static_cast<double>(runCurrent * 1.2), static_cast<double>(Config::MotorController::MAX_RUN_CURRENT)));
        driver.rms_current(newCurrent);
        runCurrent = newCurrent;
    }
    else if (load < Config::TMC5160T_Driver::LOAD_THRESHOLD / 2)
    {
        // Decrease current by 20% but not below MIN_CURRENT
        uint16_t newCurrent = static_cast<uint16_t>(
            std::max(static_cast<double>(runCurrent * 0.8), static_cast<double>(Config::MotorController::MIN_CURRENT)));
        driver.rms_current(newCurrent);
        runCurrent = newCurrent;
    }
}

// Check motor load
void MotorController::checkLoad()
{
    uint32_t load = getLoadValue();
    if (load > Config::TMC5160T_Driver::LOAD_WARNING_THRESHOLD)
    {
        String message = F("High load detected: ");
        message.concat(String(load));
        logger->warning(message, LogModule::MOTOR, motorName());
        optimizeCurrent();
    }
}

// Adjust microstepping based on speed
void MotorController::adjustMicrostepping()
{
    if (speed > Config::MotorSpecs::Operation::HIGH_SPEED_THRESHOLD)
    {
        driver.microsteps(8);  // Reduce microstepping at high speeds
    }
    else
    {
        driver.microsteps(Config::TMC5160T_Driver::MICROSTEPS);
    }
}