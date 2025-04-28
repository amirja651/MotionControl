#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <TMCStepper.h>

#define NUM_MOTORS 4

static const uint16_t DIR_A = 22;
static const uint16_t DIR_B = 15;
static const uint16_t DIR_C = 32;
static const uint16_t DIR_D = 27;

static const uint16_t STEP_A = 21;
static const uint16_t STEP_B = 2;
static const uint16_t STEP_C = 33;
static const uint16_t STEP_D = 14;

static const uint16_t EN_A = 17;
static const uint16_t EN_B = 16;
static const uint16_t EN_C = 26;
static const uint16_t EN_D = 13;

static const uint16_t CS_A = 5;
static const uint16_t CS_B = 4;
static const uint16_t CS_C = 25;
static const uint16_t CS_D = 12;

static const uint16_t W_MOSI = 23;
static const uint16_t W_MISO = 19;
static const uint16_t W_SCK  = 18;

enum class MotorType
{
    ROTATIONAL,
    LINEAR
};

static bool      isMoving[NUM_MOTORS]  = {false, false, false, false};
static MotorType motorType[NUM_MOTORS] = {MotorType::LINEAR, MotorType::ROTATIONAL, MotorType::ROTATIONAL, MotorType::ROTATIONAL};

// Define driver objects
TMC5160Stepper driver[NUM_MOTORS] = {
    TMC5160Stepper(CS_A, 0.075, W_MOSI, W_MISO, W_SCK), TMC5160Stepper(CS_B, 0.075, W_MOSI, W_MISO, W_SCK),
    TMC5160Stepper(CS_C, 0.075, W_MOSI, W_MISO, W_SCK), TMC5160Stepper(CS_D, 0.075, W_MOSI, W_MISO, W_SCK)};

void driversPinSetup()
{
    // Setup pins
    pinMode(DIR_A, OUTPUT);
    pinMode(DIR_B, OUTPUT);
    pinMode(DIR_C, OUTPUT);
    pinMode(DIR_D, OUTPUT);

    pinMode(STEP_A, OUTPUT);
    pinMode(STEP_B, OUTPUT);
    pinMode(STEP_C, OUTPUT);
    pinMode(STEP_D, OUTPUT);

    pinMode(EN_A, OUTPUT);
    pinMode(EN_B, OUTPUT);
    pinMode(EN_C, OUTPUT);
    pinMode(EN_D, OUTPUT);

    pinMode(CS_A, OUTPUT);
    pinMode(CS_B, OUTPUT);
    pinMode(CS_C, OUTPUT);
    pinMode(CS_D, OUTPUT);

    pinMode(MISO, INPUT_PULLUP);

    digitalWrite(EN_A, HIGH);
    digitalWrite(EN_B, HIGH);
    digitalWrite(EN_C, HIGH);
    digitalWrite(EN_D, HIGH);

    digitalWrite(DIR_A, LOW);
    digitalWrite(DIR_B, LOW);
    digitalWrite(DIR_C, LOW);
    digitalWrite(DIR_D, LOW);

    digitalWrite(STEP_A, LOW);
    digitalWrite(STEP_B, LOW);
    digitalWrite(STEP_C, LOW);
    digitalWrite(STEP_D, LOW);
}

void disableDrivers()
{
    digitalWrite(CS_A, HIGH);
    digitalWrite(CS_B, HIGH);
    digitalWrite(CS_C, HIGH);
    digitalWrite(CS_D, HIGH);
}

void disableMotor(uint8_t i)
{
    if (i == 0)
    {
        digitalWrite(EN_A, HIGH);
    }
    else if (i == 1)
    {
        digitalWrite(EN_B, HIGH);
    }
    else if (i == 2)
    {
        digitalWrite(EN_C, HIGH);
    }
    else if (i == 3)
    {
        digitalWrite(EN_D, HIGH);
    }
}

void enableMotor(uint8_t i)
{
    if (i == 0)
    {
        digitalWrite(EN_A, LOW);
    }
    else if (i == 1)
    {
        digitalWrite(EN_B, LOW);
    }
    else if (i == 2)
    {
        digitalWrite(EN_C, LOW);
    }
    else if (i == 3)
    {
        digitalWrite(EN_D, LOW);
    }
}

uint8_t selectDriver(uint8_t i)
{
    if (i == 0)
    {
        return CS_A;
    }
    else if (i == 1)
    {
        return CS_B;
    }
    else if (i == 2)
    {
        return CS_C;
    }
    else if (i == 3)
    {
        return CS_D;
    }
    else
    {
        return 0;
    }
}

void optimizeForPancake(uint8_t i)
{
    digitalWrite(selectDriver(i), LOW);
    delayMicroseconds(50);

    // Set optimal parameters for pancake motor
    driver[i].intpol(true);     // Enable microstep interpolation
    driver[i].sfilt(true);      // Enable StallGuard filter
    driver[i].sgt(10);          // Set StallGuard threshold
    driver[i].TCOOLTHRS(1000);  // Set CoolStep threshold

    // Configure for high precision
    driver[i].microsteps(16);  // Maximum microstepping for smooth motion
    driver[i].intpol(true);    // Enable microstep interpolation

    // Optimize current control
    driver[i].ihold(100 * 32 / 32);  // Set hold current
    driver[i].irun(200 * 32 / 32);   // Set run current

    driver[i].iholddelay(6);  // Set hold current delay

    // Configure motion control
    driver[i].RAMPMODE(0);  // Positioning mode for precise control
    driver[i].VMAX(500);    // Set maximum speed
    driver[i].a1(500);      // Set maximum acceleration
    driver[i].d1(500);      // Set maximum deceleration

    delayMicroseconds(50);
    digitalWrite(selectDriver(i), HIGH);
}

void optimize2(uint8_t i)
{
    digitalWrite(selectDriver(i), LOW);
    delayMicroseconds(50);

    // Set optimal parameters for pancake motor
    driver[i].intpol(true);     // Enable microstep interpolation
    driver[i].sfilt(true);      // Enable StallGuard filter
    driver[i].sgt(10);          // Set StallGuard threshold
    driver[i].TCOOLTHRS(1000);  // Set CoolStep threshold

    // Configure for high precision
    driver[i].microsteps(16);  // Maximum microstepping for smooth motion
    driver[i].intpol(true);    // Enable microstep interpolation

    // Optimize current control
    driver[i].ihold(100 * 32 / 32);  // Set hold current
    driver[i].irun(200 * 32 / 32);   // Set run current

    driver[i].iholddelay(6);  // Set hold current delay

    // Configure motion control
    driver[i].RAMPMODE(0);  // Positioning mode for precise control
    driver[i].VMAX(500);    // Set maximum speed
    driver[i].a1(500);      // Set maximum acceleration
    driver[i].d1(500);      // Set maximum deceleration

    delayMicroseconds(50);
    digitalWrite(selectDriver(i), HIGH);
}

void configureDriver(uint8_t i)
{
    digitalWrite(selectDriver(i), LOW);
    delayMicroseconds(50);

    // Configure GCONF register for optimal performance
    uint32_t gconf = 0;
    gconf |= (1 << 0);  // Enable internal RSense
    gconf |= (1 << 2);  // Enable stealthChop
    gconf |= (1 << 3);  // Enable microstep interpolation
    gconf |= (1 << 4);  // Enable double edge step
    gconf |= (1 << 6);  // Enable multistep filtering
    driver[i].GCONF(gconf);

    // Set current control parameters
    driver[i].rms_current(200);  // Set motor RMS current
    driver[i].ihold(100);        // Set hold current
    driver[i].irun(200);         // Set run current
    driver[i].iholddelay(6);     // Set hold delay
    driver[i].TPOWERDOWN(10);    // Set power down time

    // Configure microstepping
    driver[i].microsteps(16);  // Set microsteps
    driver[i].intpol(true);    // Set microstep interpolation

    // Configure CoolStep
    driver[i].TCOOLTHRS(1000);  // Set CoolStep threshold
    driver[i].sgt(10);          // Set StallGuard threshold
    driver[i].sfilt(true);      // Set StallGuard filter
    driver[i].sgt(10);          // Set StallGuard threshold

    // Configure stealthChop
    driver[i].TPWMTHRS(0);          // Enable stealthChop by default
    driver[i].pwm_autoscale(true);  // Enable PWM autoscale
    driver[i].pwm_autograd(true);   // Enable PWM autograd
    driver[i].pwm_ofs(36);          // Set PWM offset
    driver[i].pwm_grad(14);         // Set PWM gradient
    driver[i].pwm_freq(1);          // Set PWM frequency

    // Configure spreadCycle
    driver[i].en_pwm_mode(1);       // 0 for spread cycle, 1 for stealthChop
    driver[i].toff(3);              // Set turn-off time
    driver[i].blank_time(24);       // Set blank time
    driver[i].hysteresis_start(5);  // Set hysteresis start
    driver[i].hysteresis_end(3);    // Set hysteresis end

    // Configure motion control
    driver[i].RAMPMODE(0);  // Set ramp mode
    driver[i].VMAX(500);    // Set maximum speed
    driver[i].AMAX(500);    // Set maximum acceleration
    driver[i].DMAX(500);    // Set maximum deceleration
    driver[i].a1(500);      // Set minimum acceleration
    driver[i].v1(500 / 2);  // Set minimum speed
    driver[i].d1(500);      // Set minimum deceleration
    driver[i].VSTART(0);    // Set start velocity
    driver[i].VSTOP(10);    // Set stop velocity

    delayMicroseconds(50);
    digitalWrite(selectDriver(i), HIGH);
}

bool driverCommunicationTest(uint8_t i, bool print = true)
{
    uint8_t  version  = 0;
    uint32_t gconf    = 0;
    uint32_t status   = 0;
    uint32_t chopconf = 0;

    digitalWrite(selectDriver(i), LOW);
    delayMicroseconds(50);
    version = driver[i].version();
    delayMicroseconds(50);
    digitalWrite(selectDriver(i), HIGH);
    delay(100);

    if (print)
    {
        Serial.print(F("Driver "));
        Serial.print(i);
        Serial.print(F(": Version read attempt: 0x"));
        Serial.println(version, HEX);
    }

    if (version == 0xFF || version == 0)
    {
        if (print)
        {
            Serial.print(F("Driver "));
            Serial.print(i);
            Serial.print(F(": Invalid version (0x"));
            Serial.print(version, HEX);
            Serial.println(F(")"));
        }
        return false;
    }

    // Test GCONF register
    digitalWrite(selectDriver(i), LOW);
    delayMicroseconds(50);
    gconf = driver[i].GCONF();
    delayMicroseconds(50);
    digitalWrite(selectDriver(i), HIGH);

    if (print)
    {
        Serial.print(F("Driver "));
        Serial.print(i);
        Serial.print(F(": GCONF read: 0x"));
        Serial.println(gconf, HEX);
    }

    if (gconf == 0xFFFFFFFF)
    {
        if (print)
        {
            Serial.print(F("Driver "));
            Serial.print(i);
            Serial.println(F(": GCONF register read failed"));
        }
        return false;
    }

    // Test DRV_STATUS register
    digitalWrite(selectDriver(i), LOW);
    delayMicroseconds(50);
    status = driver[i].DRV_STATUS();
    delayMicroseconds(50);
    digitalWrite(selectDriver(i), HIGH);

    if (print)
    {
        Serial.print(F("Driver "));
        Serial.print(i);
        Serial.print(F(": DRV_STATUS read: 0x"));
        Serial.println(status, HEX);
    }

    if (status == 0xFFFFFFFF)
    {
        if (print)
        {
            Serial.print(F("Driver "));
            Serial.print(i);
            Serial.println(F(": DRV_STATUS register read failed"));
        }
        return false;
    }

    // Test CHOPCONF register
    digitalWrite(selectDriver(i), LOW);
    delayMicroseconds(50);
    chopconf = driver[i].CHOPCONF();
    delayMicroseconds(50);
    digitalWrite(selectDriver(i), HIGH);

    if (print)
    {
        Serial.print(F("Driver "));
        Serial.print(i);
        Serial.print(F(": CHOPCONF read: 0x"));
        Serial.println(chopconf, HEX);
    }

    if (chopconf == 0xFFFFFFFF)
    {
        if (print)
        {
            Serial.print(F("Driver "));
            Serial.print(i);
            Serial.println(F(": CHOPCONF register read failed"));
        }
        return false;
    }

    // Test if driver is responding to commands
    digitalWrite(selectDriver(i), LOW);
    delayMicroseconds(50);
    driver[i].GCONF(gconf);  // Write back the same value
    delayMicroseconds(50);
    digitalWrite(selectDriver(i), HIGH);

    digitalWrite(selectDriver(i), LOW);
    delayMicroseconds(50);
    uint32_t readback = driver[i].GCONF();
    delayMicroseconds(50);
    digitalWrite(selectDriver(i), HIGH);

    if (print)
    {
        Serial.print(F("Driver "));
        Serial.print(i);
        Serial.print(F(": GCONF write/read test: Original=0x"));
        Serial.print(gconf, HEX);
        Serial.print(F(", Readback=0x"));
        Serial.println(readback, HEX);
    }

    if (readback != gconf)
    {
        if (print)
        {
            Serial.print(F("Driver "));
            Serial.print(i);
            Serial.println(F(": GCONF register write/read mismatch"));
        }
        return false;
    }

    if (print)
    {
        Serial.print(F("Driver "));
        Serial.print(i);
        Serial.print(F(": Communication test passed (Version: 0x"));
        Serial.print(version, HEX);
        Serial.println(F(")"));
    }
    return true;
}

void driverTest(uint8_t i)
{
    if (!driverCommunicationTest(i))
    {
        Serial.print(F("Driver "));
        Serial.print(i);
        Serial.println(F(" communication test: FAILED"));
    }
    else
    {
        Serial.print(F("Driver "));
        Serial.print(i);
        Serial.print(F(" firmware version: "));
        Serial.println(driver[i].version());

        if (driver[i].sd_mode())
        {
            Serial.print(F("Driver "));
            Serial.print(i);
            Serial.println(F(" is hardware configured for Step & Dir mode"));
        }

        if (driver[i].drv_enn())
        {
            Serial.print(F("Driver "));
            Serial.print(i);
            Serial.println(F(" is not hardware enabled"));
        }
    }
    delay(500);
}

void initializeDriver(uint8_t i)
{
    digitalWrite(selectDriver(i), LOW);
    delayMicroseconds(50);

    driver[i].irun(200);   // Default 1000mA
    driver[i].ihold(100);  // Default 500mA
    driver[i].VMAX(500);   // Default 1000 steps/sec
    driver[i].AMAX(500);   // Default 1000 steps/sec²
    driver[i].DMAX(500);   // Default 1000 steps/sec²

    delayMicroseconds(50);
    digitalWrite(selectDriver(i), HIGH);

    configureDriver(i);
    optimizeForPancake(i);

    isMoving[i] = false;
}

void initializeDriversAndTest()
{
    driversPinSetup();
    delay(100);

    disableDrivers();
    driver[0].begin();
    driverTest(0);
    initializeDriver(0);
    Serial.println(F("--------------------------------"));

    disableDrivers();
    driver[1].begin();
    driverTest(1);
    initializeDriver(1);
    Serial.println(F("--------------------------------"));

    disableDrivers();
    driver[2].begin();
    driverTest(2);
    initializeDriver(2);
    Serial.println(F("--------------------------------"));

    disableDrivers();
    driver[3].begin();
    driverTest(3);
    initializeDriver(3);
    Serial.println(F("--------------------------------"));
}

void motorMoveForward(uint8_t i)
{
    isMoving[i] = true;

    if (i == 0)
    {
        digitalWrite(DIR_A, HIGH);
    }
    else if (i == 1)
    {
        digitalWrite(DIR_B, HIGH);
    }
    else if (i == 2)
    {
        digitalWrite(DIR_C, HIGH);
    }
    else if (i == 3)
    {
        digitalWrite(DIR_D, HIGH);
    }

    enableMotor(i);
}

void motorMoveReverse(uint8_t i)
{
    isMoving[i] = true;

    if (i == 0)
    {
        digitalWrite(DIR_A, LOW);
    }
    else if (i == 1)
    {
        digitalWrite(DIR_B, LOW);
    }
    else if (i == 2)
    {
        digitalWrite(DIR_C, LOW);
    }
    else if (i == 3)
    {
        digitalWrite(DIR_D, LOW);
    }

    enableMotor(i);
}

void motorStop(uint8_t i)
{
    digitalWrite(selectDriver(i), LOW);
    delayMicroseconds(50);
    driver[i].VMAX(0);  // Set target velocity to zero
    delayMicroseconds(50);
    digitalWrite(selectDriver(i), HIGH);

    // Wait for standstill
    uint32_t       status;
    const uint32_t timeout = 1000;  // 1000 ms
    uint32_t       elapsed = 0;
    do
    {
        digitalWrite(selectDriver(i), LOW);
        delayMicroseconds(50);
        status = driver[i].DRV_STATUS();
        delayMicroseconds(50);
        digitalWrite(selectDriver(i), HIGH);

        vTaskDelay(1);
        elapsed++;
        if (elapsed > timeout)
        {
            // Timeout handling: maybe log or break
            break;
        }
    } while (!(status & (1 << 31)));

    digitalWrite(selectDriver(i), LOW);
    delayMicroseconds(50);
    driver[i].ihold(200);  // Reduce to hold current
    delayMicroseconds(50);
    digitalWrite(selectDriver(i), HIGH);
    isMoving[i] = false;

    disableMotor(i);
}

void motorStep(uint8_t i)
{
    if (!isMoving[i])
    {
        return;
    }

    if (i == 0)
    {
        digitalWrite(STEP_A, HIGH);
        delayMicroseconds(160);
        digitalWrite(STEP_A, LOW);
        delayMicroseconds(160);
    }
    else if (i == 1)
    {
        digitalWrite(STEP_B, HIGH);
        delayMicroseconds(160);
        digitalWrite(STEP_B, LOW);
        delayMicroseconds(160);
    }
    else if (i == 2)
    {
        digitalWrite(STEP_C, HIGH);
        delayMicroseconds(160);
        digitalWrite(STEP_C, LOW);
        delayMicroseconds(160);
    }
    else if (i == 3)
    {
        digitalWrite(STEP_D, HIGH);
        delayMicroseconds(160);
        digitalWrite(STEP_D, LOW);
        delayMicroseconds(160);
    }
}

void toggleStealthChop(uint8_t i)
{
    uint32_t currentThreshold = driver[i].TPWMTHRS();
    if (currentThreshold == 0)
    {
        // Currently in StealthChop mode, switch to SpreadCycle
        driver[i].TPWMTHRS(500);  // Switch to SpreadCycle above 500 steps/sec
    }
    else
    {
        // Currently in SpreadCycle mode, switch to StealthChop
        driver[i].TPWMTHRS(0);  // Enable StealthChop mode
    }
}

void setStealthChopMode(uint8_t i, bool enable)
{
    if (enable)
    {
        // Enable StealthChop mode
        driver[i].TPWMTHRS(0);  // Enable StealthChop for all velocities

        // Configure PWM for StealthChop
        driver[i].pwm_autoscale(true);  // Enable automatic current scaling
        driver[i].pwm_autograd(true);   // Enable automatic gradient adaptation
        driver[i].pwm_ofs(36);          // Default PWM offset
        driver[i].pwm_grad(14);         // Default PWM gradient
        driver[i].pwm_freq(1);          // 1 = 23.4kHz PWM frequency

        // Configure chopper for StealthChop
        driver[i].toff(3);              // Minimum time for slow decay phase
        driver[i].hysteresis_start(1);  // Hysteresis start value
        driver[i].hysteresis_end(2);    // Hysteresis end value
        driver[i].blank_time(24);       // Blanking time
    }
    else
    {
        // Switch to SpreadCycle mode
        driver[i].TPWMTHRS(0xFFFFF);  // Disable StealthChop (very high threshold)

        // Configure for SpreadCycle
        driver[i].toff(5);              // Standard time for slow decay phase
        driver[i].hysteresis_start(4);  // Standard hysteresis start
        driver[i].hysteresis_end(1);    // Standard hysteresis end
        driver[i].blank_time(24);       // Standard blanking time
    }
}

#endif
