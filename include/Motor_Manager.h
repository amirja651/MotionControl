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

void configureDriverNEMA11_1004H(uint8_t i)
{
    disableDrivers();
    delay(5);

    // ---------------------------
    // 1. Basic Driver Configuration (GCONF)
    // ---------------------------
    uint32_t gconf = 0;
    gconf |= (1 << 0);  // Internal Rsense
    gconf |= (1 << 2);  // Enable StealthChop
    gconf |= (1 << 3);  // Microstep interpolation
    gconf |= (1 << 4);  // Double edge step
    gconf |= (1 << 6);  // Multistep filtering
    driver[i].GCONF(gconf);
    delay(5);

    // ---------------------------
    // 2. Current Settings
    // ---------------------------
    driver[i].rms_current(800);  // 0.8A RMS (~1.1A peak → مناسب این موتور)
    driver[i].irun(220);         // Run current (~86% max to prevent heating)
    driver[i].ihold(80);         // Hold current (~30% for low heat when idle)
    driver[i].iholddelay(6);     // Delay before switching to hold current
    driver[i].TPOWERDOWN(10);    // Power down after inactivity

    // ---------------------------
    // 3. Microstepping & Interpolation
    // ---------------------------
    driver[i].microsteps(16);  // Fine control, 16 microsteps
    driver[i].intpol(true);    // Enable interpolation for smooth motion

    // ---------------------------
    // 4. StealthChop Settings
    // ---------------------------
    driver[i].TPWMTHRS(500);  // StealthChop active at low speeds
    driver[i].pwm_autoscale(true);
    driver[i].pwm_autograd(true);
    driver[i].pwm_ofs(36);
    driver[i].pwm_grad(14);
    driver[i].pwm_freq(1);

    // ---------------------------
    // 5. SpreadCycle Chopper Settings
    // ---------------------------
    driver[i].en_pwm_mode(true);  // Use StealthChop for low speed, switch to SpreadCycle at higher speeds
    driver[i].toff(4);
    driver[i].blank_time(24);
    driver[i].hysteresis_start(3);
    driver[i].hysteresis_end(1);

    // ---------------------------
    // 6. StallGuard & CoolStep (Optional tuning)
    // ---------------------------
    driver[i].TCOOLTHRS(400);  // Enable CoolStep above moderate speed
    driver[i].sgt(5);          // StallGuard threshold
    driver[i].sfilt(true);

    // ---------------------------
    // 7. Motion Configuration
    // ---------------------------
    driver[i].RAMPMODE(0);  // Positioning mode
    driver[i].VSTART(10);   // Start velocity
    driver[i].VSTOP(10);    // Stop velocity
    driver[i].VMAX(1200);   // Max velocity (steps/sec)
    driver[i].AMAX(400);    // Acceleration (steps/sec²)
    driver[i].DMAX(400);    // Deceleration (steps/sec²)
    driver[i].a1(500);      // Start acceleration phase
    driver[i].d1(500);      // End deceleration phase

    delay(5);
}

// Optimize for pancake motor
void optimizeForPancake(uint8_t i)
{
    disableDrivers();
    delay(5);

    // ---------------------------
    // 1. Basic Driver Configuration (GCONF)
    // ---------------------------
    uint32_t gconf = 0;
    gconf |= (1 << 0);  // Internal Rsense
    gconf |= (1 << 2);  // StealthChop enable (initially)
    gconf |= (1 << 3);  // Microstep interpolation
    gconf |= (1 << 4);  // Double edge step
    gconf |= (1 << 6);  // Multistep filtering
    driver[i].GCONF(gconf);
    delay(5);

    // ---------------------------
    // 2. Current Settings (Low Power Mode)
    // ---------------------------
    driver[i].rms_current(350);  // About 0.35A RMS (safe for Pancake)
    driver[i].irun(200);         // Run current: ~0.35A   #160
    driver[i].ihold(200);        // Hold current: ~0.12A   #60
    driver[i].iholddelay(5);     // Short delay before switching to ihold #10
    driver[i].TPOWERDOWN(10);    // Power down delay

    // ---------------------------
    // 3. Microstepping & Interpolation
    // ---------------------------
    driver[i].microsteps(16);  // Only 4 microsteps → maximize torque
    driver[i].intpol(true);    // Smooth motion

    // ---------------------------
    // 4. StealthChop Settings
    // ---------------------------
    /* driver[i].TPWMTHRS(300);  // StealthChop active only at very low speeds
     driver[i].pwm_autoscale(true);
     driver[i].pwm_autograd(true);
     driver[i].pwm_ofs(36);
     driver[i].pwm_grad(14);
     driver[i].pwm_freq(1);*/

    // ---------------------------
    // 5. SpreadCycle Chopper Settings
    // ---------------------------
    driver[i].en_pwm_mode(false);  // Force SpreadCycle above TPWMTHRS
    driver[i].toff(4);
    driver[i].blank_time(24);
    driver[i].hysteresis_start(3);
    driver[i].hysteresis_end(1);

    // ---------------------------
    // 6. StallGuard & CoolStep
    // ---------------------------
    driver[i].TCOOLTHRS(200);  // CoolStep threshold
    driver[i].sgt(5);          // StallGuard threshold
    driver[i].sfilt(true);

    // ---------------------------
    // 7. Motion Configuration (Soft Motion)
    // ---------------------------
    driver[i].RAMPMODE(0);  // Positioning mode
    driver[i].VSTART(5);    // Very soft start
    driver[i].VSTOP(5);     // Smooth stop
    driver[i].VMAX(600);    // Max speed (limit for Pancake)
    driver[i].AMAX(200);    // Acceleration limit
    driver[i].DMAX(200);    // Deceleration limit
    driver[i].a1(500);      // Start acceleration
    driver[i].d1(500);      // Start deceleration

    delay(5);
}

void configureDriver(uint8_t i)
{
    disableDrivers();
    delay(5);

    // ---------------------------
    // 1. Basic Driver Configuration (GCONF)
    // ---------------------------
    uint32_t gconf = 0;
    gconf |= (1 << 0);  // Internal Rsense
    gconf |= (1 << 2);  // StealthChop enable
    gconf |= (1 << 3);  // Microstep interpolation
    gconf |= (1 << 4);  // Double edge step
    gconf |= (1 << 6);  // Multistep filtering
    driver[i].GCONF(gconf);
    delay(5);

    // ---------------------------
    // 2. Current Settings
    // ---------------------------
    driver[i].rms_current(1000);  // 1A RMS (overrides irun/ihold if set later)
    driver[i].irun(200);          // Run current
    driver[i].ihold(100);         // Hold current
    driver[i].iholddelay(6);      // Delay before switching to ihold
    driver[i].TPOWERDOWN(10);     // Time to power down after inactivity

    // ---------------------------
    // 3. Microstepping & Interpolation
    // ---------------------------
    driver[i].microsteps(32);  // 32 microsteps
    driver[i].intpol(true);    // Enable interpolation

    // ---------------------------
    // 4. StealthChop (Silent Mode)
    // ---------------------------
    driver[i].TPWMTHRS(0);          // StealthChop always enabled
    driver[i].pwm_autoscale(true);  // Auto current scaling
    driver[i].pwm_autograd(true);   // Auto gradient
    driver[i].pwm_ofs(36);          // Offset
    driver[i].pwm_grad(14);         // Gradient
    driver[i].pwm_freq(1);          // ~23.4kHz

    // ---------------------------
    // 5. SpreadCycle & Chopper Config
    // ---------------------------
    driver[i].en_pwm_mode(true);    // 0 = SpreadCycle, 1 = StealthChop
    driver[i].toff(5);              // Off time
    driver[i].blank_time(24);       // Blanking time
    driver[i].hysteresis_start(5);  // Hysteresis start
    driver[i].hysteresis_end(3);    // Hysteresis end

    // ---------------------------
    // 6. StallGuard & CoolStep
    // ---------------------------
    driver[i].TCOOLTHRS(1000);  // Threshold for CoolStep/StallGuard
    driver[i].sgt(10);          // StallGuard threshold
    driver[i].sfilt(true);      // StallGuard filtering

    // ---------------------------
    // 7. Motion Configuration
    // ---------------------------
    driver[i].RAMPMODE(0);  // Positioning mode
    driver[i].VSTART(0);    // Start velocity
    driver[i].VSTOP(10);    // Stop velocity
    driver[i].VMAX(1000);   // Max velocity
    driver[i].AMAX(500);    // Acceleration
    driver[i].DMAX(500);    // Deceleration
    driver[i].a1(10000);    // Initial acceleration phase
    driver[i].d1(10000);    // Initial deceleration phase

    delay(5);
}

bool driverCommunicationTest(uint8_t i, bool print = true)
{
    disableDrivers();

    uint8_t  version  = 0;
    uint32_t gconf    = 0;
    uint32_t status   = 0;
    uint32_t chopconf = 0;

    version = driver[i].version();

    delay(5);

    if (print)
    {
        Serial.print(F("Driver "));
        Serial.print(i + 1);
        Serial.print(F(": Version read attempt: 0x"));
        Serial.println(version, HEX);
    }

    if (version == 0xFF || version == 0)
    {
        if (print)
        {
            Serial.print(F("Driver "));
            Serial.print(i + 1);
            Serial.print(F(": Invalid version (0x"));
            Serial.print(version, HEX);
            Serial.println(F(")"));
        }
        return false;
    }

    // Test GCONF register

    gconf = driver[i].GCONF();

    if (print)
    {
        Serial.print(F("Driver "));
        Serial.print(i + 1);
        Serial.print(F(": GCONF read: 0x"));
        Serial.println(gconf, HEX);
    }

    if (gconf == 0xFFFFFFFF)
    {
        if (print)
        {
            Serial.print(F("Driver "));
            Serial.print(i + 1);
            Serial.println(F(": GCONF register read failed"));
        }
        return false;
    }

    // Test DRV_STATUS register

    status = driver[i].DRV_STATUS();

    if (print)
    {
        Serial.print(F("Driver "));
        Serial.print(i + 1);
        Serial.print(F(": DRV_STATUS read: 0x"));
        Serial.println(status, HEX);
    }

    if (status == 0xFFFFFFFF)
    {
        if (print)
        {
            Serial.print(F("Driver "));
            Serial.print(i + 1);
            Serial.println(F(": DRV_STATUS register read failed"));
        }
        return false;
    }

    // Test CHOPCONF register

    chopconf = driver[i].CHOPCONF();

    if (print)
    {
        Serial.print(F("Driver "));
        Serial.print(i + 1);
        Serial.print(F(": CHOPCONF read: 0x"));
        Serial.println(chopconf, HEX);
    }

    if (chopconf == 0xFFFFFFFF)
    {
        if (print)
        {
            Serial.print(F("Driver "));
            Serial.print(i + 1);
            Serial.println(F(": CHOPCONF register read failed"));
        }
        return false;
    }

    // Test if driver is responding to commands

    driver[i].GCONF(gconf);  // Write back the same value

    uint32_t readback = driver[i].GCONF();

    if (print)
    {
        Serial.print(F("Driver "));
        Serial.print(i + 1);
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
            Serial.print(i + 1);
            Serial.println(F(": GCONF register write/read mismatch"));
        }
        return false;
    }

    if (print)
    {
        Serial.print(F("Driver "));
        Serial.print(i + 1);
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
        Serial.print(i + 1);
        Serial.println(F(" communication test: FAILED"));
    }
    else
    {
        Serial.print(F("Driver "));
        Serial.print(i + 1);
        Serial.print(F(" firmware version: "));
        Serial.println(driver[i].version());

        if (driver[i].sd_mode())
        {
            Serial.print(F("Driver "));
            Serial.print(i + 1);
            Serial.println(F(" is hardware configured for Step & Dir mode"));
        }

        if (driver[i].drv_enn())
        {
            Serial.print(F("Driver "));
            Serial.print(i + 1);
            Serial.println(F(" is not hardware enabled"));
        }
    }
    delay(500);
}

void initializeDriver(uint8_t i)
{
    disableDrivers();

    if (motorType[i] == MotorType::LINEAR)
    {
        configureDriverNEMA11_1004H(i);
    }
    else if (motorType[i] == MotorType::ROTATIONAL)
    {
        optimizeForPancake(i);
    }

    isMoving[i] = false;
}

void initializeDriversAndTest()
{
    driversPinSetup();
    delay(5);

    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        disableDrivers();
        driver[i].begin();
        driverTest(i);
        initializeDriver(i);
        Serial.println(F("--------------------------------"));
    }
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
    disableDrivers();

    driver[i].VMAX(0);  // Set target velocity to zero

    // Wait for standstill
    uint32_t       status;
    const uint32_t timeout = 1000;  // 1000 ms
    uint32_t       elapsed = 0;
    do
    {
        status = driver[i].DRV_STATUS();

        vTaskDelay(1);
        elapsed++;
        if (elapsed > timeout)
        {
            // Timeout handling: maybe log or break
            break;
        }
    } while (!(status & (1 << 31)));

    driver[i].ihold(200);  // Reduce to hold current

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

#endif