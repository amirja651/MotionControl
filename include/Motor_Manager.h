#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <TMCStepper.h>

constexpr uint8_t NUM_MOTORS = 4;

static const uint16_t DIR[NUM_MOTORS]  = {22, 4, 32, 27};
static const uint16_t STEP[NUM_MOTORS] = {21, 16, 33, 14};
static const uint16_t EN[NUM_MOTORS]   = {17, 15, 26, 13};
static const uint16_t CS[NUM_MOTORS]   = {5, 2, 25, 12};

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
    TMC5160Stepper(CS[NUM_MOTORS], 0.075, W_MOSI, W_MISO, W_SCK), TMC5160Stepper(CS[NUM_MOTORS], 0.075, W_MOSI, W_MISO, W_SCK),
    TMC5160Stepper(CS[NUM_MOTORS], 0.075, W_MOSI, W_MISO, W_SCK), TMC5160Stepper(CS[NUM_MOTORS], 0.075, W_MOSI, W_MISO, W_SCK)};

void driversPinSetup()
{
    // Setup pins
    pinMode(DIR[NUM_MOTORS], OUTPUT);
    pinMode(DIR[NUM_MOTORS], OUTPUT);
    pinMode(DIR[NUM_MOTORS], OUTPUT);
    pinMode(DIR[NUM_MOTORS], OUTPUT);

    pinMode(STEP[NUM_MOTORS], OUTPUT);
    pinMode(STEP[NUM_MOTORS], OUTPUT);
    pinMode(STEP[NUM_MOTORS], OUTPUT);
    pinMode(STEP[NUM_MOTORS], OUTPUT);

    pinMode(EN[NUM_MOTORS], OUTPUT);
    pinMode(EN[NUM_MOTORS], OUTPUT);
    pinMode(EN[NUM_MOTORS], OUTPUT);
    pinMode(EN[NUM_MOTORS], OUTPUT);

    pinMode(CS[NUM_MOTORS], OUTPUT);
    pinMode(CS[NUM_MOTORS], OUTPUT);
    pinMode(CS[NUM_MOTORS], OUTPUT);
    pinMode(CS[NUM_MOTORS], OUTPUT);

    pinMode(MISO, INPUT_PULLUP);

    digitalWrite(EN[NUM_MOTORS], HIGH);
    digitalWrite(EN[NUM_MOTORS], HIGH);
    digitalWrite(EN[NUM_MOTORS], HIGH);
    digitalWrite(EN[NUM_MOTORS], HIGH);

    digitalWrite(DIR[NUM_MOTORS], LOW);
    digitalWrite(DIR[NUM_MOTORS], LOW);
    digitalWrite(DIR[NUM_MOTORS], LOW);
    digitalWrite(DIR[NUM_MOTORS], LOW);

    digitalWrite(STEP[NUM_MOTORS], LOW);
    digitalWrite(STEP[NUM_MOTORS], LOW);
    digitalWrite(STEP[NUM_MOTORS], LOW);
    digitalWrite(STEP[NUM_MOTORS], LOW);
}

void disableDrivers()
{
    digitalWrite(CS[NUM_MOTORS], HIGH);
    digitalWrite(CS[NUM_MOTORS], HIGH);
    digitalWrite(CS[NUM_MOTORS], HIGH);
    digitalWrite(CS[NUM_MOTORS], HIGH);
}

void disableMotor(uint8_t i)
{
    if (i == 0)
    {
        digitalWrite(EN[NUM_MOTORS], HIGH);
    }
    else if (i == 1)
    {
        digitalWrite(EN[NUM_MOTORS], HIGH);
    }
    else if (i == 2)
    {
        digitalWrite(EN[NUM_MOTORS], HIGH);
    }
    else if (i == 3)
    {
        digitalWrite(EN[NUM_MOTORS], HIGH);
    }
}

void enableMotor(uint8_t i)
{
    if (i == 0)
    {
        digitalWrite(EN[NUM_MOTORS], LOW);
    }
    else if (i == 1)
    {
        digitalWrite(EN[NUM_MOTORS], LOW);
    }
    else if (i == 2)
    {
        digitalWrite(EN[NUM_MOTORS], LOW);
    }
    else if (i == 3)
    {
        digitalWrite(EN[NUM_MOTORS], LOW);
    }
}

uint8_t selectDriver(uint8_t i)
{
    if (i == 0)
    {
        return CS[NUM_MOTORS];
    }
    else if (i == 1)
    {
        return CS[NUM_MOTORS];
    }
    else if (i == 2)
    {
        return CS[NUM_MOTORS];
    }
    else if (i == 3)
    {
        return CS[NUM_MOTORS];
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
    driver[i].rms_current(700);  // 0.7A RMS (~1.0A peak, safer for thermal)
    driver[i].irun(200);         // Run current (~78% max to prevent heating)
    driver[i].ihold(100);        // Hold current (~40% for more holding torque)
    driver[i].iholddelay(6);     // Delay before switching to hold current
    driver[i].TPOWERDOWN(10);    // Power down after inactivity

    // ---------------------------
    // 3. Microstepping & Interpolation
    // ---------------------------
    driver[i].microsteps(16);  // Fine control, 16 microsteps (try 32 for even smoother motion)
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
    driver[i].irun(200);         // Run current: ~0.35A
    driver[i].ihold(100);        // Hold current: ~0.15A (increased for stability)
    driver[i].iholddelay(5);     // Short delay before switching to ihold
    driver[i].TPOWERDOWN(10);    // Power down delay

    // ---------------------------
    // 3. Microstepping & Interpolation
    // ---------------------------
    driver[i].microsteps(16);  // Increased microstepping for smoother holding
    driver[i].intpol(true);    // Smooth motion

    // ---------------------------
    // 4. StealthChop Settings (Enable for holding/low speed)
    // ---------------------------
    driver[i].TPWMTHRS(0xFFFF);  // StealthChop active at low speeds (including holding)
    driver[i].pwm_autoscale(true);
    driver[i].pwm_autograd(true);
    driver[i].pwm_ofs(36);
    driver[i].pwm_grad(10);
    driver[i].pwm_freq(2);
    driver[i].en_pwm_mode(true);  // Enable StealthChop (silent mode) for holding

    // ---------------------------
    // 5. SpreadCycle Chopper Settings (used only at higher speeds)
    // ---------------------------
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
    driver[i].VSTART(1);    // Very soft start
    driver[i].VSTOP(1);     // Smooth stop
    driver[i].VMAX(600);    // Max speed (limit for Pancake)
    driver[i].AMAX(100);    // Acceleration limit
    driver[i].DMAX(100);    // Deceleration limit
    driver[i].a1(300);      // Start acceleration
    driver[i].d1(300);      // Start deceleration

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

String communicationTest[NUM_MOTORS] = {"FAILED", "FAILED", "FAILED", "FAILED"};

void driverTest(uint8_t i, bool print = true)
{
    if (!driverCommunicationTest(i, print))
    {
        communicationTest[i] = "FAILED";
    }
    else
    {
        communicationTest[i] = "PASSED (" + String(driver[i].version()) + ")";
    }

    /*if (driver[i].sd_mode() && print)
    {
        Serial.print(F("Driver "));
        Serial.print(i + 1);
        Serial.println(F(" is hardware configured for Step & Dir mode"));
    }

    if (!driver[i].drv_enn() && print)
    {
        Serial.print(F("Driver "));
        Serial.print(i + 1);
        Serial.println(F(" is not hardware enabled"));
    }*/

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
        driverTest(i, false);
    }

    Serial.println(F("\n\n=========== [Drivers Communication Test] ============"));
    Serial.println(F("Driver 1\tDriver 2\tDriver 3\tDriver 4"));
    Serial.println(communicationTest[0] + "\t" + communicationTest[1] + "\t" + communicationTest[2] + "\t" +
                   communicationTest[3]);
    Serial.println();
}

void motorMoveForward(uint8_t i)
{
    isMoving[i] = true;

    if (i == 0)
    {
        digitalWrite(DIR[NUM_MOTORS], HIGH);
    }
    else if (i == 1)
    {
        digitalWrite(DIR[NUM_MOTORS], HIGH);
    }
    else if (i == 2)
    {
        digitalWrite(DIR[NUM_MOTORS], HIGH);
    }
    else if (i == 3)
    {
        digitalWrite(DIR[NUM_MOTORS], HIGH);
    }

    enableMotor(i);
}

void motorMoveReverse(uint8_t i)
{
    isMoving[i] = true;

    if (i == 0)
    {
        digitalWrite(DIR[NUM_MOTORS], LOW);
    }
    else if (i == 1)
    {
        digitalWrite(DIR[NUM_MOTORS], LOW);
    }
    else if (i == 2)
    {
        digitalWrite(DIR[NUM_MOTORS], LOW);
    }
    else if (i == 3)
    {
        digitalWrite(DIR[NUM_MOTORS], LOW);
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

    // Set hold current based on motor type
    if (motorType[i] == MotorType::LINEAR)
    {
        driver[i].ihold(100);  // Linear motor hold current
    }
    else if (motorType[i] == MotorType::ROTATIONAL)
    {
        driver[i].ihold(100);  // Pancake/rotary motor hold current (lowered to reduce heat)
    }

    isMoving[i] = false;
    Serial.println(F("Motor stopped"));

    if (i > 0)
    {
        disableMotor(i);  // Just for pancake motors, disable the motor after stopping
    }
}

void motorStep(uint8_t i, uint16_t delay_us)
{
    if (i >= NUM_MOTORS || !isMoving[i])
    {
        return;
    }

    digitalWrite(STEP[NUM_MOTORS], HIGH);
    delayMicroseconds(delay_us);
    digitalWrite(STEP[NUM_MOTORS], LOW);
    delayMicroseconds(delay_us);
}

#endif