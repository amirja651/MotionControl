#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <TMCStepper.h>

#define NUM_MOTORS 4

static const uint16_t pDIR[NUM_MOTORS]  = {22, 4, 32, 27};
static const uint16_t pSTEP[NUM_MOTORS] = {21, 16, 33, 14};
static const uint16_t pEN[NUM_MOTORS]   = {17, 15, 26, 13};
static const uint16_t pCS[NUM_MOTORS]   = {5, 2, 25, 12};

static const uint8_t LEDC_CHANNEL[NUM_MOTORS] = {0, 1, 2, 3};  // LEDC channel for motor A, B, C, D

#define ROTATIONAL_THRESHOLD   0.05f  // Reduced from 0.1f
#define LINEAR_THRESHOLD       0.05f  // Reduced from 0.1f
#define PREDICTIVE_STOP_FACTOR 1.1f   // Reduced from 1.2f for later slowdown

static const float MOTOR_THRESHOLD[NUM_MOTORS] = {LINEAR_THRESHOLD, ROTATIONAL_THRESHOLD, ROTATIONAL_THRESHOLD,
                                                  ROTATIONAL_THRESHOLD};

static const uint16_t pMOSI = 23;
static const uint16_t pMISO = 19;
static const uint16_t pSCK  = 18;

String communication_test[NUM_MOTORS] = {"FAILED", "FAILED", "FAILED", "FAILED"};

enum class MotorType
{
    ROTATIONAL,
    LINEAR
};

static MotorType motorType[NUM_MOTORS] = {MotorType::LINEAR, MotorType::ROTATIONAL, MotorType::ROTATIONAL, MotorType::ROTATIONAL};

// Define driver objects
TMC5160Stepper driver[NUM_MOTORS] = {
    TMC5160Stepper(pCS[0], 0.075, pMOSI, pMISO, pSCK), TMC5160Stepper(pCS[1], 0.075, pMOSI, pMISO, pSCK),
    TMC5160Stepper(pCS[2], 0.075, pMOSI, pMISO, pSCK), TMC5160Stepper(pCS[3], 0.075, pMOSI, pMISO, pSCK)};

// LEDC Configuration for motor control
#define LEDC_TIMER_BIT 10     // 10-bit timer resolution
#define LEDC_BASE_FREQ 10000  // Base frequency in Hz

void driversPinSetup()
{
    // Setup pins
    for (int8_t motor_index = 0; motor_index < NUM_MOTORS; motor_index++)
    {
        pinMode(pDIR[motor_index], OUTPUT);
        pinMode(pSTEP[motor_index], OUTPUT);
        pinMode(pEN[motor_index], OUTPUT);
        pinMode(pCS[motor_index], OUTPUT);

        digitalWrite(pEN[motor_index], HIGH);
        digitalWrite(pDIR[motor_index], LOW);
        digitalWrite(pSTEP[motor_index], LOW);
    }

    pinMode(MISO, INPUT_PULLUP);
}

void disableDrivers()
{
    for (int8_t motor_index = 0; motor_index < NUM_MOTORS; motor_index++)
    {
        digitalWrite(pCS[motor_index], HIGH);
    }
}

void disableMotor(uint8_t motor_index)
{
    digitalWrite(pEN[motor_index], HIGH);
}

void enableMotor(uint8_t motor_index)
{
    digitalWrite(pEN[motor_index], LOW);
}

uint8_t selectDriver(uint8_t motor_index)
{
    return pCS[motor_index];
}

void configureDriverNEMA11_1004H(uint8_t i)
{
    if (motorType[i] != MotorType::LINEAR)
    {
        Serial.println("Warning: Wrong motor config!");
        return;
    }

    disableDrivers();

    // ✱ TMC5160 – Minimum configuration for STEP/DIR mode ✱
    uint32_t gconf = 0;
    gconf |= (1 << 0);  // Internal Rsense
    gconf |= (1 << 2);  // Enable StealthChop
    gconf |= (1 << 3);  // Microstep interpolation
    gconf |= (1 << 4);  // Double edge step
    gconf |= (1 << 6);  // Multistep filtering
    driver[i].GCONF(gconf);

    // Current and microstep
    driver[i].rms_current(700);                  // 0.7A RMS (~1.0A peak, safer for thermal)
    uint32_t val32 = (4 << 16) | (20 << 8) | 8;  // (8, 20, 4)
    driver[i].IHOLD_IRUN(val32);                 // // 40% hold, 100% move, 64 ms ramp
    driver[i].microsteps(16);                    // Fine control, 16 microsteps (try 32 for even smoother motion)
    driver[i].intpol(true);                      // Enable interpolation for smooth motion

    // StealthChop / SpreadCycle
    driver[i].en_pwm_mode(true);  // StealthChop at low speed
    driver[i].pwm_autoscale(true);
    driver[i].TPWMTHRS(500);   // Switch threshold
    driver[i].toff(4);         // Chopper off-time
    driver[i].blank_time(24);  // Blank time

    delay(1);
}

// Optimize for pancake motor
void optimizeForPancake(uint8_t i)
{
    if (motorType[i] != MotorType::ROTATIONAL)
    {
        Serial.println("Warning: Wrong motor config!");
        return;
    }

    disableDrivers();
    delay(1);

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
    delay(1);

    // ---------------------------
    // 2. Current Settings (Low Power Mode)
    // ---------------------------
    driver[i].rms_current(350);  // About 0.35A RMS (safe for Pancake)
    driver[i].irun(200);         // Run current: ~0.35A
    driver[i].ihold(100);        // Hold current: ~0.15A (increased for stability)
    driver[i].iholddelay(1);     // Short delay before switching to ihold
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

    delay(1);
}

bool driverCommunicationTest(uint8_t i, bool print = true)
{
    disableDrivers();

    uint8_t  version  = 0;
    uint32_t gconf    = 0;
    uint32_t status   = 0;
    uint32_t chopconf = 0;

    version = driver[i].version();

    delay(1);

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

void driverTest(uint8_t i, bool print = true)
{
    if (!driverCommunicationTest(i, print))
    {
        communication_test[i] = "FAILED";
    }
    else
    {
        communication_test[i] = "PASSED (" + String(driver[i].version()) + ")";
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

    delay(100);
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
}

void initializeDriversAndTest()
{
    driversPinSetup();
    delay(1);

    for (uint8_t motor_index = 0; motor_index < NUM_MOTORS; motor_index++)
    {
        disableDrivers();
        driver[motor_index].begin();
        driverTest(motor_index, false);
    }

    Serial.println(F("\n\n=========== [Drivers Communication Test] ============"));
    Serial.println(F("Driver 1\tDriver 2\tDriver 3\tDriver 4"));
    Serial.println(communication_test[0] + "\t" + communication_test[1] + "\t" + communication_test[2] + "\t" +
                   communication_test[3]);
    Serial.println();
}

void set_motor_direction(uint8_t motor_index, bool dir)  // dir = true (Forward), false (Reverse)
{
    digitalWrite(pDIR[motor_index], dir);
    enableMotor(motor_index);
}

void motorStop(uint8_t motor_index)
{
    static const uint8_t IHOLDDELAY   = 4;   // 64 ms ramp
    static const uint8_t IRUN_FINAL   = 25;  // ≈125% final approach current
    static const uint8_t IHOLD_LINEAR = 15;  // ≈50% holding current

    disableDrivers();

    /* 1. Final approach with increased current */
    uint32_t val32 = (IHOLDDELAY << 16) | (IRUN_FINAL << 8) | IHOLD_LINEAR;
    driver[motor_index].IHOLD_IRUN(val32);

    /* 2. Quick stop with high frequency */
    ledcWriteTone(LEDC_CHANNEL[motor_index], 400);  // Use 400Hz for final stop
    delayMicroseconds(100);

    /* 3. Stop pulses */
    ledcWriteTone(LEDC_CHANNEL[motor_index], 0);

    /* 4. Wait for mechanical settling */
    delayMicroseconds(300);

    /* 5. Set final holding current */
    val32 = (IHOLDDELAY << 16) | (20 << 8) | IHOLD_LINEAR;
    driver[motor_index].IHOLD_IRUN(val32);

    /* 6. Disable output for rotary motors if needed */
    if (motorType[motor_index] == MotorType::ROTATIONAL)
        disableMotor(motor_index);

    Serial.println(F("Motor Stop"));
}

void initializeLEDC()
{
    for (uint8_t motor_index = 0; motor_index < NUM_MOTORS; motor_index++)
    {
        // Configure LEDC timer
        ledcSetup(LEDC_CHANNEL[motor_index], LEDC_BASE_FREQ, LEDC_TIMER_BIT);

        // Attach LEDC channels to step pins
        ledcAttachPin(pSTEP[motor_index], LEDC_CHANNEL[motor_index]);
    }
}

float calculateFrequencyFromError(float error)
{
    // Convert error to absolute value for comparison
    float abs_error = fabs(error);

    // Define frequency ranges (Hz) for different error thresholds
    if (abs_error <= 0.2f)
        return 200;  // Minimum reliable speed - 200 Hz
    else if (abs_error <= 0.5f)
        return 300;  // Slow - 300 Hz
    else if (abs_error <= 1.0f)
        return 500;  // Moderate slow - 500 Hz
    else if (abs_error <= 2.0f)
        return 1000;  // Moderate - 1 kHz
    else if (abs_error <= 5.0f)
        return 2000;  // Moderate fast - 2 kHz
    else if (abs_error <= 10.0f)
        return 4000;  // Fast - 4 kHz
    else if (abs_error <= 20.0f)
        return 6000;  // Very fast - 6 kHz
    else
        return 8000;  // Maximum speed - 8 kHz
}

float calculateStoppingDistance(float current_freq)
{
    // Reduced stopping distances for more precise control
    if (current_freq <= 25)
        return 0.02f;
    else if (current_freq <= 50)
        return 0.05f;
    else if (current_freq <= 100)
        return 0.08f;
    else if (current_freq <= 200)
        return 0.1f;
    else if (current_freq <= 500)
        return 0.15f;
    else if (current_freq <= 1000)
        return 0.2f;
    else if (current_freq <= 2000)
        return 0.3f;
    else if (current_freq <= 5000)
        return 0.4f;
    else
        return 0.5f;
}

void updateMotorFrequency(uint8_t motor_index, float error, float target_position, float current_pos)
{
    static float last_freq[NUM_MOTORS] = {0};
    float        abs_error             = fabs(error);

    // محاسبه فرکانس پایه
    float base_freq = calculateFrequencyFromError(error);

    // محاسبه فاصله تا هدف
    float distance_to_target = fabs(target_position - current_pos);
    float stopping_distance  = calculateStoppingDistance(last_freq[motor_index]);

    // کاهش تدریجی سرعت
    if (distance_to_target <= (stopping_distance * PREDICTIVE_STOP_FACTOR))
    {
        float reduction_factor = distance_to_target / (stopping_distance * PREDICTIVE_STOP_FACTOR);
        // تغییر الگوریتم کاهش سرعت - شیب ملایم‌تر
        reduction_factor = pow(reduction_factor, 1.5f);  // از 0.7 به 1.5 تغییر کرده
        base_freq        = min(base_freq, last_freq[motor_index] * reduction_factor);
    }

    // کاهش حداکثر تغییرات فرکانس
    float max_freq_change = 200.0f;  // از 800 به 200 کاهش یافته

    // محدود کردن تغییرات فرکانس
    if (base_freq > last_freq[motor_index])
    {
        base_freq = min(base_freq, last_freq[motor_index] + max_freq_change);
    }
    else
    {
        base_freq = max(base_freq, last_freq[motor_index] - max_freq_change);
    }

    // ذخیره فرکانس برای دفعه بعد
    last_freq[motor_index] = base_freq;

    // اعمال فرکانس جدید
    uint8_t channel = LEDC_CHANNEL[motor_index];
    ledcWriteTone(channel, base_freq);
    ledcWrite(channel, (1 << LEDC_TIMER_BIT) / 2);
}

void stopMotorLEDC(uint8_t motor_index)
{
    uint8_t channel = LEDC_CHANNEL[motor_index];

    // Stop PWM output
    ledcWrite(channel, 0);

    Serial.println(F("Motor Stop"));
}

#endif