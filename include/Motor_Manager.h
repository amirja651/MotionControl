#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include "MAE3Encoder.h"
#include <TMCStepper.h>

enum class MotorType
{
    ROTATIONAL,
    LINEAR
};

#define NUM_MOTORS 4

// LEDC Configuration for motor control
#define LEDC_TIMER_BIT 10     // 10-bit timer resolution
#define LEDC_BASE_FREQ 10000  // Base frequency in Hz

#define ROTATIONAL_THRESHOLD   0.1f  // Reduced from 0.1f
#define LINEAR_THRESHOLD       0.1f  // Reduced from 0.1f
#define PREDICTIVE_STOP_FACTOR 1.1f  // Reduced from 1.2f for later slowdown

static const float MOTOR_THRESHOLD[NUM_MOTORS] = {LINEAR_THRESHOLD, ROTATIONAL_THRESHOLD, ROTATIONAL_THRESHOLD,
                                                  ROTATIONAL_THRESHOLD};

static MotorType motorType[NUM_MOTORS] = {MotorType::LINEAR, MotorType::ROTATIONAL, MotorType::ROTATIONAL, MotorType::ROTATIONAL};

String communication_test[NUM_MOTORS] = {"FAILED", "FAILED", "FAILED", "FAILED"};

static const uint16_t pMOSI = 23;
static const uint16_t pMISO = 19;
static const uint16_t pSCK  = 18;

static const uint16_t pDIR[NUM_MOTORS]  = {22, 4, 32, 27};
static const uint16_t pSTEP[NUM_MOTORS] = {21, 16, 33, 14};
static const uint16_t pEN[NUM_MOTORS]   = {17, 15, 26, 13};
static const uint16_t pCS[NUM_MOTORS]   = {5, 2, 25, 12};

static const uint8_t LEDC_CHANNEL[NUM_MOTORS] = {0, 1, 2, 3};  // LEDC channel for motor A, B, C, D

// Define driver objects
TMC5160Stepper driver[NUM_MOTORS] = {
    TMC5160Stepper(pCS[0], 0.075, pMOSI, pMISO, pSCK), TMC5160Stepper(pCS[1], 0.075, pMOSI, pMISO, pSCK),
    TMC5160Stepper(pCS[2], 0.075, pMOSI, pMISO, pSCK), TMC5160Stepper(pCS[3], 0.075, pMOSI, pMISO, pSCK)};

void driversPinSetup()
{
    // Setup pins
    for (int8_t index = 0; index < NUM_MOTORS; index++)
    {
        pinMode(pDIR[index], OUTPUT);
        pinMode(pSTEP[index], OUTPUT);
        pinMode(pEN[index], OUTPUT);
        pinMode(pCS[index], OUTPUT);

        digitalWrite(pEN[index], HIGH);
        digitalWrite(pDIR[index], LOW);
        digitalWrite(pSTEP[index], LOW);
    }

    pinMode(MISO, INPUT_PULLUP);
}

void disable_all_drivers()
{
    for (int8_t index = 0; index < NUM_MOTORS; index++)
    {
        digitalWrite(pCS[index], HIGH);
    }
}

uint8_t select_driver(uint8_t index)
{
    return pCS[index];
}

void disable_motor(uint8_t index)
{
    digitalWrite(pEN[index], HIGH);
}

void enable_motor(uint8_t index)
{
    digitalWrite(pEN[index], LOW);
}

void setMotorDirection(uint8_t index, bool dir)  // dir = true (Forward), false (Reverse)
{
    digitalWrite(pDIR[index], dir ? HIGH : LOW);
    // encoders[index].setDirection(dir ? Direction::CLOCKWISE : Direction::COUNTER_CLOCKWISE);
    enable_motor(index);
}

void configure_driver_nema11_1004H(uint8_t index)
{
    if (motorType[index] != MotorType::LINEAR)
    {
        Serial.println("Warning: Wrong linear motor config!");
        return;
    }

    disable_all_drivers();
    delay(1);

    // ---------------------------
    // 1. Basic Driver Configuration (GCONF)
    // ---------------------------
    uint32_t gconf = 0;
    // gconf |= (1 << 0);  // Internal Rsense
    gconf |= (1 << 2);  // StealthChop enable (initially)
    gconf |= (1 << 3);  // Microstep interpolation
    gconf |= (1 << 4);  // Double edge step
    gconf |= (1 << 6);  // Multistep filtering
    driver[index].GCONF(gconf);
    delay(1);

    // ---------------------------
    // 2. Current Settings (Low Power Mode)
    // ---------------------------
    driver[index].rms_current(700);  // 0.7A RMS (safer for motor life)
    driver[index].irun(16);
    driver[index].ihold(8);
    driver[index].iholddelay(8);   // 8 * 16 = 128 ms before going to hold current
    driver[index].TPOWERDOWN(10);  // Power down delay (10 × 100ms) (irun -> ihold)

    // ---------------------------
    // 3. Microstepping & Interpolation
    // ---------------------------
    driver[index].microsteps(16);  // Increased microstepping for smoother holding
    driver[index].intpol(true);    // Smooth motion

    // ---------------------------
    // 4. StealthChop Settings (Enable for holding/low speed)
    // ---------------------------
    driver[index].TPWMTHRS(0xFFFF);  // StealthChop active at low speeds (including holding)
    driver[index].pwm_autoscale(true);
    driver[index].pwm_autograd(true);
    driver[index].pwm_ofs(36);
    driver[index].pwm_grad(10);
    driver[index].pwm_freq(2);
    driver[index].en_pwm_mode(true);  // Enable StealthChop (silent mode) for holding

    // ---------------------------
    // 5. SpreadCycle Chopper Settings (used only at higher speeds)
    // ---------------------------
    // driver[index].toff(4);
    // driver[index].blank_time(24);
    // driver[index].hysteresis_start(3);
    // driver[index].hysteresis_end(1);

    // ---------------------------
    // 6. StallGuard & CoolStep
    // ---------------------------
    // driver[index].TCOOLTHRS(200);  // CoolStep threshold
    // driver[index].sgt(5);          // StallGuard threshold
    // driver[index].sfilt(true);

    // ---------------------------
    // 7. Motion Configuration (Soft Motion)
    // ---------------------------
    driver[index].RAMPMODE(0);  // Positioning mode
    driver[index].VSTART(1);    // Start slowly
    driver[index].VSTOP(1);     // Stop slowly
    driver[index].VMAX(600);    // Maximum speed, suitable for initial testing
    driver[index].AMAX(100);    // Acceleration
    driver[index].DMAX(100);    // Deceleration
    driver[index].a1(300);      // Initial acceleration
    driver[index].d1(300);      // Initial deceleration

    delay(1);
}

void optimizeForPancake(uint8_t index)
{
    if (motorType[index] != MotorType::ROTATIONAL)
    {
        Serial.println("Warning: Wrong rotary motor config!");
        return;
    }

    disable_all_drivers();
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
    driver[index].GCONF(gconf);
    delay(1);

    // ---------------------------
    // 2. Current Settings (Low Power Mode)
    // ---------------------------
    driver[index].rms_current(350);  // About 0.35A RMS (safe for Pancake)
    driver[index].irun(200);         // Run current: ~0.35A
    driver[index].ihold(100);        // Hold current: ~0.15A (increased for stability)
    driver[index].iholddelay(1);     // Short delay before switching to ihold
    driver[index].TPOWERDOWN(10);    // Power down delay

    // ---------------------------
    // 3. Microstepping & Interpolation
    // ---------------------------
    driver[index].microsteps(16);  // Increased microstepping for smoother holding
    driver[index].intpol(true);    // Smooth motion

    // ---------------------------
    // 4. StealthChop Settings (Enable for holding/low speed)
    // ---------------------------
    driver[index].TPWMTHRS(0xFFFF);  // StealthChop active at low speeds (including holding)
    driver[index].pwm_autoscale(true);
    driver[index].pwm_autograd(true);
    driver[index].pwm_ofs(36);
    driver[index].pwm_grad(10);
    driver[index].pwm_freq(2);
    driver[index].en_pwm_mode(true);  // Enable StealthChop (silent mode) for holding

    // ---------------------------
    // 5. SpreadCycle Chopper Settings (used only at higher speeds)
    // ---------------------------
    driver[index].toff(4);
    driver[index].blank_time(24);
    driver[index].hysteresis_start(3);
    driver[index].hysteresis_end(1);

    // ---------------------------
    // 6. StallGuard & CoolStep
    // ---------------------------
    driver[index].TCOOLTHRS(200);  // CoolStep threshold
    driver[index].sgt(5);          // StallGuard threshold
    driver[index].sfilt(true);

    // ---------------------------
    // 7. Motion Configuration (Soft Motion)
    // ---------------------------
    driver[index].RAMPMODE(0);  // Positioning mode
    driver[index].VSTART(1);    // Very soft start
    driver[index].VSTOP(1);     // Smooth stop
    driver[index].VMAX(600);    // Max speed (limit for Pancake)
    driver[index].AMAX(100);    // Acceleration limit
    driver[index].DMAX(100);    // Deceleration limit
    driver[index].a1(300);      // Start acceleration
    driver[index].d1(300);      // Start deceleration

    delay(1);
}

bool driverCommunicationTest(uint8_t index, bool print = true)
{
    disable_all_drivers();

    uint8_t  version  = 0;
    uint32_t gconf    = 0;
    uint32_t status   = 0;
    uint32_t chopconf = 0;

    version = driver[index].version();

    delay(1);

    if (print)
    {
        Serial.print(F("Driver "));
        Serial.print(index + 1);
        Serial.print(F(": Version read attempt: 0x"));
        Serial.println(version, HEX);
    }

    if (version == 0xFF || version == 0)
    {
        if (print)
        {
            Serial.print(F("Driver "));
            Serial.print(index + 1);
            Serial.print(F(": Invalid version (0x"));
            Serial.print(version, HEX);
            Serial.println(F(")"));
        }
        return false;
    }

    // Test GCONF register
    gconf = driver[index].GCONF();

    if (print)
    {
        Serial.print(F("Driver "));
        Serial.print(index + 1);
        Serial.print(F(": GCONF read: 0x"));
        Serial.println(gconf, HEX);
    }

    if (gconf == 0xFFFFFFFF)
    {
        if (print)
        {
            Serial.print(F("Driver "));
            Serial.print(index + 1);
            Serial.println(F(": GCONF register read failed"));
        }
        return false;
    }

    // Test DRV_STATUS register
    status = driver[index].DRV_STATUS();

    if (print)
    {
        Serial.print(F("Driver "));
        Serial.print(index + 1);
        Serial.print(F(": DRV_STATUS read: 0x"));
        Serial.println(status, HEX);
    }

    if (status == 0xFFFFFFFF)
    {
        if (print)
        {
            Serial.print(F("Driver "));
            Serial.print(index + 1);
            Serial.println(F(": DRV_STATUS register read failed"));
        }
        return false;
    }

    // Test CHOPCONF register
    chopconf = driver[index].CHOPCONF();

    if (print)
    {
        Serial.print(F("Driver "));
        Serial.print(index + 1);
        Serial.print(F(": CHOPCONF read: 0x"));
        Serial.println(chopconf, HEX);
    }

    if (chopconf == 0xFFFFFFFF)
    {
        if (print)
        {
            Serial.print(F("Driver "));
            Serial.print(index + 1);
            Serial.println(F(": CHOPCONF register read failed"));
        }
        return false;
    }

    // Test if driver is responding to commands
    driver[index].GCONF(gconf);  // Write back the same value

    uint32_t readback = driver[index].GCONF();

    if (print)
    {
        Serial.print(F("Driver "));
        Serial.print(index + 1);
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
            Serial.print(index + 1);
            Serial.println(F(": GCONF register write/read mismatch"));
        }
        return false;
    }

    if (print)
    {
        Serial.print(F("Driver "));
        Serial.print(index + 1);
        Serial.print(F(": Communication test passed (Version: 0x"));
        Serial.print(version, HEX);
        Serial.println(F(")"));
    }
    return true;
}

void driverTest(uint8_t index, bool print = true)
{
    if (!driverCommunicationTest(index, print))
    {
        communication_test[index] = "FAILED";
    }
    else
    {
        communication_test[index] = "PASSED (" + String(driver[index].version()) + ")";
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

void initializeDriver(uint8_t index)
{
    if (motorType[index] == MotorType::LINEAR)
        configure_driver_nema11_1004H(index);

    else if (motorType[index] == MotorType::ROTATIONAL)
        optimizeForPancake(index);
}

void initializeDriversAndTest()
{
    driversPinSetup();
    delay(1);

    String header = "";
    String body   = "";

    for (uint8_t index = 0; index < NUM_MOTORS; index++)
    {
        disable_all_drivers();
        driver[index].begin();
        driverTest(index, false);

        //  Header
        header += "Driver " + String(index);  // 7c + 1c + 13c = 21c;

        if (index != 3)
            header += "      |      ";

        // Body
        if (communication_test[index] == "FAILED")
            body += communication_test[index] + "               ";  // 6c + 15c = 21c
        else                                                        // 11c + 10c = 21c
        {
            body += communication_test[index] + "          ";
            initializeDriver(index);
        }
    }
    Serial.println("====================== [ Communication Test  ] ========================");
    Serial.println(header);
    Serial.println(body);
    Serial.println();
}

void initializeLEDC()
{
    for (uint8_t index = 0; index < NUM_MOTORS; index++)
    {
        // Configure LEDC timer
        ledcSetup(LEDC_CHANNEL[index], LEDC_BASE_FREQ, LEDC_TIMER_BIT);
        // ledcSetup(0, 1000, 1);  // dummy freq, 1-bit duty (square)

        // Attach LEDC channels to step pins
        ledcAttachPin(pSTEP[index], 0);
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

void updateMotorFrequency(uint8_t index, float error, float target_position, float current_pos)
{
    static float last_freq[NUM_MOTORS] = {0};

    // Calculate base frequency
    float base_freq = calculateFrequencyFromError(error);

    // Calculate distance to target
    float distance_to_target = fabs(target_position - current_pos);
    float stopping_distance  = calculateStoppingDistance(last_freq[index]);

    // Gradual speed reduction
    if (distance_to_target <= (stopping_distance * PREDICTIVE_STOP_FACTOR))
    {
        float reduction_factor = distance_to_target / (stopping_distance * PREDICTIVE_STOP_FACTOR);
        // Change speed reduction algorithm - gentler gradient
        reduction_factor = pow(reduction_factor, 1.5f);  // changed from 0.7 to 1.5
        base_freq        = min(base_freq, last_freq[index] * reduction_factor);
    }

    // Reduce maximum frequency change
    float max_freq_change = 200.0f;  // reduced from 800 to 200

    // Limit frequency change
    if (base_freq > last_freq[index])
    {
        base_freq = min(base_freq, last_freq[index] + max_freq_change);
    }
    else
    {
        base_freq = max(base_freq, last_freq[index] - max_freq_change);
    }

    // Save frequency for next time
    last_freq[index] = base_freq;

    // Apply new frequency
    uint8_t channel = LEDC_CHANNEL[index];
    ledcWriteTone(channel, base_freq);
    uint32_t duty = (1 << LEDC_TIMER_BIT) / 2;
    ledcWrite(channel, duty);

    static String buffer      = "ch: " + String(channel) + " freq: " + String(base_freq) + " duty: " + String(duty) + "ch: ";
    static String last_buffer = " ";
    if (buffer != last_buffer)
    {
        Serial.print(buffer);
        Serial.println();
        last_buffer = buffer;
    }
}

void stopMotorLEDC(uint8_t index)
{
    uint8_t channel = LEDC_CHANNEL[index];

    // Stop PWM output
    ledcWrite(channel, 0);
}

void motorStop(uint8_t index)
{
    // 0. Stop all pulses immediately (very important)
    ledcWriteTone(LEDC_CHANNEL[index], 0);
    delayMicroseconds(100);  // A little pause until the last pulse is finished

    // 1. Final advance with higher current for precise stop
    // set_IHOLD_IRUN(index, 4, 25, 15);  // ≈48 % hold, ≈81 % run, 64 ms ramp

    // 2. If needed, give the final frequency to complete the rotation (e.g. 400 Hz)
    ledcWriteTone(LEDC_CHANNEL[index], 400);
    delay(2);  // How many milliseconds should we give until some real pulses are given

    // 3. Remove all pulses (complete stop)
    ledcWriteTone(LEDC_CHANNEL[index], 0);

    // 4. Wait for the mechanical body to relax
    delayMicroseconds(300);

    // 5. Set the final holding current (for a linear axis about 26% is enough)
    // set_IHOLD_IRUN(index, 4, 20, 8);  // ≈26 % hold, ≈65 % run, 64 ms ramp

    // 6. Disable the output if the motor is rotary
    if (motorType[index] == MotorType::ROTATIONAL)
        disable_motor(index);
}

#endif
