#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <TMCStepper.h>

static const uint16_t CS_A = 5;
static const uint16_t CS_B = 4;
static const uint16_t CS_C = 25;
static const uint16_t CS_D = 12;

static const uint16_t W_MOSI = 23;
static const uint16_t W_MISO = 19;
static const uint16_t W_SCK  = 18;

#define NUM_MOTORS 4

// Define driver objects
TMC5160Stepper driver[NUM_MOTORS] = {
    TMC5160Stepper(CS_A, 0.075, W_MOSI, W_MISO, W_SCK), TMC5160Stepper(CS_B, 0.075, W_MOSI, W_MISO, W_SCK),
    TMC5160Stepper(CS_C, 0.075, W_MOSI, W_MISO, W_SCK), TMC5160Stepper(CS_D, 0.075, W_MOSI, W_MISO, W_SCK)};

bool testCommunication(TMC5160Stepper driver, int i, uint8_t csPin)
{
    uint8_t  version  = 0;
    uint32_t gconf    = 0;
    uint32_t status   = 0;
    uint32_t chopconf = 0;

    // Try to read version multiple times
    for (int attempt = 0; attempt < 1; attempt++)
    {
        digitalWrite(csPin, LOW);
        delayMicroseconds(50);
        version = driver.version();
        delayMicroseconds(50);
        digitalWrite(csPin, HIGH);

        if (version != 0xFF && version != 0)
        {
            break;
        }
        delay(100);
    }

    Serial.print(F("Driver "));
    Serial.print(i);
    Serial.print(F(": Version read attempt: 0x"));
    Serial.println(version, HEX);

    if (version == 0xFF || version == 0)
    {
        Serial.print(F("Driver "));
        Serial.print(i);
        Serial.print(F(": Invalid version (0x"));
        Serial.print(version, HEX);
        Serial.println(F(")"));
        return false;
    }

    // Test GCONF register
    digitalWrite(csPin, LOW);
    delayMicroseconds(50);
    gconf = driver.GCONF();
    delayMicroseconds(50);
    digitalWrite(csPin, HIGH);

    Serial.print(F("Driver "));
    Serial.print(i);
    Serial.print(F(": GCONF read: 0x"));
    Serial.println(gconf, HEX);

    if (gconf == 0xFFFFFFFF)
    {
        Serial.print(F("Driver "));
        Serial.print(i);
        Serial.println(F(": GCONF register read failed"));
        return false;
    }

    // Test DRV_STATUS register
    digitalWrite(csPin, LOW);
    delayMicroseconds(50);
    status = driver.DRV_STATUS();
    delayMicroseconds(50);
    digitalWrite(csPin, HIGH);

    Serial.print(F("Driver "));
    Serial.print(i);
    Serial.print(F(": DRV_STATUS read: 0x"));
    Serial.println(status, HEX);

    if (status == 0xFFFFFFFF)
    {
        Serial.print(F("Driver "));
        Serial.print(i);
        Serial.println(F(": DRV_STATUS register read failed"));
        return false;
    }

    // Test CHOPCONF register
    digitalWrite(csPin, LOW);
    delayMicroseconds(50);
    chopconf = driver.CHOPCONF();
    delayMicroseconds(50);
    digitalWrite(csPin, HIGH);

    Serial.print(F("Driver "));
    Serial.print(i);
    Serial.print(F(": CHOPCONF read: 0x"));
    Serial.println(chopconf, HEX);

    if (chopconf == 0xFFFFFFFF)
    {
        Serial.print(F("Driver "));
        Serial.print(i);
        Serial.println(F(": CHOPCONF register read failed"));
        return false;
    }

    // Test if driver is responding to commands
    digitalWrite(csPin, LOW);
    delayMicroseconds(50);
    driver.GCONF(gconf);  // Write back the same value
    delayMicroseconds(50);
    digitalWrite(csPin, HIGH);

    digitalWrite(csPin, LOW);
    delayMicroseconds(50);
    uint32_t readback = driver.GCONF();
    delayMicroseconds(50);
    digitalWrite(csPin, HIGH);

    Serial.print(F("Driver "));
    Serial.print(i);
    Serial.print(F(": GCONF write/read test: Original=0x"));
    Serial.print(gconf, HEX);
    Serial.print(F(", Readback=0x"));
    Serial.println(readback, HEX);

    if (readback != gconf)
    {
        Serial.print(F("Driver "));
        Serial.print(i);
        Serial.println(F(": GCONF register write/read mismatch"));
        return false;
    }

    Serial.print(F("Driver "));
    Serial.print(i);
    Serial.print(F(": Communication test passed (Version: 0x"));
    Serial.print(version, HEX);
    Serial.println(F(")"));
    return true;
}

void test(TMC5160Stepper driver, int i, uint8_t csPin)
{
    if (!testCommunication(driver, i, csPin))
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
        Serial.println(driver.version());

        if (driver.sd_mode())
        {
            Serial.print(F("Driver "));
            Serial.print(i);
            Serial.println(F(" is hardware configured for Step & Dir mode"));
        }

        if (driver.drv_enn())
        {
            Serial.print(F("Driver "));
            Serial.print(i);
            Serial.println(F(" is not hardware enabled"));
        }
    }
    delay(500);
}

void disableDerivers()
{
    digitalWrite(CS_A, HIGH);
    digitalWrite(CS_B, HIGH);
    digitalWrite(CS_C, HIGH);
    digitalWrite(CS_D, HIGH);
}

void DriverPinSetup()
{
    pinMode(CS_A, OUTPUT);
    pinMode(CS_B, OUTPUT);
    pinMode(CS_C, OUTPUT);
    pinMode(CS_D, OUTPUT);
}

void initializeDriversAndTest()
{
    disableDerivers();
    driver[0].begin();
    test(driver[0], 1, CS_A);
    Serial.println(F("--------------------------------"));
    disableDerivers();
    driver[1].begin();
    test(driver[1], 2, CS_B);
    Serial.println(F("--------------------------------"));
    disableDerivers();
    driver[2].begin();
    test(driver[2], 3, CS_C);
    Serial.println(F("--------------------------------"));
    disableDerivers();
    driver[3].begin();
    test(driver[3], 4, CS_D);
    Serial.println(F("--------------------------------"));
}

#endif
