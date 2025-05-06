#ifndef POSITION_STORAGE_H
#define POSITION_STORAGE_H

#include <Arduino.h>
#include <EEPROM.h>

// EEPROM addresses for motor positions
// Each motor position takes 4 bytes (float)
#define EEPROM_SIZE               512  // Total EEPROM size
#define MOTOR_POSITION_START_ADDR 0    // Starting address for motor positions
#define MOTOR_POSITION_SIZE       4    // Size of each position in bytes
#define NUM_MOTORS                4

// Initialize EEPROM
void initPositionStorage()
{
    EEPROM.begin(EEPROM_SIZE);
}

// Save motor position to EEPROM
void saveMotorPosition(uint8_t motorIndex, float position)
{
    if (motorIndex >= NUM_MOTORS)
        return;

    int address = MOTOR_POSITION_START_ADDR + (motorIndex * MOTOR_POSITION_SIZE);
    EEPROM.put(address, position);
    EEPROM.commit();
}

// Get saved motor position from EEPROM
float loadMotorPosition(uint8_t motorIndex)
{
    if (motorIndex >= NUM_MOTORS)
        return 0.0f;

    int   address = MOTOR_POSITION_START_ADDR + (motorIndex * MOTOR_POSITION_SIZE);
    float position;
    EEPROM.get(address, position);
    return position;
}

#endif  // POSITION_STORAGE_H