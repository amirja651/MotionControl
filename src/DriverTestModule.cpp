#include "DriverTestModule.h"
#include "ArduinoLog.h"
#include "Config/TMC5160T_Driver.h"
#include "MotorController.h"

DriverTestModule::DriverTestModule(MotorController* motors, uint8_t numMotors) : motors(motors), numMotors(numMotors) {}

bool DriverTestModule::testAllDrivers()
{
    Log.noticeln(F("Starting comprehensive driver test..."));

    // Test 1: Basic Communication
    if (!testBasicCommunication())
    {
        Log.errorln(F("Basic communication test failed"));
        return false;
    }

    // Test 2: Individual Driver Configuration
    if (!testDriverConfiguration())
    {
        Log.errorln(F("Driver configuration test failed"));
        return false;
    }

    // Test 3: Daisy Chain Communication
    if (!testDaisyChainCommunication())
    {
        Log.errorln(F("Daisy chain communication test failed"));
        return false;
    }

    // Test 4: Concurrent Operation
    if (!testConcurrentOperation())
    {
        Log.errorln(F("Concurrent operation test failed"));
        return false;
    }

    Log.noticeln(F("All driver tests completed successfully"));
    return true;
}

bool DriverTestModule::testBasicCommunication()
{
    Log.noticeln(F("Testing basic communication with all drivers..."));

    for (uint8_t i = 0; i < numMotors; i++)
    {
        Log.noticeln(F("Testing driver %d..."), i + 1);

        // Try multiple times to ensure stable communication
        bool success = false;
        for (int attempt = 0; attempt < 3; attempt++)
        {
            if (motors[i].testCommunication())
            {
                success = true;
                break;
            }
            delay(200);
        }

        if (!success)
        {
            Log.errorln(F("Driver %d communication test failed"), i + 1);
            return false;
        }

        uint32_t version = motors[i].driver.version();
        Log.noticeln(F("Driver %d version: 0x%08X"), i + 1, version);

        delay(100);
    }

    return true;
}

bool DriverTestModule::testDriverConfiguration()
{
    Log.noticeln(F("Testing driver configuration..."));

    for (uint8_t i = 0; i < numMotors; i++)
    {
        Log.noticeln(F("Configuring driver %d..."), i + 1);

        // Test GCONF register
        uint32_t gconf = motors[i].driver.GCONF();
        Log.noticeln(F("Driver %d GCONF: 0x%08X"), i + 1, gconf);

        // Test IHOLD_IRUN register
        uint32_t ihold_irun = motors[i].driver.IHOLD_IRUN();
        Log.noticeln(F("Driver %d IHOLD_IRUN: 0x%08X"), i + 1, ihold_irun);

        // Test TPOWERDOWN register
        uint32_t tpowerdown = motors[i].driver.TPOWERDOWN();
        Log.noticeln(F("Driver %d TPOWERDOWN: 0x%08X"), i + 1, tpowerdown);

        delay(100);
    }

    return true;
}

bool DriverTestModule::testDaisyChainCommunication()
{
    Log.noticeln(F("Testing daisy chain communication..."));

    // Test sequential communication
    for (uint8_t i = 0; i < numMotors; i++)
    {
        Log.noticeln(F("Testing daisy chain with driver %d..."), i + 1);

        // Write unique value to each driver
        uint32_t testValue = 0x12345678 + i;
        motors[i].driver.GCONF(testValue);
        delay(50);

        // Read back and verify
        uint32_t readValue = motors[i].driver.GCONF();
        if (readValue != testValue)
        {
            Log.errorln(F("Driver %d daisy chain test failed. Expected: 0x%08X, Got: 0x%08X"), i + 1, testValue, readValue);
            return false;
        }

        Log.noticeln(F("Driver %d daisy chain test passed"), i + 1);
        delay(100);
    }

    return true;
}

bool DriverTestModule::testConcurrentOperation()
{
    Log.noticeln(F("Testing concurrent operation..."));

    // Enable all drivers
    for (uint8_t i = 0; i < numMotors; i++)
    {
        motors[i].enable();
    }

    // Test concurrent communication
    for (int test = 0; test < 5; test++)
    {
        Log.noticeln(F("Concurrent test iteration %d..."), test + 1);

        for (uint8_t i = 0; i < numMotors; i++)
        {
            // Read status from each driver
            uint32_t status = motors[i].driver.DRV_STATUS();
            Log.noticeln(F("Driver %d status: 0x%08X"), i + 1, status);

            // Verify driver is responding
            if (status == 0xFFFFFFFF)
            {
                Log.errorln(F("Driver %d not responding in concurrent test"), i + 1);
                return false;
            }
        }

        delay(100);
    }

    // Disable all drivers
    for (uint8_t i = 0; i < numMotors; i++)
    {
        motors[i].disable();
    }

    return true;
}