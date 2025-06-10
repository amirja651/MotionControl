#include "main.h"

void setup()
{
    SPI.begin();
    Serial.begin(115200);
    esp_task_wdt_init(10, true);  // 10 seconds timeout, panic reset enabled
    esp_task_wdt_add(NULL);       // Add the current task (setup)
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    delay(1000);
    while (!Serial)
    {
        delay(10);
    }

    printSystemInfo();
    initializeCLI();
    // initPositionStorage();  // Initialize EEPROM
    initializeDriversAndTest();
    initializeLEDC();  // Initialize LEDC channels

    for (int8_t index = 0; index < NUM_MOTORS; index++)
    {
        if (communication_test[index] != "FAILED")
            encoders[index].begin();
    }

    xTaskCreatePinnedToCore(encoderUpdateTask, "EncoderUpdateTask", 2048, NULL, 5, &encoderUpdateTaskHandle, 1);  // Core 1
    xTaskCreatePinnedToCore(motorUpdateTask, "MotorUpdateTask", 2048, NULL, 3, &motorUpdateTaskHandle, 1);        // Core 0
    xTaskCreatePinnedToCore(serialReadTask, "SerialReadTask", 2048, NULL, 2, &serialReadTaskHandle, 0);           // Core 0
    xTaskCreatePinnedToCore(serialPrintTask, "SerialPrintTask", 2048, NULL, 1, &serialPrintTaskHandle, 0);        // Core 0

    esp_task_wdt_add(encoderUpdateTaskHandle);  // Register with WDT
    esp_task_wdt_add(motorUpdateTaskHandle);    // Register with WDT
    esp_task_wdt_add(serialReadTaskHandle);     // Register with WDT
    esp_task_wdt_add(serialPrintTaskHandle);    // Register with WDT
}

void loop()
{
    esp_task_wdt_reset();
    vTaskDelay(100);  // Add a short delay to prevent WDT reset
}

void encoderUpdateTask(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(1);
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        if (communication_test[getMotorIndex()] == "FAILED")
        {
            Serial.println(F("ERROR: Motor communication failed!"));

            esp_task_wdt_reset();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        for (int8_t index = 0; index < NUM_MOTORS; index++)
        {
            if (index != getMotorIndex())
            {
                if (encoders[index].isEnabled())
                    encoders[index].disable();  // Other encoder
            }
            else
            {
                if (encoders[index].isDisabled())
                    encoders[index].enable();  // current encoder
            }
        }

        encoders[getMotorIndex()].processPWM();
        // printSerial();
        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

float wrapAngle180(float value)
{
    if (value > 180.0f)
        value -= 360.0f;
    else if (value < -180.0f)
        value += 360.0f;

    return value;
}

float calculateSignedPositionError(float current_pos)
{
    float target_pos = getTarget();
    float error      = target_pos - current_pos;
    return error;
}

void motorUpdateTask(void* pvParameters)
{
    const uint8_t    MOTOR_UPDATE_TIME = 4;
    const TickType_t xFrequency        = pdMS_TO_TICKS(MOTOR_UPDATE_TIME);
    TickType_t       xLastWakeTime     = xTaskGetTickCount();

    while (1)
    {
        if (!command_received[getMotorIndex()])
        {
            esp_task_wdt_reset();

            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        if (communication_test[getMotorIndex()] == "FAILED")
        {
            Serial.println(F("ERROR: Motor communication failed!"));
            esp_task_wdt_reset();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // Get current position and calculate error
        int32_t lap_id = encoders[getMotorIndex()].gLap().id;

        float current_pos = (motorType[getMotorIndex()] == MotorType::ROTATIONAL)
                                ? encoders[getMotorIndex()].getPositionDegrees(lap_id)
                                : encoders[getMotorIndex()].getTotalTravelUM(lap_id);

        float error = calculateSignedPositionError(current_pos);

        if (motorType[getMotorIndex()] == MotorType::ROTATIONAL)
            error = wrapAngle180(error);

        float target = getTarget();

        // Motor control based on error threshold
        if (isVeryShortDistance || abs(error) > MOTOR_THRESHOLD[getMotorIndex()])
        {
            isVeryShortDistance = false;

            // Set direction based on error sign
            set_motor_direction(getMotorIndex(), error > 0);

            // Update frequency based on error and target position
            updateMotorFrequency(getMotorIndex(), error, target, current_pos);
        }
        else
        {
            // Target reached - stop motor
            motorStopAndSavePosition();
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void serialReadTask(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(100);
    TickType_t       xLastWakeTime = xTaskGetTickCount();
    String           inputBuffer   = "";
    String           lastInput     = "";

    while (1)
    {
        while (Serial.available())
        {
            char c = Serial.read();

            // Handle escape sequences for arrow keys
            static int escState = 0;  // 0: normal, 1: got '\x1b', 2: got '['
            if (escState == 0 && c == '\x1b')
            {
                escState = 1;
                esp_task_wdt_reset();
                continue;
            }
            if (escState == 1 && c == '[')
            {
                escState = 2;
                esp_task_wdt_reset();
                continue;
            }
            if (escState == 2)
            {
                if (c == 'A')
                {  // Up arrow
                    if (historyCount > 0)
                    {
                        if (historyIndex < historyCount - 1)
                            historyIndex++;
                        inputBuffer = commandHistory[(historyCount - 1 - historyIndex) % HISTORY_SIZE];
                        // Clear current line and print inputBuffer
                        Serial.print(F("\r> "));
                        Serial.print(inputBuffer);
                        Serial.print(F("           \r> "));  // Overwrite any old chars
                        Serial.print(inputBuffer);
                    }
                    escState = 0;
                    esp_task_wdt_reset();
                    continue;
                }
                else if (c == 'B')
                {  // Down arrow
                    if (historyCount > 0 && historyIndex > 0)
                    {
                        historyIndex--;
                        inputBuffer = commandHistory[(historyCount - 1 - historyIndex) % HISTORY_SIZE];
                        Serial.print(F("\r> "));
                        Serial.print(inputBuffer);
                        Serial.print(F("           \r> "));
                        Serial.print(inputBuffer);
                    }
                    else if (historyIndex == 0)
                    {
                        historyIndex = -1;
                        inputBuffer  = lastInput;
                        Serial.print(F("\r> "));
                        Serial.print(inputBuffer);
                        Serial.print(F("           \r> "));
                        Serial.print(inputBuffer);
                    }
                    escState = 0;
                    esp_task_wdt_reset();
                    continue;
                }
                escState = 0;
                esp_task_wdt_reset();
                continue;
            }

            // Handle Enter
            if (c == '\n')
            {
                if (inputBuffer.length() > 0)
                {
                    Serial.println();
                    Serial.print(F("# "));
                    Serial.println(inputBuffer.c_str());
                    cli.parse(inputBuffer);
                    // Add to history
                    if (historyCount == 0 || commandHistory[(historyCount - 1) % HISTORY_SIZE] != inputBuffer)
                    {
                        commandHistory[historyCount % HISTORY_SIZE] = inputBuffer;
                        if (historyCount < HISTORY_SIZE)
                            historyCount++;
                        else
                            historyCount = HISTORY_SIZE;
                    }
                    historyIndex = -1;
                    lastInput    = "";
                    inputBuffer  = "";  // Clear the buffer
                }
            }
            else if (c == '\b' || c == 127)  // Handle backspace
            {
                if (inputBuffer.length() > 0)
                {
                    inputBuffer.remove(inputBuffer.length() - 1);
                    Serial.print(F("\b \b"));
                }
            }
            else if (c == 'i')
            {
                // Manual step right (forward)
                set_motor_direction(getMotorIndex(), true);
                // motorStep();
            }
            else if (c == 'k')
            {
                // Manual step left (reverse)
                set_motor_direction(getMotorIndex(), false);
                // motorStep();
            }
            else
            {
                inputBuffer += c;  // Add character to buffer
                Serial.print(c);
                if (historyIndex == -1)
                    lastInput = inputBuffer;
            }
            esp_task_wdt_reset();
        }

        if (cli.available())
        {
            Command c = cli.getCmd();

            // Handle motor commands
            if (c == cmdMotor)
            {
                // Get motor number
                if (c.getArgument("n").isSet())
                {
                    String motorNumStr = c.getArgument("n").getValue();

                    if (!validationInputAndSetMotorIndex(motorNumStr))
                    {
                        esp_task_wdt_reset();
                        continue;
                    }
                }
                else
                {
                    Serial.println(errorMotorNumberIsRequired);
                    esp_task_wdt_reset();
                    continue;
                }

                // Handle Get motor position command
                if (c.getArgument("c").isSet())
                {
                    if (!is_set_motor_number)
                    {
                        Serial.println(errorMotorNumberIsRequired);
                        esp_task_wdt_reset();
                        continue;
                    }

                    is_set_motor_number = false;
                    int32_t lap_id      = encoders[getMotorIndex()].gLap().id;

                    Serial.print(F("*"));
                    Serial.print(getMotorIndex() + 1);
                    Serial.print(F("#"));

                    if (motorType[getMotorIndex()] == MotorType::ROTATIONAL)
                    {
                        Serial.print(encoders[getMotorIndex()].getPositionDegrees(lap_id));
                    }
                    else
                    {
                        Serial.print(encoders[getMotorIndex()].getTotalTravelUM(lap_id));
                    }

                    Serial.println(F("#"));
                    esp_task_wdt_reset();
                    continue;
                }

                // Handle stop command
                if (c.getArgument("s").isSet())
                {
                    if (!is_set_motor_number)
                    {
                        Serial.println(errorMotorNumberIsRequired);
                        esp_task_wdt_reset();
                        continue;
                    }

                    is_set_motor_number = false;

                    resetAllCommandReceivedFlags();
                    motorStop(getMotorIndex());

                    Serial.print(F("Motor "));
                    Serial.print(getMotorIndex() + 1);
                    Serial.println(F(" stopped"));
                    esp_task_wdt_reset();
                    continue;
                }

                // Handle position commands
                if (c.getArgument("p").isSet())
                {
                    if (!is_set_motor_number)
                    {
                        Serial.println(errorMotorNumberIsRequired);
                        esp_task_wdt_reset();
                        continue;
                    }

                    is_set_motor_number = false;

                    String targetStr = c.getArgument("p").getValue();

                    if (!validateAndSetTargetPosition(targetStr))
                    {
                        esp_task_wdt_reset();
                        continue;
                    }
                }

                // Get motor load position
                if (c.getArgument("l").isSet())
                {
                    if (!is_set_motor_number)
                    {
                        Serial.println(errorMotorNumberIsRequired);
                        esp_task_wdt_reset();
                        continue;
                    }

                    is_set_motor_number = false;

                    String targetStr = c.getArgument("l").getValue();

                    esp_task_wdt_reset();
                    continue;
                }
            }
            else if (c == cmdHelp)
            {
                Serial.print(F("Help:"));
                Serial.println(cli.toString());
            }
            else if (c == cmdRestart)
            {
                resetAllCommandReceivedFlags();
                Serial.println(F("System restarting..."));

                // Ensure motors are stopped before restart
                for (int8_t index = 0; index < NUM_MOTORS; index++)
                {
                    if (communication_test[index] != "FAILED")
                    {
                        stopMotorLEDC(index);
                        motorStop(index);
                        command_received[index] = false;  // Reset command flag when target is reached or no command
                        Serial.print(F("Motor "));
                        Serial.print(index);
                        Serial.println(F("Stop"));
                    }
                }

                delay(1000);
                // Give time for motors to stop
                ESP.restart();
            }
        }

        if (cli.errored())
        {
            CommandError cmdError = cli.getError();

            Serial.print(F("ERROR: "));
            Serial.println(cmdError.toString());
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void serialPrintTask(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(300);
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        printSerial();
        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void motorStopAndSavePosition()
{
    stopMotorLEDC(getMotorIndex());
    motorStop(getMotorIndex());
    command_received[getMotorIndex()] = false;  // Reset command flag when target is reached or no command
    Serial.println(F("Motor Stop"));

    if (0)
    {
        int32_t lap_id = encoders[getMotorIndex()].gLap().id;

        float currentPosition = motorType[getMotorIndex()] == MotorType::LINEAR
                                    ? encoders[getMotorIndex()].getTotalTravelUM(lap_id)
                                    : encoders[getMotorIndex()].getPositionDegrees(lap_id);
        saveMotorPosition(getMotorIndex(), currentPosition);  // Save the final position to EEPROM when motor stops
    }
}

bool validationInputAndSetMotorIndex(String motorNumber)
{
    if (motorNumber.length() == 0)
    {
        Serial.println(errorMotorNumberIsRequired);
        return false;
    }

    int motor_number = motorNumber.toInt() - 1;  // Convert to 0-based index

    if (motor_number < 0 || motor_number >= NUM_MOTORS)
    {
        Serial.println(errorMotorNumberIsInvalid);
        return false;
    }

    is_set_motor_number = true;
    setMotorIndex(motor_number);

    return true;
}

void resetAllCommandReceivedFlags()
{
    for (int8_t index = 0; index < NUM_MOTORS; index++)
        command_received[index] = false;
}

bool validateAndSetTargetPosition(String targetStr)
{
    if (targetStr.length() == 0)
    {
        Serial.println(F("ERROR: Position (-p) requires a value"));
        return false;
    }

    float position = targetStr.toFloat();

    Serial.print(F("Motor "));
    Serial.print(getMotorIndex() + 1);
    Serial.print(F(" > new position is: "));
    Serial.print(position);
    Serial.println(motorType[getMotorIndex()] == MotorType::LINEAR ? F(" um") : F(" °"));

    int32_t lap_id = encoders[getMotorIndex()].gLap().id;

    float current_pos = motorType[getMotorIndex()] == MotorType::ROTATIONAL ? encoders[getMotorIndex()].getPositionDegrees(lap_id)
                                                                            : encoders[getMotorIndex()].getTotalTravelUM(lap_id);

    float error = calculateSignedPositionError(current_pos);

    if (motorType[getMotorIndex()] == MotorType::ROTATIONAL)
        error = wrapAngle180(error);

    if (fabs(error) <= 0.01f)
        isVeryShortDistance = true;

    resetAllCommandReceivedFlags();
    setTarget(position);
    command_received[getMotorIndex()] = true;  // Set flag only after valid set target command
    return true;
}

void printSerial()
{
    int32_t current_pulse = encoders[getMotorIndex()].gState().current_pulse;

    int32_t lap_id     = encoders[getMotorIndex()].gLap().id;
    float   lap_period = encoders[getMotorIndex()].getPeriod(lap_id);

    float average_period = encoders[getMotorIndex()].getAveragePeriod(lap_id);

    float degrees_per_pulse = encoders[getMotorIndex()].getDegreesPerPulse(lap_id);
    float position_degrees  = encoders[getMotorIndex()].getPositionDegrees(lap_id);

    float mm_per_pulse    = encoders[getMotorIndex()].getMMPerPulse(lap_id);
    float position_mm     = encoders[getMotorIndex()].getPositionMM(lap_id);
    float total_travel_mm = encoders[getMotorIndex()].getTotalTravelMM(lap_id);
    float total_travel_um = encoders[getMotorIndex()].getTotalTravelUM(lap_id);

    if (fabs(current_pulse - last_pulse[getMotorIndex()]) > 1)
    {
        //  table header
        Serial.print(F("MOT\tCPL\tLAP\tPRD\tAVG\tDPP\tPDG\t\tMPP\tPMM\tTTM\tTTU\n"));

        // Format all values into the buffer
        Serial.print((getMotorIndex() + 1));
        Serial.print(F("\t"));

        Serial.print(current_pulse);
        Serial.print(F("\t"));

        Serial.print(lap_id);
        Serial.print(F("\t"));
        Serial.print(lap_period);
        Serial.print(F("\t"));

        Serial.print(average_period);
        Serial.print(F("\t"));

        Serial.print(degrees_per_pulse);
        Serial.print(F("\t"));
        Serial.print(position_degrees);
        Serial.print(F("\t\t"));

        Serial.print(mm_per_pulse);
        Serial.print(F("\t"));
        Serial.print(position_mm);
        Serial.print(F("\t"));
        Serial.print(total_travel_mm);
        Serial.print(F("\t"));
        Serial.print(total_travel_um);

        Serial.println("\n");

        last_pulse[getMotorIndex()] = encoders[getMotorIndex()].gState().current_pulse;
    }
}