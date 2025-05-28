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

    encoders[0].begin();
    encoders[1].begin();
    encoders[2].begin();
    encoders[3].begin();

    xTaskCreatePinnedToCore(encoderUpdateTask, "EncoderUpdateTask", 2048, NULL, 5, &encoderUpdateTaskHandle, 1);  // Core 1
    xTaskCreatePinnedToCore(motorUpdateTask, "MotorUpdateTask", 2048, NULL, 3, &motorUpdateTaskHandle, 1);        // Core 0
    xTaskCreatePinnedToCore(serialReadTask, "SerialReadTask", 2048, NULL, 2, &serialReadTaskHandle, 0);           // Core 0
    xTaskCreatePinnedToCore(serialPrintTask, "SerialPrintTask", 2048, NULL, 1, &serialPrintTaskHandle, 0);        // Core 0

    esp_task_wdt_add(encoderUpdateTaskHandle);  // Register with WDT
    esp_task_wdt_add(motorUpdateTaskHandle);    // Register with WDT
    esp_task_wdt_add(serialReadTaskHandle);     // Register with WDT
    esp_task_wdt_add(serialPrintTaskHandle);    // Register with WDT

    // clearScreen();
}

void loop()
{
    esp_task_wdt_reset();
    vTaskDelay(100);  // Add a short delay to prevent WDT reset
}

void encoderUpdateTask(void* pvParameters)
{
    const uint8_t    ENCODER_UPDATE_TIME = 4;
    const TickType_t xFrequency          = pdMS_TO_TICKS(ENCODER_UPDATE_TIME);
    TickType_t       xLastWakeTime       = xTaskGetTickCount();

    while (1)
    {
        uint8_t motorIndex = getMotorIndex();

        for (int i = 0; i < NUM_MOTORS; i++)
        {
            if (i != motorIndex && encoders[i].isEnabled())
            {
                encoders[i].disable();
            }
        }

        if (!encoders[motorIndex].isEnabled())
        {
            encoders[motorIndex].enable();
        }

        encoders[motorIndex].processPWM();

        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void motorUpdateTask(void* pvParameters)
{
    const uint8_t    MOTOR_UPDATE_TIME = 4;
    const TickType_t xFrequency        = pdMS_TO_TICKS(MOTOR_UPDATE_TIME);
    TickType_t       xLastWakeTime     = xTaskGetTickCount();

    while (1)
    {
        uint8_t motorIndex = getMotorIndex();

        if (!command_received[motorIndex])
        {
            esp_task_wdt_reset();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        if (communicationTest[motorIndex] == "FAILED")
        {
            Serial.println(F("ERROR: Motor communication failed!"));
            esp_task_wdt_reset();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        if (motorType[motorIndex] == MotorType::ROTATIONAL)
        {
            rotationalMotorUpdate();
        }
        else
        {
            linearMotorUpdate();
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void serialReadTask(void* pvParameters)
{
    const uint8_t    SERIAL_READ_TIME = 100;
    const TickType_t xFrequency       = pdMS_TO_TICKS(SERIAL_READ_TIME);
    TickType_t       xLastWakeTime    = xTaskGetTickCount();
    String           inputBuffer      = "";
    String           lastInput        = "";

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
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }
            if (escState == 1 && c == '[')
            {
                escState = 2;
                esp_task_wdt_reset();
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
                    vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
                    vTaskDelayUntil(&xLastWakeTime, xFrequency);
                    continue;
                }
                escState = 0;
                esp_task_wdt_reset();
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
                motorMoveForward(getMotorIndex());
                motorStep(getMotorIndex(), 10);
            }
            else if (c == 'k')
            {
                // Manual step left (reverse)
                motorMoveReverse(getMotorIndex());
                motorStep(getMotorIndex(), 10);
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
                        vTaskDelayUntil(&xLastWakeTime, xFrequency);
                        continue;
                    }
                }
                else
                {
                    Serial.println(errorMotorNumberIsRequired);
                    esp_task_wdt_reset();
                    vTaskDelayUntil(&xLastWakeTime, xFrequency);
                    continue;
                }

                // Handle Get motor position command
                if (c.getArgument("c").isSet())
                {
                    if (!is_set_motor_number)
                    {
                        Serial.println(errorMotorNumberIsRequired);
                        esp_task_wdt_reset();
                        vTaskDelayUntil(&xLastWakeTime, xFrequency);
                        continue;
                    }

                    is_set_motor_number = false;
                    uint8_t motorIndex  = getMotorIndex();

                    Serial.print(F("*"));
                    Serial.print(motorIndex + 1);
                    Serial.print(F("#"));

                    if (motorType[motorIndex] == MotorType::ROTATIONAL)
                    {
                        Serial.print(encoders[motorIndex].getPositionDegrees());
                    }
                    else
                    {
                        Serial.print(encoders[motorIndex].getPositionUM());
                    }

                    Serial.println(F("#"));
                    esp_task_wdt_reset();
                    vTaskDelayUntil(&xLastWakeTime, xFrequency);
                    continue;
                }

                // Handle stop command
                if (c.getArgument("s").isSet())
                {
                    if (!is_set_motor_number)
                    {
                        Serial.println(errorMotorNumberIsRequired);
                        esp_task_wdt_reset();
                        vTaskDelayUntil(&xLastWakeTime, xFrequency);
                        continue;
                    }

                    is_set_motor_number = false;
                    uint8_t motorIndex  = getMotorIndex();

                    resetCommandReceived();
                    motorStop(motorIndex);

                    Serial.print(F("Motor "));
                    Serial.print(motorIndex + 1);
                    Serial.println(F(" stopped"));
                    esp_task_wdt_reset();
                    vTaskDelayUntil(&xLastWakeTime, xFrequency);
                    continue;
                }

                // Handle offset command
                if (c.getArgument("o").isSet())
                {
                    if (!is_set_motor_number)
                    {
                        Serial.println(errorMotorNumberIsRequired);
                        esp_task_wdt_reset();
                        vTaskDelayUntil(&xLastWakeTime, xFrequency);
                        continue;
                    }

                    is_set_motor_number = false;
                    uint8_t motorIndex  = getMotorIndex();

                    // Only allow for Motor 1
                    if (motorType[motorIndex] != MotorType::LINEAR)
                    {
                        Serial.println(errorTheCommandIsOnlyValidForLinearMotors);
                        esp_task_wdt_reset();
                        vTaskDelayUntil(&xLastWakeTime, xFrequency);
                        continue;
                    }

                    String linearOffsetStr = c.getArgument("o").getValue();

                    if (!validationInputAndSetLinearOffset(linearOffsetStr))
                    {
                        esp_task_wdt_reset();
                        vTaskDelayUntil(&xLastWakeTime, xFrequency);
                        continue;
                    }
                }

                // Handle lower limit command
                if (c.getArgument("lo").isSet())
                {
                    if (!is_set_motor_number)
                    {
                        Serial.println(errorMotorNumberIsRequired);
                        esp_task_wdt_reset();
                        vTaskDelayUntil(&xLastWakeTime, xFrequency);
                        continue;
                    }

                    is_set_motor_number = false;
                    uint8_t motorIndex  = getMotorIndex();

                    // Only allow for Motor 1
                    if (motorType[motorIndex] != MotorType::LINEAR)
                    {
                        Serial.println(errorTheCommandIsOnlyValidForLinearMotors);
                        esp_task_wdt_reset();
                        vTaskDelayUntil(&xLastWakeTime, xFrequency);
                        continue;
                    }

                    String lowerLimitPx = c.getArgument("lo").getValue();

                    if (!validationInputAndSetLinearLowerLimit(lowerLimitPx))
                    {
                        esp_task_wdt_reset();
                        vTaskDelayUntil(&xLastWakeTime, xFrequency);
                        continue;
                    }
                }

                // Handle upper limit command
                if (c.getArgument("up").isSet())
                {
                    if (!is_set_motor_number)
                    {
                        Serial.println(errorMotorNumberIsRequired);
                        esp_task_wdt_reset();
                        vTaskDelayUntil(&xLastWakeTime, xFrequency);
                        continue;
                    }

                    is_set_motor_number = false;
                    uint8_t motorIndex  = getMotorIndex();

                    // Only allow for Motor 1
                    if (motorType[motorIndex] != MotorType::LINEAR)
                    {
                        Serial.println(errorTheCommandIsOnlyValidForLinearMotors);
                        esp_task_wdt_reset();
                        vTaskDelayUntil(&xLastWakeTime, xFrequency);
                        continue;
                    }

                    String upperLimitPx = c.getArgument("up").getValue();

                    if (!validationInputAndSetLinearUpperLimit(upperLimitPx))
                    {
                        esp_task_wdt_reset();
                        vTaskDelayUntil(&xLastWakeTime, xFrequency);
                        continue;
                    }
                }

                // Handle position commands
                if (c.getArgument("p").isSet())
                {
                    if (!is_set_motor_number)
                    {
                        Serial.println(errorMotorNumberIsRequired);
                        esp_task_wdt_reset();
                        vTaskDelayUntil(&xLastWakeTime, xFrequency);
                        continue;
                    }

                    is_set_motor_number = false;

                    String targetStr = c.getArgument("p").getValue();

                    if (!validationInputAndSetTarget(targetStr))
                    {
                        esp_task_wdt_reset();
                        vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
                        vTaskDelayUntil(&xLastWakeTime, xFrequency);
                        continue;
                    }

                    is_set_motor_number = false;

                    String targetStr = c.getArgument("l").getValue();

                    esp_task_wdt_reset();
                    vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
                resetCommandReceived();
                Serial.println(F("System restarting..."));

                // Ensure motors are stopped before restart
                for (int i = 0; i < NUM_MOTORS; i++)
                {
                    motorStop(i);
                }

                delay(100);  // Give time for motors to stop
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
    const uint16_t   SERIAL_PRINT_TIME = 300;
    const TickType_t xFrequency        = pdMS_TO_TICKS(SERIAL_PRINT_TIME);
    TickType_t       xLastWakeTime     = xTaskGetTickCount();

    while (1)
    {
        printSerial();
        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void motorStopAndSavePosition()
{
    uint8_t motorIndex = getMotorIndex();

    motorLastState[motorIndex] = motorState::STOPPED;
    motorStop(motorIndex);
    command_received[motorIndex] = false;  // Reset command flag when target is reached or no command

    float currentPosition = motorType[motorIndex] == MotorType::LINEAR ? encoders[motorIndex].getPositionUM()
                                                                       : encoders[motorIndex].getPositionDegrees();

    // saveMotorPosition(motorIndex, currentPosition);  // Save the final position to EEPROM when motor stops

    Serial.print(F("\nMotor "));
    Serial.print(motorIndex + 1);
    Serial.print(F(" stopped at "));
    Serial.print(currentPosition);
    Serial.println(motorType[motorIndex] == MotorType::LINEAR ? F(" um") : F(" °"));
}

double getShortestAngularDistanceError()
{
    uint8_t motorIndex = getMotorIndex();
    float   currentDeg = encoders[motorIndex].getPositionDegrees();
    float   targetDeg  = getTarget() + ROTATIONAL_THRESHOLD;
    float   error      = targetDeg - currentDeg;

    if (error > 180.0f)
        error -= 360.0f;
    else if (error < -180.0f)
        error += 360.0f;

    return error;
}

double getSignedPositionError()
{
    uint8_t motorIndex      = getMotorIndex();
    float   currentPosition = encoders[motorIndex].getTotalTravelUM();
    float   targetPosition  = getTarget() + LINEAR_THRESHOLD;

    return targetPosition - currentPosition;  // retains sign
}

void rotationalMotorUpdate()
{
    uint8_t motorIndex = getMotorIndex();

    // Calculate signed angular position error [-180, 180]
    float    positionError = getShortestAngularDistanceError();
    uint64_t steps         = fabs(positionError) * DEGREE_TO_PULSE_FACTOR_12B_256;

    // If previously stopped, save the distance
    if (motorLastState[motorIndex] == motorState::STOPPED)
    {
        distanceToTarget = fabs(positionError);
    }

    // Check if we haven't reached the target yet
    if (fabs(positionError) > ROTATIONAL_THRESHOLD)
    {
        motorLastState[motorIndex] = motorState::MOVING;

        // Select the appropriate direction and move the motor
        if (fabs(positionError) <= ROTATIONAL_THRESHOLD)  // Threshold to stop
            motorStopAndSavePosition();
        else if (positionError > 0.0f)
            motorMoveReverse(motorIndex);  // Counterclockwise rotation
        else
            motorMoveForward(motorIndex);  // Clockwise rotation

        if (steps > 120)
            motorStep(motorIndex, 120);
        else if (steps > 80)
            motorStep(motorIndex, 80);
        else if (steps > 40)
            motorStep(motorIndex, 40);
        else if (steps > 20)
            motorStep(motorIndex, 20);
        else if (steps > 10)
            motorStep(motorIndex, 10);
        else
            motorStep(motorIndex, 1);  // Execute all remaining steps
    }
    else
    {
        // If previously moving, stop and save the position
        if (motorLastState[motorIndex] == motorState::MOVING)
        {
            motorStopAndSavePosition();
        }
    }
}

void linearMotorUpdate()
{
    uint8_t motorIndex    = getMotorIndex();
    float   positionError = getSignedPositionError();

    uint64_t steps = fabs(positionError) * UM_TO_PULSE_FACTOR_12B_32;

    if (fabs(positionError) > LINEAR_THRESHOLD)
    {
        motorLastState[motorIndex] = motorState::MOVING;

        if (fabs(positionError) <= LINEAR_THRESHOLD)  // Threshold to stop
            motorStopAndSavePosition();
        else if (positionError > 0.0f)
            motorMoveReverse(motorIndex);  // Counterclockwise rotation
        else
            motorMoveForward(motorIndex);  // Clockwise rotation

        if (steps > 200)
            motorStep(motorIndex, 200);  // Execute the step
        else if (steps > 160)
            motorStep(motorIndex, 160);
        else if (steps > 120)
            motorStep(motorIndex, 120);
        else if (steps > 80)
            motorStep(motorIndex, 80);
        else if (steps > 40)
            motorStep(motorIndex, 40);
        else if (steps > 10)
            motorStep(motorIndex, 10);
        else
            motorStep(motorIndex, 1);  // Execute all remaining steps
    }
    else
    {
        if (motorLastState[motorIndex] == motorState::MOVING)
        {
            motorStopAndSavePosition();
        }
    }
}
double x = 0;
void   printSerial()
{
    uint8_t motorIndex = getMotorIndex();
    float   position   = motorType[motorIndex] == MotorType::ROTATIONAL ? encoders[motorIndex].getPositionDegrees()
                                                                        : encoders[motorIndex].getTotalTravelUM();
    float   error = motorType[motorIndex] == MotorType::ROTATIONAL ? getShortestAngularDistanceError() : getSignedPositionError();
    EncoderState state = encoders[motorIndex].getState();
    String direction   = state.direction == Direction::UNKNOWN ? "---" : state.direction == Direction::CLOCKWISE ? "CW" : "CCW";
    double target      = getTarget();

    if (target == 0)
        error = 0;

    if (fabs(state.current_pulse - last_pulse[motorIndex]) > 1)
    {
        if (0)
        {
            //  table header
            Serial.print(F("Motor\tLaps\tDir\tPulse\tHigh\tLow\tPeriod"
                           "\tPosition\tTarget\tError\tlast_sector\tcurrent_sector\ttouched_sectors\n"));

            // Format all values into the buffer
            Serial.print(motorIndex + 1);
            Serial.print(F("\t"));
            Serial.print(state.laps);
            Serial.print(F("\t"));
            Serial.print(direction.c_str());
            Serial.print(F("\t"));
            Serial.print(state.current_pulse);
            Serial.print(F("\t"));
            Serial.print(state.width_high);
            Serial.print(F("\t"));
            Serial.print(state.width_low);
            Serial.print(F("\t"));
            Serial.print(state.period);
            Serial.print(F("\t"));
            Serial.print(position);
            Serial.print(F("\t\t"));
            Serial.print(target);
            Serial.print(F("\t"));
            Serial.print(error);
            Serial.print(F("\t"));
            Serial.print(state.last_sector);
            Serial.print(F("\t\t"));
            Serial.print(state.current_sector);
            Serial.print(F("\t\t"));
            Serial.print(state.touched_sectors);
            Serial.println("\n");
        }
        last_pulse[motorIndex] = state.current_pulse;
    }
}

void setTarget(double position)
{
    uint8_t motorIndex = getMotorIndex();
    target[motorIndex] = position;
}

double getTarget()
{
    uint8_t motorIndex = getMotorIndex();
    return target[motorIndex];
}

bool isValidMotorIndex(uint8_t motorIndex)
{
    if (motorIndex >= NUM_MOTORS)
    {
        Serial.println(F("ERROR: Invalid motor index!"));
        return false;
    }
    return true;
}

bool validationInputAndSetMotorIndex(String motorNumber)
{
    if (motorNumber.length() == 0)
    {
        Serial.println(errorMotorNumberIsRequired);
        return false;
    }

    int motorIndex = motorNumber.toInt() - 1;  // Convert to 0-based index

    if (motorIndex < 0 || motorIndex >= NUM_MOTORS)
    {
        Serial.println(errorMotorNumberIsInvalid);
        return false;
    }

    is_set_motor_number = true;
    setMotorIndex(motorIndex);

    return true;
}

uint8_t getMotorIndex()
{
    return motor_index;
}

void setMotorIndex(uint8_t motorIndex)
{
    if (!isValidMotorIndex(motorIndex))
        return;
    motor_index = motorIndex;
}

void resetCommandReceived()
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        command_received[i] = false;
    }
}

bool validationInputAndSetLinearOffset(String linearOffsetStr)
{
    double offsetUm = linearOffsetStr.toDouble() * UM_PER_PIXEL;
    if (offsetUm < linear_Lower_limit_um || offsetUm > linear_upper_limit_um)
    {
        Serial.print(F("ERROR: Motor 1 offset must be between "));
        Serial.print(linear_Lower_limit_um);
        Serial.print(F(" and "));
        Serial.println(linear_upper_limit_um);
        return false;
    }
    setLinearOffset(offsetUm);
    return true;
}

void setLinearOffset(double offsetUm)
{
    linear_offset_um = offsetUm;
    Serial.print(F("Motor 1 offset set to (px): "));
    Serial.println(linear_offset_um, 2);
}

bool validationInputAndSetLinearLowerLimit(String lowerLimitPx)
{
    double lowerLimit = lowerLimitPx.toDouble();
    setLowerLimits(lowerLimit);
    Serial.print(F("Motor 1 lower limit set to (px): "));
    Serial.println(lowerLimit, 2);
    return true;
}

bool validationInputAndSetLinearUpperLimit(String upperLimitPx)
{
    double upperLimit = upperLimitPx.toDouble();
    setUpperLimits(upperLimit);
    Serial.print(F("Motor 1 upper limit set to (px): "));
    Serial.println(upperLimit, 2);
    return true;
}

void setLowerLimits(double lowerLimit)
{
    linear_Lower_limit_um = lowerLimit;
}

void setUpperLimits(double upperLimit)
{
    linear_upper_limit_um = upperLimit;
}

bool validationInputAndSetTarget(String targetStr)
{
    if (targetStr.length() == 0)
    {
        Serial.println(F("ERROR: Position (-p) requires a value"));
        return false;
    }

    uint8_t motorIndex = getMotorIndex();
    double  position   = targetStr.toDouble();

    Serial.print(F("Motor "));
    Serial.print(motorIndex + 1);
    Serial.print(F(" > new position is: "));
    Serial.print(position);
    Serial.println(motorType[motorIndex] == MotorType::LINEAR ? F(" um") : F(" °"));

    resetCommandReceived();
    setTarget(position);
    command_received[motorIndex] = true;  // Set flag only after valid set target command
    return true;
}

void clearScreen()
{
    printf("\e[1;1H\e[2J");  // clear screen
}
