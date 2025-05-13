#include "Constants.h"
#include "MAE3Encoder2.h"
#include "Object_Manager.h"
#include "PIDController.h"
#include "UnitConversion.h"

static const uint16_t ENC_A = 36;
static const uint16_t ENC_B = 39;
static const uint16_t ENC_C = 34;
static const uint16_t ENC_D = 35;

static constexpr double _KP = 2.0;
static constexpr double _KI = 0.5;
static constexpr double _KD = 0.1;

PIDConfig pidConfig = {_KP, _KI, _KD};

// Use direct construction with placement new
alignas(MAE3Encoder2) char encoderStorage[sizeof(MAE3Encoder2) * NUM_MOTORS];
MAE3Encoder2* encoders2 = reinterpret_cast<MAE3Encoder2*>(encoderStorage);

// Initialize encoders using placement new
void initializeEncoders()
{
    new (&encoders2[0]) MAE3Encoder2(ENC_A, ENC_A, 0);
    new (&encoders2[1]) MAE3Encoder2(ENC_B, ENC_B, 1);
    new (&encoders2[2]) MAE3Encoder2(ENC_C, ENC_C, 2);
    new (&encoders2[3]) MAE3Encoder2(ENC_D, ENC_D, 3);
}

PIDController pids[NUM_MOTORS] = {PIDController(pidConfig), PIDController(pidConfig), PIDController(pidConfig),
                                  PIDController(pidConfig)};

void initializeOtherObjects()
{
    initializeEncoders();  // Initialize encoders first

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        // Initialize with default settings
        encoders2[i].begin();
        pids[i].begin();

        if (i == 0)
        {
            pids[i].setOutputLimits(LINEAR_LOWER_LIMIT_PX * UM_PER_PIXEL, LINEAR_UPPER_LIMIT_PX * UM_PER_PIXEL);
        }
        else
        {
            pids[i].setOutputLimits(ROTATIONAL_OUTPUT_LIMIT_MIN, ROTATIONAL_OUTPUT_LIMIT_MAX);
        }
    }
}