#include "Constants.h"
#include "Object_Manager.h"
#include "PIDController.h"
#include "UnitConversion.h"
#include <MAE3Encoder.h>

static const uint16_t ENC_A = 36;
static const uint16_t ENC_B = 39;
static const uint16_t ENC_C = 34;
static const uint16_t ENC_D = 35;

static constexpr double _KP = 2.0;
static constexpr double _KI = 0.5;
static constexpr double _KD = 0.1;

PIDConfig pidConfig = {_KP, _KI, _KD};

PIDController pids[NUM_MOTORS] = {PIDController(pidConfig), PIDController(pidConfig), PIDController(pidConfig),
                                  PIDController(pidConfig)};

void initializeOtherObjects()
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
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