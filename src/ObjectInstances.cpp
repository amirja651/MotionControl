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

MAE3Encoder2 encoders2[NUM_MOTORS] = {
    MAE3Encoder2(ENC_A, ENC_A, 0, EncoderResolution::BITS_12), MAE3Encoder2(ENC_B, ENC_B, 1, EncoderResolution::BITS_12),
    MAE3Encoder2(ENC_C, ENC_C, 2, EncoderResolution::BITS_12), MAE3Encoder2(ENC_D, ENC_D, 3, EncoderResolution::BITS_12)};

PIDController pids[NUM_MOTORS] = {PIDController(pidConfig), PIDController(pidConfig), PIDController(pidConfig),
                                  PIDController(pidConfig)};

void initializeOtherObjects()
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        encoders2[i].begin();
        pids[i].begin();

        if (i == 0)
        {
            pids[i].setOutputLimits(mmToUm(encoders2[i].getLowerLimits()), mmToUm(encoders2[i].getUpperLimits()));
        }
        else
        {
            pids[i].setOutputLimits(ROTATIONAL_OUTPUT_LIMIT_MIN, ROTATIONAL_OUTPUT_LIMIT_MAX);
        }
    }
}