#ifndef UNIT_CONVERSION_H
#define UNIT_CONVERSION_H

// Encoder and Microstep Definitions
#define PULSE_PER_REV_10B   1024
#define PULSE_PER_REV_12B   4096
#define MICROSTEP_256       256
#define MICROSTEP_32        32
#define MICROSTEP_16        16
#define LEAD_SCREW_PITCH_MM 0.2f
#define UM_PER_PIXEL        5.2f
#define MM_TO_UM_FACTOR     1000.0f

// Pre-calculated Conversion Factors (Optimized for Speed)
// Universal Factors
#define UM_TO_PX_FACTOR 0.192308f  // 1 / 5.2
#define PX_TO_UM_FACTOR 5.2f
#define UM_TO_MM_FACTOR 0.001f

// 12-bit Encoder with 256 Microsteps
#define PULSE_TO_MM_FACTOR_12B_256     1.907349e-07f
#define PULSE_TO_UM_FACTOR_12B_256     0.000191f
#define PULSE_TO_DEGREE_FACTOR_12B_256 0.000343f
#define PULSE_TO_PX_FACTOR_12B_256     0.000037f

// 12-bit Encoder with 32 Microsteps
#define PULSE_TO_MM_FACTOR_12B_32     1.525879e-06f
#define PULSE_TO_UM_FACTOR_12B_32     0.001526f
#define PULSE_TO_DEGREE_FACTOR_12B_32 0.002747f
#define PULSE_TO_PX_FACTOR_12B_32     0.000293f

// 12-bit Encoder with 16 Microsteps
#define PULSE_TO_MM_FACTOR_12B_16     3.051758e-06f
#define PULSE_TO_UM_FACTOR_12B_16     0.003052f
#define PULSE_TO_DEGREE_FACTOR_12B_16 0.005493f
#define PULSE_TO_PX_FACTOR_12B_16     0.000586f

// 10-bit Encoder with 32 Microsteps
#define PULSE_TO_MM_FACTOR_10B_32     6.103516e-06f
#define PULSE_TO_UM_FACTOR_10B_32     0.006104f
#define PULSE_TO_DEGREE_FACTOR_10B_32 0.010986f
#define PULSE_TO_PX_FACTOR_10B_32     0.001174f

// 10-bit Encoder with 16 Microsteps
#define PULSE_TO_MM_FACTOR_10B_16     1.220703e-05f
#define PULSE_TO_UM_FACTOR_10B_16     0.012207f
#define PULSE_TO_DEGREE_FACTOR_10B_16 0.021973f
#define PULSE_TO_PX_FACTOR_10B_16     0.002348f

#endif  // UNIT_CONVERSION_H
