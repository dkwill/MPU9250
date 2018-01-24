/**
 * @file FusionBias.h
 * @author Seb Madgwick
 * @brief The gyroscope bias correction algorithm provides an estimate of the
 * gyroscope bias to achieve run-time calibration.  The algorithm will detect
 * when the gyroscope is stationary for a set period of time and then begin to
 * sample the gyroscope output to calculate the bias as an average.
 * 
 * This algorithm is intended to be used in conjunction with the AHRS sensor
 * fusion algorithm to improve the accuracy of the gyroscope measurements
 * provided to the AHRS sensor fusion algorithm.
 */

#ifndef FUSION_BIAS_H
#define FUSION_BIAS_H

//------------------------------------------------------------------------------
// Includes

#include "FusionTypes.h"
#include <stdbool.h>

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief Bias correction algorithm structure.  Must be initialised using
 * FusionBiasInitialise before use.
 */
typedef struct {
    int adcThreshold;
    float samplePeriod;
    float stationaryTimer; // internal state (must not be modified by the application)
    FusionVector3 gyroscopeBias; // algorithm output (may be modified at any time by the application)
} FusionBias;

//------------------------------------------------------------------------------
// Function prototypes

void FusionBiasInitialise(FusionBias * const fusionBias, const int adcThreshold, const float samplePeriod);
void FusionBiasUpdate(FusionBias * const fusionBias, const int xAdc, const int yAdc, const int zAdc);
bool FusionBiasIsActive(FusionBias * const fusionBias);

#endif

//------------------------------------------------------------------------------
// End of file
