/**
 * @file FusionAhrs.h
 * @author Seb Madgwick
 * @brief The AHRS sensor fusion algorithm to combines gyroscope, accelerometer,
 * and magnetometer measurements into a single measurement of orientation
 * relative to the Earth (NWU convention).
 * 
 * The algorithm can be used with only gyroscope and accelerometer measurements,
 * or only gyroscope measurements.  Measurements of orientation obtained without
 * magnetometer measurements can be expected to drift in the yaw component of
 * orientation only.  Measurements of orientation obtained without magnetometer
 * and accelerometer measurements can be expected to drift in all three degrees
 * of freedom.
 * 
 * The algorithm also provides a measurement of linear acceleration and Earth
 * acceleration.  Linear acceleration is equal to the accelerometer  measurement
 * with the 1 g of gravity subtracted.  Earth acceleration is a measurement of
 * linear acceleration in the Earth coordinate frame.
 * 
 * The algorithm outputs a quaternion describing the Earth relative to the
 * sensor.  The library includes a quaternion conjugate function for converting
 * this to a quaternion describing the sensor relative to the Earth, as well as
 * functions for converting a quaternion to a rotation matrix or Euler angles.
 */

#ifndef FUSION_AHRS_H
#define FUSION_AHRS_H

//------------------------------------------------------------------------------
// Includes

#include "FusionTypes.h"
#include <stdbool.h>

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief AHRS algorithm structure.  Must be initialised using
 * FusionAhrsInitialise before use.
 */
typedef struct {
    // Adjustable parameters (may be modified at any time by application)
    float gain;
    float minMagneticFieldSquared;
    float maxMagneticFieldSquared;

    // Algorithm output and internal states (must not be modified by application)
    FusionQuaternion quaternion; // describes the Earth relative to the sensor
    FusionVector3 linearAcceleration;
    float rampedGain;
} FusionAhrs;

//------------------------------------------------------------------------------
// Function prototypes

void FusionAhrsInitialise(FusionAhrs * const fusionAhrs, const float gain, const float minMagneticField, const float maxMagneticField);
void FusionAhrsUpdate(FusionAhrs * const fusionAhrs, const FusionVector3 gyroscope, const FusionVector3 accelerometer, const FusionVector3 magnetometer, const float samplePeriod);
FusionVector3 FusionAhrsCalculateEarthAcceleration(const FusionAhrs * const fusionAhrs);
bool FusionAhrsIsInitialising(const FusionAhrs * const fusionAhrs);
void FusionAhrsReinitialise(FusionAhrs * const fusionAhrs);
void FusionAhrsZeroYaw(FusionAhrs * const fusionAhrs);

#endif

//------------------------------------------------------------------------------
// End of file
