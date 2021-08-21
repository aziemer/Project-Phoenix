/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Digilent

  @File Name
    eprom.h

  @Description
        This file contains the declarations for the CALIB module functions.
        The EPROM functions are defined in calib.c source file.
 */
/* ************************************************************************** */

#ifndef _CALIB_H    /* Guard against multiple inclusion */
#define _CALIB_H

#include "dmm.h"

#define MEASURE_CNT_AVG			20		// the number of values to be used when measuring for calibration
#define CALIB_RES_ZERO_REFVAL	0.05	// 50 mOhm

extern CALIBDATA calib;

// initialization
uint8_t CALIB_Init();

uint8_t CALIB_ImportCalibCoefficients(int idxScale, float fMult, float fAdd);

// Calibration procedure functions
uint8_t CALIB_CalibOnZero(double *pMeasuredVal);

uint8_t CALIB_MeasureForCalibPositiveVal(double *pMeasuredVal);
uint8_t CALIB_CalibOnPositive(double dRefVal, double *pMeasuredVal, uint8_t bEarlyMeasurement);

uint8_t CALIB_MeasureForCalibNegativeVal(double *pMeasuredVal);
uint8_t CALIB_CalibOnNegative(double dRefVal, double *pMeasuredVal, uint8_t bEarlyMeasurement);

#endif /* _CALIB_H */


