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

#define NO_CALIBS				10

#define MEASURE_CNT_AVG			20		// the number of values to be used when measuring for calibration
#define CALIB_RES_ZERO_REFVAL	0.05	// assume 50 mOhm

typedef struct _CALIBDATA {    //
    double  Mult;
    double  Add;
}  __attribute__((__packed__)) CALIBDATA;

typedef struct _PARTCALIB{
    double Calib_Ms_Zero;
    double Calib_Ms_ValP;
    double Calib_Ref_ValP;
    double Calib_Ms_ValN;
    double Calib_Ref_ValN;
    uint8_t fCalibDirty;
} PARTCALIB;

typedef struct _PARTCALIBDATA{    //
    PARTCALIB  DmmPartCalib[DMM_CNTSCALES];    // stores the data needed to the calibration
} PARTCALIBDATA;

extern CALIBDATA calib[DMM_CNTSCALES];

//#define VERSION		((const char*)0x0801F814)
#define SERIALNO		((const char*)0x0801F828)
#define MODEL			((const char*)0x0801F850)
#define MANUFACTURER	((const char*)0x0801F864)

// Calibration procedure functions
uint8_t	CALIB_CalibOnZero(double *pMeasuredVal);
uint8_t	CALIB_CalibOnPositive(double dRefVal, double *pMeasuredVal);
uint8_t	CALIB_CalibOnNegative(double dRefVal, double *pMeasuredVal);

// initialization
uint8_t	CALIB_Init();

#endif /* _CALIB_H */


