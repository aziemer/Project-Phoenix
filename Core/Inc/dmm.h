/* ************************************************************************** */
/** Descriptive File Name

  @Company
 Digilent

  @File Name
    dmm.h

  @Description
        This file contains the declaration for the interface functions of DMM module.
        The DMM functions are defined in dmm.c source file.

  @Versioning:
 	 Cristian Fatu - 2018/06/29 - Initial release, DMMShield Library

 */
/* ************************************************************************** */

#ifndef __DMMCFG_H    /* Guard against multiple inclusion */
#define __DMMCFG_H

#include "stdint.h"

#define ERRVAL_SUCCESS                  0       // success

#define ERRVAL_EPROM_WRTIMEOUT          0xFF    // EPROM write data ready timeout
#define ERRVAL_EPROM_CRC                0xFE    // wrong CRC when reading data from EPROM
#define ERRVAL_EPROM_MAGICNO            0xFD    // wrong Magic No. when reading data from EPROM

#define ERRVAL_DMM_IDXCONFIG            0xFC    // wrong scale index
#define ERRVAL_CALIB_NANDOUBLE          0xFB    // not a number double value

#define ERRVAL_DMM_VALIDDATATIMEOUT     0xFA    // valid data DMM timeout
#define ERRVAL_CMD_WRONGPARAMS          0xF9    // wrong parameters when sending UART commands
#define ERRVAL_CMD_MISSINGCODE          0xF8    // the provided code is not among accepted values
#define ERRVAL_EPROM_VERIFY             0xF7    // eprom verify error
#define ERRVAL_EPROM_ADDR_VIOLATION     0xF6    // EPROM write address violation: attempt to write over system data
#define ERRVAL_DMM_CFGVERIFY            0xF5    // DMM Configuration verify error
#define ERRVAL_CMD_VALWRONGUNIT         0xF4    // The provided value has a wrong measure unit.
#define ERRVAL_CMD_VALFORMAT            0xF2    // The numeric value cannot be extracted from the provided string.
#define ERRVAL_CALIB_MISSINGMEASUREMENT 0xF0    // A measurement must be performed before calling the finalize calibration.

#define DMM_DIODEOPENTHRESHOLD			3

enum DMM_MODE {
	DmmIllegalMode,
	DmmResistance,
	DmmResistance4W,
	DmmDCVoltage,
	DmmACVoltage,
	DmmDCCurrent,
	DmmACCurrent,
	DmmCapacitance,
	DmmFrequency,
	DmmTemperature,
	DmmContinuity,
	DmmDiode
};

enum DM_SCALE {
	SCALE_DC_50mV,
	SCALE_DC_500mV,
	SCALE_DC_5V,
	SCALE_DC_50V,
	SCALE_DC_500V,
	SCALE_DC_1kV,

	SCALE_AC_500mV,
	SCALE_AC_5V,
	SCALE_AC_50V,
	SCALE_AC_500V,
	SCALE_AC_750V,

	SCALE_DC_500uA,
	SCALE_DC_5mA,
	SCALE_DC_50mA,
	SCALE_DC_500mA,
	SCALE_DC_5A,
	SCALE_DC_10A,

	SCALE_AC_500uA,
	SCALE_AC_5mA,
	SCALE_AC_50mA,
	SCALE_AC_500mA,
	SCALE_AC_5A,
	SCALE_AC_10A,

	SCALE_50_Ohm,
	SCALE_500_Ohm,
	SCALE_5_kOhm,
	SCALE_50_kOhm,
	SCALE_500_kOhm,
	SCALE_5_MOhm,
	SCALE_50_MOhm,

	SCALE_4W_500_Ohm,
	SCALE_4W_5_kOhm,
	SCALE_4W_50_kOhm,

	SCALE_50_nF,
	SCALE_500_nF,
	SCALE_5_uF,
	SCALE_50_uF,
	SCALE_500_uF,
	SCALE_5_mF,
	SCALE_50_mF,

	SCALE_FREQ,

	SCALE_TEMP,

	SCALE_CONT,
	SCALE_DIODE,

	DMM_CNTSCALES,

	SCALE_AUTO,
	SCALE_UP,
	SCALE_DOWN,
	SCALE_ALT
};

#define DMM_VALIDDATA_CNTTIMEOUT    0x100   // number of valid data retrieval re-tries

#define DMMVoltageDC50Scale				SCALE_DC_50V
#define DMM_Voltage50DCLinearCoeff_P3   -1.59128E-06
#define DMM_Voltage50DCLinearCoeff_P1   1.003918916
#define DMM_Voltage50DCLinearCoeff_P0   0.000196999

// calibration values

#define NO_CALIBS   10

typedef struct _CALIB{
    float  Mult;
    float  Add;
} CALIB;

typedef struct _CALIBDATA {    //
    CALIB      Dmm[DMM_CNTSCALES];
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

#define NUM_CHANNELS	3

extern double dMeasuredVal[NUM_CHANNELS];

uint8_t	DMM_SetScale( uint8_t ch, int idxScale );
int		DMM_GetScale( uint8_t ch );

int		DMM_GetMode( int idxScale );
double	DMM_GetRange( int idxScale );
uint8_t	DMM_GetScaleUnit( uint8_t channel, double *pdScaleFact, char *szUnitPrefix, char *szUnit, char *szRange );

// value functions
double	DMM_DGetAvgValue( uint8_t channel, int cbSamples, uint8_t *pbErr );
uint8_t	DMM_Measure( uint8_t channel, uint8_t fRaw, uint8_t fAvg );

uint8_t	DMM_FormatValue(double dVal, char *pString, uint8_t fUnit);

uint8_t SPrintfDouble( char *pString, double dVal, uint8_t precision );

void	DMM_SetUseCalib(uint8_t f);
char	DMM_GetUseCalib();

uint8_t	DMM_IsNotANumber( double dVal );

uint8_t	DMM_ERR_CheckIdxCalib( int idxScale );
uint8_t	DMM_FDCScale( int idxScale );
uint8_t	DMM_FACScale( int idxScale );
uint8_t	DMM_FCapacitanceScale( int idxScale );
uint8_t	DMM_FDCScale( int idxScale );
uint8_t	DMM_FResistorScale( int idxScale );
uint8_t	DMM_FCapacitanceScale( int idxScale );
uint8_t	DMM_FDiodeScale( int idxScale );
uint8_t	DMM_FContinuityScale( int idxScale );

void	DMM_Init( void );

#endif /* __DMMCFG_H */

/* *****************************************************************************
 End of File
 */
