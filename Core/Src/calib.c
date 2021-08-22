/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Digilent

  @File Name
    calib.c

  @Description
        This file groups the functions that implement the CALIB module.
        For each scale, two calibration coefficients (additive and multiplicative) are maintained in the calib global data structure.
        Calibration process consists of declaring pairs of measured value / reference value for zero, positive and eventually negative calibration points.
        This is done by calls of CALIB_MeasureForCalib() and Calib() functions.
        When all the required steps are performed, calibration coefficients are computed and stored in the calib global data structure.
        After calibration, the calibration data must be stored in user calibration area of EPROM.
        During manufacturing, the factory calibration is performed. This is also stored in a factory calibration area of EPROM.
        The user calibration area of EPROM stores the calibration performed by user.
        When the calibration module is initialized the calibration data is read from user calibration area of EPROM.
        The data from factory calibration area of EPROM can be later restored and saved in the user calibration area of EPROM.
        The "Interface functions" section groups functions that can also be called by User. These are initialization functions,
        EPROM functions and calibration procedure functions.

  @Author
    Cristian Fatu
    cristian.fatu@digilent.ro

 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "dmm.h"
#include "calib.h"

CALIBDATA calib;							// global variable - also visible in dmm.c (where declared as extern)

static PARTCALIBDATA partCalib;				// used to store calibration related values, until all the needed calibration data is present and calibration can be finalized.

static uint8_t CALIB_MeasureForCalibZeroVal(double *pMeasuredVal);
static void CALIB_InitPartCalibData(void);
static uint8_t CALIB_ReadAllCalibsFromEPROM(void);
static uint8_t CALIB_CheckCompleteCalib();

static void CALIB_ReplaceCalibNullValues();

static uint8_t CALIB_ERR_CheckDoubleVal(double dVal);

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */
/***	CALIB_Init()
 **
 **	Parameters:
 **		none
 **
 **	Return Value:
 **		uint8_t
 **          ERRVAL_SUCCESS                  0       // success
 **          ERRVAL_EPROM_MAGICNO            0xFD    // wrong Magic No. when reading data from EPROM
 **          ERRVAL_EPROM_CRC                0xFE    // wrong CRC when reading data from EPROM
 **
 **	Description:
 **		This function initializes the calibration related data.
 **      It initializes the EPROM module, the partial calibration data
 **      and reads all the calibration values from user calibration area of EPROM.
 **      The return values are related to errors when calibration is read from user calibration area of EPROM.
 **      The function returns ERRVAL_SUCCESS when success.
 **      The function returns ERRVAL_EPROM_MAGICNO when a wrong magic number was detected in the data read from EPROM.
 **      The function returns ERRVAL_EPROM_CRC when the checksum is wrong for the data read from EPROM.
 **
 */
uint8_t CALIB_Init( void )
{
	CALIB_InitPartCalibData();				// initialize partial calibration data

	return CALIB_ReadAllCalibsFromEPROM();
}

/***	CALIB_ReadAllCalibsFromEPROM_User
 **
 **	Parameters:
 **      none
 **
 **	Return Value:
 **		uint8_t
 **          ERRVAL_SUCCESS                  0       // success
 **          ERRVAL_EPROM_MAGICNO            0xFD    // wrong Magic No. when reading data from EPROM
 **          ERRVAL_EPROM_CRC                0xFE    // wrong CRC when reading data from EPROM
 **
 **	Description:
 **		This function reads the user calibration data from EPROM.
 **      It calls the local function CALIB_ReadAllCalibsFromEPROM_Raw function providing the address of user calibration area in EPROM,
 **      The function returns ERRVAL_SUCCESS for success.
 **      The function returns ERRVAL_EPROM_MAGICNO when a wrong magic number was detected in the data read from EPROM.
 **      The function returns ERRVAL_EPROM_CRC when the checksum is wrong for the data read from EPROM.
 **
 */
static uint8_t CALIB_ReadAllCalibsFromEPROM( void )
{
	uint8_t bResult = ERRVAL_SUCCESS;

	// TODO
//	bResult = ReadCalibsFromEPROM();
	memset( &calib, 0, sizeof(calib) );

	CALIB_ReplaceCalibNullValues();
	return bResult;
}

// End EPROM functions
// Calibration functions

/***	CALIB_ComputeMult
 **
 **	Parameters:
 **		int idxScale    - the Scale index
 **
 **	Return Value:
 **		float               - the MULT calibration coefficient
 **
 **	Description:
 **		This function computes and returns the MULT calibration coefficient for a specific Scale index.
 **      It uses the data stored in partCalib, collected throughout the calibration process.
 **      It uses different formulas, depending on the configuration type (AC, DC, Diode or Resistance).
 **
 **
 */
float CALIB_ComputeMult( int idxScale )
{
	float fResult = 0;
	uint8_t fAC, fDC, fResistance, fDiode, fContinuity;
	fAC = DMM_FACScale( idxScale );
	fDC = DMM_FDCScale( idxScale );
	fResistance = DMM_FResistorScale( idxScale );
	fDiode = DMM_FDiodeScale( idxScale );
	fContinuity = DMM_FContinuityScale( idxScale );

	if( fAC )
	{
		// 2 points calib AC
		fResult = (float)( ( partCalib.DmmPartCalib[ idxScale ].Calib_Ref_ValP ) / sqrt( pow( partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValP, 2 ) - pow( partCalib.DmmPartCalib[ idxScale ].Calib_Ms_Zero, 2 ) ) );
	}
	else if( fDC )
	{
		// 3 points calib DC
		fResult = (float)( ( partCalib.DmmPartCalib[ idxScale ].Calib_Ref_ValP - partCalib.DmmPartCalib[ idxScale ].Calib_Ref_ValN ) / ( partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValP - partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValN ) );
	}
	if( fResistance || fDiode || fContinuity )
	{
		// 2 points calib Diode, Resistance
		fResult = (float)( ( (
			fResistance || fContinuity ? CALIB_RES_ZERO_REFVAL : 0 ) - partCalib.DmmPartCalib[ idxScale ].Calib_Ref_ValP ) / ( partCalib.DmmPartCalib[ idxScale ].Calib_Ms_Zero - partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValP ) );
	}

	if( DMM_IsNotANumber( fResult ) )
	{
		fResult = 0;
	}
	return fResult;
}

/***	CALIB_ComputeAdd
 **
 **	Parameters:
 **		int idxScale    - the Scale index
 **
 **	Return Value:
 **		float               - the ADD calibration coefficient
 **
 **	Description:
 **		This function computes and returns the ADD calibration coefficient for a specific Scale index.
 **      It uses the data stored in partCalib, collected throughout the calibration process.
 **      It uses different formulas, depending on the configuration type (AC, DC, Diode or Resistance).
 **
 **
 */
float CALIB_ComputeAdd( int idxScale )
{
	float fResult = 0;
	uint8_t fAC, fDC, fResistance, fDiode, fContinuity;
	fAC = DMM_FACScale( idxScale );
	fDC = DMM_FDCScale( idxScale );
	fResistance = DMM_FResistorScale( idxScale );
	fDiode = DMM_FDiodeScale( idxScale );
	fContinuity = DMM_FContinuityScale( idxScale );
	if( fAC )
	{
		// 2 points calib AC
		fResult = partCalib.DmmPartCalib[ idxScale ].Calib_Ms_Zero;
	}
	else
	{
		if( fDC )
		{
			// 3 points calib DC
			fResult = (float)( 0 - partCalib.DmmPartCalib[ idxScale ].Calib_Ms_Zero ) * ( 1.0 + CALIB_ComputeMult( idxScale ) );
		}
		if( fResistance || fDiode || fContinuity )
		{
			// 2 points calib Diode, Resistance
			fResult = (float)( (
				fResistance || fContinuity ? CALIB_RES_ZERO_REFVAL : 0 ) - partCalib.DmmPartCalib[ idxScale ].Calib_Ms_Zero ) * ( 1.0 + CALIB_ComputeMult( idxScale ) );
		}
	}
	if( DMM_IsNotANumber( fResult ) )
	{
		fResult = 0;
	}
	return fResult;
}

/***	CALIB_MeasureForCalibZeroVal
 **
 **	Parameters:
 **		double *pMeasuredVal    - Pointer to a double variable that will store the measured value
 **
 **	Return Value:
 **		uint8_t
 **          ERRVAL_SUCCESS                  0       // success
 **          ERRVAL_DMM_IDXCONFIG            0xFC    // wrong scale index
 **          ERRVAL_DMM_VALIDDATATIMEOUT     0xFA    // valid data DMM timeout
 **
 **	Description:
 **		This function performs the measurement for calibration on zero, for the currently selected scale.
 **      The function calls the DMM_DGetAvgValue function in order to acquire the measured value without the calibration correction being applied.
 **      When success, the measured value is stored in the Calib_Ms_Zero field of partCalibData, and it's set as measured value.
 **      If there is no valid current configuration selected, the function returns ERRVAL_DMM_IDXCONFIG and the measured value is set to NAN.
 **      If a valid measurement cannot be performed, the function returns ERRVAL_DMM_VALIDDATATIMEOUT and the measured value is set to NAN.
 **      This function is normally called by the CALIB_CalibOnZero function but it can also be called directly by the user.
 **
 */
uint8_t CALIB_MeasureForCalibZeroVal( double *pMeasuredVal )
{
	double dVal = NAN;
	int idxScale = DMM_GetScale( 1 );
	uint8_t bResult = DMM_ERR_CheckIdxCalib( idxScale );
	if( bResult == ERRVAL_SUCCESS )
	{
		DMM_SetUseCalib( 0 );
		dVal = DMM_DGetAvgValue( 1, MEASURE_CNT_AVG, &bResult ); // compute average value
		DMM_SetUseCalib( 1 );

		if( bResult == ERRVAL_SUCCESS )
		{
			// store the measured value
			partCalib.DmmPartCalib[ idxScale ].Calib_Ms_Zero = dVal;
		}
	}
	if( bResult != ERRVAL_SUCCESS )
	{
		dVal = NAN;
	}

	if( pMeasuredVal )
	{
		*pMeasuredVal = dVal;
	}

	return bResult;
}

/***	CALIB_CalibOnZero
 **
 **	Parameters:
 **		double *pMeasuredVal            - Pointer to a double variable that will store the measured value
 **
 **	Return Value:
 **		uint8_t
 **          ERRVAL_SUCCESS                  0       // success
 **          ERRVAL_DMM_IDXCONFIG            0xFC    // wrong scale index
 **          ERRVAL_DMM_VALIDDATATIMEOUT     0xFA    // valid data DMM timeout
 **
 **	Description:
 **		This function implements the calibration on zero procedure, for the currently selected scale.
 **      The function calls the CALIB_MeasureForCalibZeroVal local function in order to perform the measurement and provide the measured value.
 **      When success, the function calls local function CALIB_CheckCompleteCalib, to check if the calibration process is complete.
 **      If there is no valid current configuration selected, the function returns ERRVAL_DMM_IDXCONFIG and the measured value is set to NAN.
 **      If a valid measurement cannot be performed, the function returns ERRVAL_DMM_VALIDDATATIMEOUT and the measured value is set to NAN.
 **
 */
uint8_t CALIB_CalibOnZero( double *pMeasuredVal )
{
	int idxScale = DMM_GetScale( 1 );
	uint8_t bResult = DMM_ERR_CheckIdxCalib( idxScale );
	// do the measurement now
	if( bResult == ERRVAL_SUCCESS )
	{
		bResult = CALIB_MeasureForCalibZeroVal( pMeasuredVal );

		if( bResult == ERRVAL_SUCCESS )
		{
			if( bResult == ERRVAL_SUCCESS )
			{
				// check if the calibration data is complete
				CALIB_CheckCompleteCalib();
			}
			else
			{
				// remove the measurement data
				partCalib.DmmPartCalib[ idxScale ].Calib_Ms_Zero = NAN;
			}
		}
	}
	return bResult;
}

/***	CALIB_MeasureForCalibPositiveVal
 **
 **	Parameters:
 **		double *pMeasuredVal    - Pointer to a double variable that will store the measured value
 **
 **	Return Value:
 **		uint8_t
 **          ERRVAL_SUCCESS                  0       // success
 **          ERRVAL_DMM_IDXCONFIG            0xFC    // wrong scale index
 **          ERRVAL_DMM_VALIDDATATIMEOUT     0xFA    // valid data DMM timeout
 **
 **	Description:
 **		This function performs the measurement for the calibration on positive value procedure, for the currently selected scale.
 **      The function calls the DMM_DGetAvgValue function in order to acquire the measured value without the calibration correction being applied.
 **      When success, the measured value is stored in the Calib_Ms_ValP field of partCalibData structure, and it's set as measured value.
 **      If there is no valid current configuration selected, the function returns ERRVAL_DMM_IDXCONFIG and the measured value is set to NAN.
 **      If a valid measurement cannot be performed, the function returns ERRVAL_DMM_VALIDDATATIMEOUT and the measured value is set to NAN.
 **      This function can be called by CALIB_CalibOnPositive or can be called directly, before CALIB_CalibOnPositive (this is considered early measurement).
 **
 **
 */
uint8_t CALIB_MeasureForCalibPositiveVal( double *pMeasuredVal )
{
	double dVal = NAN;
	int idxScale = DMM_GetScale( 1 );
	uint8_t bResult = DMM_ERR_CheckIdxCalib( idxScale );
	if( bResult == ERRVAL_SUCCESS )
	{
		DMM_SetUseCalib( 0 );
		dVal = DMM_DGetAvgValue( 1, MEASURE_CNT_AVG, &bResult ); // compute and get average value
		DMM_SetUseCalib( 1 );

		if( bResult == ERRVAL_SUCCESS )
		{
			// store the measured value
			partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValP = dVal;
		}
	}
	if( bResult != ERRVAL_SUCCESS )
	{
		dVal = NAN;
	}
	if( pMeasuredVal )
	{
		*pMeasuredVal = dVal;
	}
	return bResult;
}

/***	CALIB_CalibOnPositive
 **
 **	Parameters:
 **		double dRefVal                  - The reference value, to be used in the calibration procedure
 **		double *pMeasuredVal            - Pointer to a double variable that will store the measured value
 **      uint8_t bEarlyMeasurement - Boolean parameter indicating if an early measurement was performed
 **
 **	Return Value:
 **		uint8_t
 **          ERRVAL_SUCCESS                  0       // success
 **          ERRVAL_DMM_IDXCONFIG            0xFC    // wrong scale index
 **          ERRVAL_DMM_VALIDDATATIMEOUT     0xFA    // valid data DMM timeout
 **          ERRVAL_CALIB_MISSINGMEASUREMENT 0xF0    // A measurement must be performed before calling the finalize calibration function.
 **
 **	Description:
 **		This function implements the calibration on positive value procedure, for the currently selected scale.
 **      It is possible that function CALIB_MeasureForCalibPositiveVal was previously called. This is considered early measurement,
 **      and in this case parameter bEarlyMeasurement must be set to a non 0 value.
 **      If no early measurement was performed (bEarlyMeasurement parameter is 0), the function calls CALIB_MeasureForCalibPositiveVal
 **      in order to perform the measurement and provide the measured value.
 **      If early measurement was performed (bEarlyMeasurement parameter is non 0), the measured value is copied from Calib_Ms_ValP field of partCalibData.
 **      If bEarlyMeasurement parameter is non 0 and Calib_Ms_ValP is not valid (no valid measurement
 **      was previously performed), the function returns ERRVAL_CALIB_MISSINGMEASUREMENT.
 **      When success, the reference value is stored in the Calib_Ref_ValP field of partCalibData.
 **      When success, the function calls local function CALIB_CheckCompleteCalib, to check if the calibration process is complete.
 **      If there is no valid current configuration selected, the function returns ERRVAL_DMM_IDXCONFIG and the measured value is set to NAN.
 **      If a valid measurement cannot be performed, the function returns ERRVAL_DMM_VALIDDATATIMEOUT and the measured value is set to NAN.
 **
 */
uint8_t CALIB_CalibOnPositive( double dRefVal, double *pMeasuredVal, uint8_t bEarlyMeasurement )
{
	int idxScale = DMM_GetScale( 1 );
	uint8_t bResult = DMM_ERR_CheckIdxCalib( idxScale );
	if( !bEarlyMeasurement )
	{
		// no early measurement has been done, do the measurement now
		bResult = CALIB_MeasureForCalibPositiveVal( pMeasuredVal );
	}
	else
	{
		// just inform the caller about the existing measurement
		if( pMeasuredVal )
		{
			*pMeasuredVal = partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValP;
		}
		if( DMM_IsNotANumber( partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValP ) )
		{
			bResult = ERRVAL_CALIB_MISSINGMEASUREMENT;
		}
	}
	if( bResult == ERRVAL_SUCCESS )
	{
		bResult = CALIB_ERR_CheckDoubleVal( dRefVal );
		if( bResult == ERRVAL_SUCCESS )
		{
			partCalib.DmmPartCalib[ idxScale ].Calib_Ref_ValP = dRefVal;
		}
		if( bResult == ERRVAL_SUCCESS )
		{
			// check if the calibration data is complete
			CALIB_CheckCompleteCalib();
		}
		else
		{
			// remove the reference data
			partCalib.DmmPartCalib[ idxScale ].Calib_Ref_ValP = NAN;
		}
	}
	return bResult;
}

/***	CALIB_MeasureForCalibNegativeVal
 **
 **	Parameters:
 **		double *pMeasuredVal    - Pointer to a double variable that will store the measured value
 **
 **	Return Value:
 **		uint8_t
 **          ERRVAL_SUCCESS                  0       // success
 **          ERRVAL_DMM_IDXCONFIG            0xFC    // wrong scale index
 **          ERRVAL_DMM_VALIDDATATIMEOUT     0xFA    // valid data DMM timeout
 **
 **	Description:
 **		This function performs the measurement for the calibration on negative value procedure, for the currently selected scale.
 **      The function calls the DMM_DGetAvgValue function in order to acquire the measured value without the calibration correction being applied.
 **      When success, the measured value is stored in the Calib_Ms_ValN field of partCalibData, and it's set as measured value.
 **      If there is no valid current configuration selected, the function returns ERRVAL_DMM_IDXCONFIG and the measured value is set to NAN.
 **      If a valid measurement cannot be performed, the function returns ERRVAL_DMM_VALIDDATATIMEOUT and the measured value is set to NAN.
 **
 */
uint8_t CALIB_MeasureForCalibNegativeVal( double *pMeasuredVal )
{
	double dVal = NAN;
	int idxScale = DMM_GetScale( 1 );
	uint8_t bResult = DMM_ERR_CheckIdxCalib( idxScale );
	if( bResult == ERRVAL_SUCCESS )
	{
		DMM_SetUseCalib( 0 );
		dVal = DMM_DGetAvgValue( 1, 20, &bResult );   // aquire average value
		DMM_SetUseCalib( 1 );

		if( bResult == ERRVAL_SUCCESS )
		{
			// store the measured value
			partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValN = dVal;
		}
	}
	if( bResult != ERRVAL_SUCCESS )
	{
		dVal = NAN;
	}

	if( pMeasuredVal )
	{
		*pMeasuredVal = dVal;
	}
	return bResult;
}

/***	CALIB_CalibOnNegative
 **
 **	Parameters:
 **		double dRefVal                  - The reference value, to be used in the calibration procedure
 **		double *pMeasuredVal            - Pointer to a double variable that will store the measured value
 **      uint8_t bEarlyMeasurement - Boolean parameter indicating if an early measurement was performed
 **
 **	Return Value:
 **		uint8_t
 **          ERRVAL_SUCCESS                  0       // success
 **          ERRVAL_DMM_IDXCONFIG            0xFC    // wrong scale index
 **          ERRVAL_DMM_VALIDDATATIMEOUT     0xFA    // valid data DMM timeout
 **          ERRVAL_CALIB_MISSINGMEASUREMENT 0xF0    // A measurement must be performed before calling the finalize calibration.
 **
 **	Description:
 **		This function implements the calibration on negative value procedure, for the current selected scale.
 **      It is possible that function CALIB_MeasureForCalibNegativeVal was previously called. This is considered early measurement,
 **      and in this case parameter bEarlyMeasurement must be set to any non 0 value.
 **      If no early measurement was performed (bEarlyMeasurement parameter is 0), the function calls CALIB_MeasureForCalibNegativeVal
 **      in order to perform the measurement and provide the measured value.
 **      If early measurement was performed (bEarlyMeasurement parameter is non 0), the measured value is copied from Calib_Ms_ValN field of partCalibData.
 **      If bEarlyMeasurement parameter is non 0 and Calib_Ms_ValP is not valid (no valid measurement
 **      was previously performed), the function returns ERRVAL_CALIB_MISSINGMEASUREMENT.
 **      If there is no valid current configuration selected, the function returns ERRVAL_DMM_IDXCONFIG and the measured value is set to NAN.
 **      If a valid measurement cannot be performed, the function returns ERRVAL_DMM_VALIDDATATIMEOUT and the measured value is set to NAN.
 **      When success, the reference value is stored in the Calib_Ref_ValN field of partCalibData.
 **      When success, the function calls local function CALIB_CheckCompleteCalib, to check if the calibration process is complete.
 **
 */
uint8_t CALIB_CalibOnNegative( double dRefVal, double *pMeasuredVal, uint8_t bEarlyMeasurement )
{
	int idxScale = DMM_GetScale( 1 );

	uint8_t bResult = DMM_ERR_CheckIdxCalib( idxScale );
	if( !bEarlyMeasurement )
	{
		// no early measurement has been done, do the measurement now
		bResult = CALIB_MeasureForCalibNegativeVal( pMeasuredVal );
	}
	else
	{
		// just inform the caller about the existing measurement
		if( pMeasuredVal )
		{
			*pMeasuredVal = partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValN;
		}
		if( DMM_IsNotANumber( partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValN ) )
		{
			bResult = ERRVAL_CALIB_MISSINGMEASUREMENT;
		}
	}
	if( bResult == ERRVAL_SUCCESS )
	{
		bResult = CALIB_ERR_CheckDoubleVal( dRefVal );
		if( bResult == ERRVAL_SUCCESS )
		{
			partCalib.DmmPartCalib[ idxScale ].Calib_Ref_ValN = dRefVal;
		}
		if( bResult == ERRVAL_SUCCESS )
		{
			// check if the calibration data is complete
			CALIB_CheckCompleteCalib();
		}
		else
		{
			// remove the reference data
			partCalib.DmmPartCalib[ idxScale ].Calib_Ref_ValN = NAN;
		}
	}
	return bResult;
}

/***	CALIB_ImportCalibCoefficients
 **
 **	Parameters:
 **		int idxScale    - the Scale index
 **      float fMult     - the calibration MULT coefficient
 **      float fAdd      - the calibration ADD coefficient
 **
 **	Return Value:
 **		uint8_t
 **          ERRVAL_SUCCESS                  0       // success
 **          ERRVAL_DMM_IDXCONFIG            0xFC    // wrong scale index
 **
 **	Description:
 **		This function imports the MULT and ADD calibration coefficients, for the provided Scale.
 **      On success, the coefficients are copied in the calibration data corresponding to the provided Scale.
 **      The function copies the calibration coefficients into the calibration data and marks the calibration
 **      for the provided Scale as dirty (needs to be written in EPROM).
 **      The function returns ERRVAL_DMM_IDXCONFIG if the provided Scale is not valid.
 **
 */
uint8_t CALIB_ImportCalibCoefficients( int idxScale, float fMult, float fAdd )
{
	uint8_t bResult = DMM_ERR_CheckIdxCalib( idxScale );
	if( bResult == ERRVAL_SUCCESS )
	{
		calib.Dmm[ idxScale ].Mult = fMult;
		calib.Dmm[ idxScale ].Add = fAdd;
		partCalib.DmmPartCalib[ idxScale ].fCalibDirty = 1; // needs to be written to EPROM
	}
	return bResult;
}

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

/***	CALIB_InitPartCalibData()
 **
 **	Parameters:
 **		none
 **
 **	Return Value:
 **		none
 **
 **	Description:
 **		This function initializes the partCalib data, used to store calibration
 **      values, to be used when all the needed calibration will be present.
 **      It also clears the dirty flags, used to mark configurations that were calibrated since last save to EPROM.
 **      This function is intended to be called when the application starts
 **      and every time the calibration data is saved to user space in EPROM
 **
 */
static void CALIB_InitPartCalibData( void )
{
	int idxScale;
	for( idxScale = 0; idxScale < DMM_CNTSCALES; idxScale++ )
	{
		partCalib.DmmPartCalib[ idxScale ].Calib_Ms_Zero = NAN;
		partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValN = NAN;
		partCalib.DmmPartCalib[ idxScale ].Calib_Ref_ValN = NAN;
		partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValP = NAN;
		partCalib.DmmPartCalib[ idxScale ].Calib_Ref_ValP = NAN;
		partCalib.DmmPartCalib[ idxScale ].fCalibDirty = 0;
	}
}

/***	CALIB_ERR_CheckDoubleVal
 **
 **	Parameters:
 **      double dVal     - the value to be checked if valid
 **
 **	Return Value:
 **		uint8_t
 **          ERRVAL_SUCCESS                  0       // the value is a valid double number
 **          ERRVAL_CALIB_NANDOUBLE          0xFB    // the value is not a number double value
 **
 **	Description:
 **		This function checks if the provided double value is a valid value (when it returns ERRVAL_SUCCESS)
 **      or a NAN (not a number) value (when it returns ERRVAL_CALIB_NANDOUBLE).
 **
 */
uint8_t CALIB_ERR_CheckDoubleVal( double dVal )
{
	uint8_t bResult =
		( !DMM_IsNotANumber( dVal ) ) ? ERRVAL_SUCCESS : ERRVAL_CALIB_NANDOUBLE;
	return bResult;
}

/***	CALIB_CheckCompleteCalib
 **
 **	Parameters:
 **		none
 **
 **	Return Value:
 **		0               - calibration is not complete
 **      1               - calibration is complete
 **
 **	Description:
 **		This function checks if the calibration is complete for the currently selected scale.
 **      A calibration is complete if all the partial data is present. This depends on the configuration type.
 **      For example, for DC configurations, a calibration is complete if
 **      Calib_Ms_Zero (zero measurement), Calib_Ms_ValP (positive measurement), Calib_Ref_ValP (positive reference),
 **      Calib_Ms_ValN (negative measurement) and Calib_Ref_ValN (negative reference)
 **      are present. They were previously filled by calls to CALIB_CalibOnZero (or CALIB_MeasureForCalibZeroVal),
 **      CALIB_CalibOnPositive (or CALIB_MeasureForCalibPositiveVal) and CALIB_CalibOnNegative (or CALIB_MeasureForCalibNegativeVal).
 **      If the calibration is found to be complete, the calibration coefficients are computed using CALIB_ComputeMult and CALIB_ComputeAdd functions,
 **      and the scale index is marked as dirty, meaning that calibrations should be written to EPROM user space.
 **      In this moment the calibration is considered finalized, and will be applied to the measured values.
 **
 */
uint8_t CALIB_CheckCompleteCalib( void )
{
	uint8_t fResult = 0;
	uint8_t fCalibZ, fCalibP, fCalibN, fAC, fDC, fResistance, fDiode;
	int idxScale = DMM_GetScale( 1 );

	if( idxScale >= 0 && idxScale < DMM_CNTSCALES )
	{
		fCalibZ = !DMM_IsNotANumber( partCalib.DmmPartCalib[ idxScale ].Calib_Ms_Zero );
		fCalibP = !DMM_IsNotANumber( partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValP ) &&
			!DMM_IsNotANumber( partCalib.DmmPartCalib[ idxScale ].Calib_Ref_ValP );
		fCalibN = !DMM_IsNotANumber( partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValN ) &&
			!DMM_IsNotANumber( partCalib.DmmPartCalib[ idxScale ].Calib_Ref_ValN );
		fAC = DMM_FACScale( idxScale );
		fDC = DMM_FDCScale( idxScale );
		fDiode = DMM_FDiodeScale( idxScale );
		fResistance = DMM_FResistorScale( idxScale );

		fResult = ( ( fDC && fCalibZ && fCalibP && fCalibN ) ||
			( ( fAC || fResistance || fDiode ) && fCalibZ && fCalibP ) );

		if( fResult )
		{
			calib.Dmm[ idxScale ].Mult = CALIB_ComputeMult( idxScale );
			calib.Dmm[ idxScale ].Add = CALIB_ComputeAdd( idxScale );
			partCalib.DmmPartCalib[ idxScale ].fCalibDirty = 1; // needs to be written to EPROM
		}
	}
	return fResult;
}

/***	CALIB_ReplaceCalibNullValues()
 **
 **	Parameters:
 **      none
 **
 **	Return Value:
 **		none
 **
 **	Description:
 **		This function checks all the calibration data and replaces the forbidden values (not a number) with neutral value 0.
 **      It is abnormal to have forbidden values, still this situation theoretically might occur.
 **
 */
void CALIB_ReplaceCalibNullValues()
{
	int idxScale;
	for( idxScale = 0; idxScale < DMM_CNTSCALES; idxScale++ )
	{
		if( *(uint32_t*)&calib.Dmm[ idxScale ].Mult == 0 || DMM_IsNotANumber( calib.Dmm[ idxScale ].Mult ) )
		{
			calib.Dmm[ idxScale ].Mult = 1.0; // use 1 if no calibration data is available (abnormal situation)
		}

		if( *(uint32_t*)&calib.Dmm[ idxScale ].Add == 0 || DMM_IsNotANumber( calib.Dmm[ idxScale ].Add ) )
		{
			calib.Dmm[ idxScale ].Add = 0.0; // use 0 if no calibration data is available (abnormal situation)
		}
	}
}

/* *****************************************************************************
 End of File
 */
