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
        After calibration, the calibration data must be stored in user calibration area of FLASH.
        During manufacturing, the factory calibration is performed. This is also stored in a factory calibration area of FLASH.
        The user calibration area of FLASH stores the calibration performed by user.
        When the calibration module is initialized the calibration data is read from user calibration area of FLASH.
        The data from factory calibration area of FLASH can be later restored and saved in the user calibration area of FLASH.
        The "Interface functions" section groups functions that can also be called by User. These are initialization functions,
        FLASH functions and calibration procedure functions.

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

#include "main.h"
#include "dmm.h"
#include "calib.h"

// calibration values

CALIBDATA calib[DMM_CNTSCALES] =
{
	/* SCALE_DC_50mV */
	/* SCALE_DC_500mV */
	/* SCALE_DC_5V */
	/* SCALE_DC_50V */
	/* SCALE_DC_500V */
	/* SCALE_DC_1kV */

	/* SCALE_AC_500mV */
	/* SCALE_AC_5V */
	/* SCALE_AC_50V */
	/* SCALE_AC_500V */
	/* SCALE_AC_750V */

	/* SCALE_DC_500uA */
	/* SCALE_DC_50mA */
	/* SCALE_DC_500mA */
	/* SCALE_DC_5A */
	/* SCALE_DC_10A */

	/* SCALE_AC_500uA */
	/* SCALE_AC_5mA */
	/* SCALE_AC_50mA */
	/* SCALE_AC_500mA */
	/* SCALE_AC_5A */
	/* SCALE_AC_10A */

	/* SCALE_50_Ohm */
	/* SCALE_500_Ohm */
	/* SCALE_5_kOhm */
	/* SCALE_50_kOhm */
	/* SCALE_500_kOhm */
	/* SCALE_5_MOhm */
	/* SCALE_50_MOhm */

	/* SCALE_4W_500_Ohm */
	/* SCALE_4W_5_kOhm */
	/* SCALE_4W_50_kOhm */

	/* SCALE_50_nF */
	/* SCALE_500_nF */
	/* SCALE_5_uF */
	/* SCALE_50_uF */
	/* SCALE_500_uF */
	/* SCALE_5_mF */
	/* SCALE_50_mF */

	/* SCALE_FREQ */

	/* SCALE_TEMP */

	/* SCALE_CONT */
	/* SCALE_DIODE */
};

static PARTCALIBDATA partCalib;				// used to store calibration related values, until all the needed calibration data is present and calibration can be finalized.

/*
 * DCV		1(50mV),	2(500mV),	3(5V),		4(50V),		5(500V),	6(1kV)					6
 * ACV		1(500mV),	2(5V),		3(50V),		4(500V),	5(750V)								4	(500V = 750V)
 * DCI		1(500uA),	2(5mA),		3(50mA),	4(500mA),	5(5A),		6(10A)					6
 * ACI		1(500uA),	2(5mA),		3(50mA),	4(500mA),	5(5A),		6(10A)					6
 * RES		1(500Ω),	2(5KΩ),		3(50KΩ),	4(500KΩ),	5(5MΩ),		6(50MΩ)					6
 * FRES		1(500Ω),	2(5KΩ),		3(50KΩ)														3
 * DIOD		1(3V)																				1
 * CONT		1(1kΩ)																				1
 * CAP		1(50nF),	2(500nF),	3(5uF),		4(50uF),	5(500uF),	6(5mF),		7(50mF)		3	(50nF..500uF are the same settings)
 * TEMP		1(KITS90),	2(PT100)																2
 * 																							-----
 * 																							   41
 */

typedef struct CALIBRATION_AREA {
	char HW_Model[20];		// 0801F0A8 / 0801F800 - "XDM2041" (0xFF padded)
	char HW_Version1[20];	// 0801F0BC / 0801F814 - "V1.7.2" (OWON), "1.9.0" (VC)
	char SerialNo[20];		// 0801F0D0 / 0801F828 - "2038315"
	char Unknown1[20];		// 0801F0E4 / 0801F83C - "" (zeroes, but seems to be another string, as it is 20 bytes, too)
	char BRAND_Model[20];	// 0801F0F8 / 0801F850 - "XDM2041", "VC7055BT" etc.
	char BRAND_Name[20];	// 0801F10C / 0801F864 - "OWON", "Voltcraft" etc.

	char Flags1[112];		// 0801F120 / 0801F878 - there seem to be 112 calibration slots
	char Flags2[112];		// 0801F190 / 0801F8E8 -

	char Unknown2[3];		// 0801F200 / 0801F958 -
	char FW_Version[5];		// 0801F203 / 0801F95B - "V1.3"

	char Flags3[152];		// 0801F208 / 0801F960 -

	int32_t Mult[112];		// 0801F2A0 / 0801F9F8 -
	int32_t Add[112];		// 0801F460 / 0801FAA0 -

	// other stuff follows
} __attribute__((__packed__)) CAL_DATA;

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
 **      It also clears the dirty flags, used to mark configurations that were calibrated since last save to FLASH.
 **      This function is intended to be called when the application starts
 **      and every time the calibration data is saved to user space in FLASH
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

/***	CALIB_ReadAllCalibsFromFLASH_User
 **
 **	Parameters:
 **      none
 **
 **	Return Value:
 **		uint8_t
 **          ERRVAL_SUCCESS                  0       // success
 **          ERRVAL_FLASH_MAGICNO            0xFD    // wrong Magic No. when reading data from FLASH
 **          ERRVAL_FLASH_CRC                0xFE    // wrong CRC when reading data from FLASH
 **
 **	Description:
 **		This function reads the user calibration data from FLASH.
 **      It calls the local function CALIB_ReadAllCalibsFromFLASH_Raw function providing the address of user calibration area in FLASH,
 **      The function returns ERRVAL_SUCCESS for success.
 **      The function returns ERRVAL_FLASH_MAGICNO when a wrong magic number was detected in the data read from FLASH.
 **      The function returns ERRVAL_FLASH_CRC when the checksum is wrong for the data read from FLASH.
 **
 */
static uint8_t CALIB_ReadAllCalibsFromFLASH( void )
{
	uint8_t idxScale, bResult = ERRVAL_SUCCESS;

	// TODO
#if 0
	const CAL_DATA *caldata = (const CAL_DATA *)0x0801F800;
	if( caldata->HW_Model == 0xFF )			// missing calibration data !
	{
		HAL_FLASH_Unlock();

		size_t i;
		for( i = 0; i < sizeof(cal_defaults); i += 4 )
		{
			uint32_t Address = (uint32_t)caldata + i;
			Data = ((uint32_t*)cal_defaults)[i];
			HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, Address, Data );		// program 32-Bit word
		}

		HAL_FLASH_Lock();
	}

#else
	for( idxScale = 0; idxScale < DMM_CNTSCALES; ++idxScale )
	{
		calib[ idxScale ].Mult = 1.0;
		calib[ idxScale ].Add = 0.0;
	}
#endif

	for( idxScale = 0; idxScale < DMM_CNTSCALES; ++idxScale )
	{
		if( *(uint32_t*)&calib[ idxScale ].Mult == 0 || DMM_isNAN( calib[ idxScale ].Mult ) )
			calib[ idxScale ].Mult = 1.0;	// use 1 if no calibration data is available (abnormal situation)

		if( DMM_isNAN( calib[ idxScale ].Add ) )
			calib[ idxScale ].Add = 0.0;	// use 0 if no calibration data is available (abnormal situation)
	}
	return bResult;
}

/***	CALIB_Init()
 **
 **	Parameters:
 **		none
 **
 **	Return Value:
 **		uint8_t
 **          ERRVAL_SUCCESS                  0       // success
 **          ERRVAL_FLASH_MAGICNO            0xFD    // wrong Magic No. when reading data from FLASH
 **          ERRVAL_FLASH_CRC                0xFE    // wrong CRC when reading data from FLASH
 **
 **	Description:
 **		This function initializes the calibration related data.
 **      It initializes the FLASH module, the partial calibration data
 **      and reads all the calibration values from user calibration area of FLASH.
 **      The return values are related to errors when calibration is read from user calibration area of FLASH.
 **      The function returns ERRVAL_SUCCESS when success.
 **      The function returns ERRVAL_FLASH_MAGICNO when a wrong magic number was detected in the data read from FLASH.
 **      The function returns ERRVAL_FLASH_CRC when the checksum is wrong for the data read from FLASH.
 **
 */
uint8_t CALIB_Init( void )
{
	CALIB_InitPartCalibData();				// initialize partial calibration data

	return CALIB_ReadAllCalibsFromFLASH();
}

/***	CALIB_ComputeMult
 **
 **	Parameters:
 **		int idxScale    - the Scale index
 **
 **	Return Value:
 **		double               - the MULT calibration coefficient
 **
 **	Description:
 **		This function computes and returns the MULT calibration coefficient for a specific Scale index.
 **      It uses the data stored in partCalib, collected throughout the calibration process.
 **      It uses different formulas, depending on the configuration type (AC, DC, Diode or Resistance).
 **
 **
 */
static double CALIB_ComputeMult( int idxScale )
{
	double fResult = 0;

	if(		 DMM_isAC( idxScale ) )		// 2 points calib AC
		fResult = (double)( ( partCalib.DmmPartCalib[ idxScale ].Calib_Ref_ValP ) / sqrt( pow( partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValP, 2 ) - pow( partCalib.DmmPartCalib[ idxScale ].Calib_Ms_Zero, 2 ) ) );
	else if( DMM_isDC( idxScale ) )		// 3 points calib DC
		fResult = (double)( ( partCalib.DmmPartCalib[ idxScale ].Calib_Ref_ValP - partCalib.DmmPartCalib[ idxScale ].Calib_Ref_ValN ) / ( partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValP - partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValN ) );
	else if( DMM_isRES( idxScale ) ||
			 DMM_isDIOD( idxScale ) ||
			 DMM_isCONT( idxScale ) )	// 2 points calib Resistance and Diode
		fResult = (double)( ( 0 - partCalib.DmmPartCalib[ idxScale ].Calib_Ref_ValP ) / ( partCalib.DmmPartCalib[ idxScale ].Calib_Ms_Zero - partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValP ) );

	if( DMM_isNAN( fResult ) )
		fResult = 0;
	return fResult;
}

/***	CALIB_ComputeAdd
 **
 **	Parameters:
 **		int idxScale    - the Scale index
 **
 **	Return Value:
 **		double               - the ADD calibration coefficient
 **
 **	Description:
 **		This function computes and returns the ADD calibration coefficient for a specific Scale index.
 **      It uses the data stored in partCalib, collected throughout the calibration process.
 **      It uses different formulas, depending on the configuration type (AC, DC, Diode or Resistance).
 **
 **
 */
static double CALIB_ComputeAdd( int idxScale )
{
	double fResult = 0;

	if( DMM_isAC( idxScale ) )		// 2 points calib AC
		fResult = partCalib.DmmPartCalib[ idxScale ].Calib_Ms_Zero;
	else if( DMM_isDC( idxScale ) )	// 3 points calib DC
		fResult = (double)( 0 - partCalib.DmmPartCalib[ idxScale ].Calib_Ms_Zero ) * ( 1.0 + CALIB_ComputeMult( idxScale ) );
	else if( DMM_isRES( idxScale ) || DMM_isDIOD( idxScale ) || DMM_isCONT( idxScale ) )	// 2 points calib Diode, Resistance
		fResult = (double)( 0 - partCalib.DmmPartCalib[ idxScale ].Calib_Ms_Zero ) * ( 1.0 + CALIB_ComputeMult( idxScale ) );

	if( DMM_isNAN( fResult ) )
		fResult = 0;
	return fResult;
}

static uint8_t CALIB_Measure( double *val )
{
	int bResult = ERRVAL_SUCCESS;

	DMM_SetUseCalib( 1, 0 );
	DMM_SetAveraging( 1, MEASURE_CNT_AVG ); // compute average value

	uint16_t timeout = DMM_VALIDDATA_CNTTIMEOUT;
	do
	{
		DMM_Trigger( 1 );
		while( ( bResult = DMM_Measure( 1 ) ) == ERRVAL_CMD_BUSY )
			;
	} while( ( bResult != ERRVAL_SUCCESS || DMM_isNAN( dMeasuredVal[0] ) ) && --timeout );

	DMM_SetUseCalib( 1, 1 );
	DMM_SetAveraging( 1, 1 );

	if( bResult == ERRVAL_SUCCESS )
		*val = dMeasuredVal[0];

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
 **      and the scale index is marked as dirty, meaning that calibrations should be written to FLASH user space.
 **      In this moment the calibration is considered finalized, and will be applied to the measured values.
 **
 */
static uint8_t CALIB_CheckCompleteCalib( void )
{
	int idxScale = DMM_GetScale( 1 );
	uint8_t bResult = DMM_isScale( idxScale );
	if( bResult != ERRVAL_SUCCESS)
		return bResult;

	if( DMM_isNAN( partCalib.DmmPartCalib[ idxScale ].Calib_Ms_Zero ) ||
		DMM_isNAN( partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValP ) ||
		DMM_isNAN( partCalib.DmmPartCalib[ idxScale ].Calib_Ref_ValP ) )
		return ERRVAL_CALIB_NANDOUBLE;

	if( DMM_isDC( idxScale ) && (
		DMM_isNAN( partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValN ) ||
		DMM_isNAN( partCalib.DmmPartCalib[ idxScale ].Calib_Ref_ValN ) ) )
		return ERRVAL_CALIB_NANDOUBLE;

	calib[ idxScale ].Mult = CALIB_ComputeMult( idxScale );
	calib[ idxScale ].Add = CALIB_ComputeAdd( idxScale );
	partCalib.DmmPartCalib[ idxScale ].fCalibDirty = 1;			// needs to be written to FLASH
	return ERRVAL_SUCCESS;
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
	double dVal = NAN;
	int idxScale = DMM_GetScale( 1 );
	uint8_t bResult = DMM_isScale( idxScale );
	if( bResult == ERRVAL_SUCCESS )
		bResult = CALIB_Measure( &dVal );

	if( bResult != ERRVAL_SUCCESS )
		dVal = NAN;

	if( pMeasuredVal )
		*pMeasuredVal = dVal;

	partCalib.DmmPartCalib[ idxScale ].Calib_Ms_Zero = dVal;	// store the measured value

	if( bResult == ERRVAL_SUCCESS )
		CALIB_CheckCompleteCalib();		// check if the calibration data is complete

	return bResult;
}

/***	CALIB_CalibOnPositive
 **
 **	Parameters:
 **		double dRefVal            - The reference value, to be used in the calibration procedure
 **		double *pMeasuredVal      - Pointer to a double variable that will store the measured value
 **
 **	Return Value:
 **		uint8_t
 **          ERRVAL_SUCCESS                  0       // success
 **          ERRVAL_DMM_IDXCONFIG            0xFC    // wrong scale index
 **          ERRVAL_DMM_VALIDDATATIMEOUT     0xFA    // valid data DMM timeout
 **          ERRVAL_CALIB_MISSINGMEASUREMENT 0xF0    // A measurement must be performed before calling the finalize calibration function.
 **
 **	Description:
 **      This function implements the calibration on positive value procedure, for the currently selected scale.
 **      The function calls CALIB_MeasureForCalibPositiveVal in order to perform the measurement and provide the measured value.
 **      If Calib_Ms_ValP is not valid the function returns ERRVAL_CALIB_MISSINGMEASUREMENT.
 **      When success, the reference value is stored in the Calib_Ref_ValP field of partCalibData.
 **      When success, the function calls local function CALIB_CheckCompleteCalib, to check if the calibration process is complete.
 **      If there is no valid current configuration selected, the function returns ERRVAL_DMM_IDXCONFIG and the measured value is set to NAN.
 **      If a valid measurement cannot be performed, the function returns ERRVAL_DMM_VALIDDATATIMEOUT and the measured value is set to NAN.
 **
 */
uint8_t CALIB_CalibOnPositive( double dRefVal, double *pMeasuredVal )
{
	double dVal = NAN;
	int idxScale = DMM_GetScale( 1 );
	uint8_t bResult = DMM_isScale( idxScale );
	if( bResult == ERRVAL_SUCCESS )
		bResult = CALIB_Measure( &dVal );

	if( bResult != ERRVAL_SUCCESS )
		dVal = NAN;

	if( pMeasuredVal )
		*pMeasuredVal = dVal;

	partCalib.DmmPartCalib[ idxScale ].Calib_Ref_ValP = dRefVal;	// store the reference value
	partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValP = dVal;		// store the measured value or NAN on error

	if( bResult == ERRVAL_SUCCESS )
		CALIB_CheckCompleteCalib();		// check if the calibration data is complete

	return bResult;
}

/***	CALIB_CalibOnNegative
 **
 **	Parameters:
 **		double dRefVal                  - The reference value, to be used in the calibration procedure
 **		double *pMeasuredVal            - Pointer to a double variable that will store the measured value
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
 **      The function calls CALIB_MeasureForCalibNegativeVal in order to perform the measurement and provide the measured value.
 **      If Calib_Ms_ValP is not valid the function returns ERRVAL_CALIB_MISSINGMEASUREMENT.
 **      If there is no valid current configuration selected, the function returns ERRVAL_DMM_IDXCONFIG and the measured value is set to NAN.
 **      If a valid measurement cannot be performed, the function returns ERRVAL_DMM_VALIDDATATIMEOUT and the measured value is set to NAN.
 **      When success, the reference value is stored in the Calib_Ref_ValN field of partCalibData.
 **      When success, the function calls local function CALIB_CheckCompleteCalib, to check if the calibration process is complete.
 **
 */
uint8_t CALIB_CalibOnNegative( double dRefVal, double *pMeasuredVal )
{
	double dVal = NAN;
	int idxScale = DMM_GetScale( 1 );
	uint8_t bResult = DMM_isScale( idxScale );
	if( bResult == ERRVAL_SUCCESS )
		bResult = CALIB_Measure( &dVal );

	if( bResult != ERRVAL_SUCCESS )
		dVal = NAN;

	if( pMeasuredVal )
		*pMeasuredVal = dVal;

	partCalib.DmmPartCalib[ idxScale ].Calib_Ref_ValN = dRefVal;	// store the reference value
	partCalib.DmmPartCalib[ idxScale ].Calib_Ms_ValN = dVal;		// store the measured value or NAN on error

	if( bResult == ERRVAL_SUCCESS )
		CALIB_CheckCompleteCalib();	// check if the calibration data is complete

	return bResult;
}

/* *****************************************************************************
 End of File
 */
