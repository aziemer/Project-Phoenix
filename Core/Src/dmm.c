#include <dmm.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "stm32f1xx_hal.h"

#include "calib.h"

#define CALIB_ACCEPTANCE_DEFAULT    0.2

#define SPI_NSS_RLY		GPIO_PIN_11		// SPI2_NSS2 (relay latch)
#define SPI_NSS_DMM		GPIO_PIN_12		// SPI2_NSS1 (DMM HY3131)
#define SPI_SCK			GPIO_PIN_13		// SPI2_SCK
#define SPI_MISO		GPIO_PIN_14		// SPI2_MISO
#define SPI_MOSI		GPIO_PIN_15		// SPI2_MOSI

#define CS_DMM			0
#define CS_RLY			1

// registers from 0x00 to 0x1F
typedef struct _DMMSTS{
    uint8_t ad1[3];		// 0x00
    uint8_t ad2[3];		// 0x03
    uint8_t lpf[3];		// 0x06
    uint8_t rms[5];		// 0x09
    uint8_t pkhmin[3];	// 0x0E
    uint8_t pkhmax[3];	// 0x11
    uint8_t ctsta;		// 0x14
    uint8_t ctc[3];		// 0x15
    uint8_t ctb[3];		// 0x18
    uint8_t cta[3];		// 0x1B
    uint8_t intf;		// 0x1E
    uint8_t inte;		// 0x1F
    uint8_t r[23];		// 0x20
} __attribute__((__packed__)) DMMSTS;

typedef struct _DMMCFG{
    int mode;			// scale
    double range;		// full scale range
    uint8_t sw;			// switch bits
    uint8_t cfg[20];	// configuration bits: 0x20...0x33
    double mul;			// dmm measurement (ad1/rms) multiplication factor to get value in corresponding unit
} DMMCFG;

static const DMMCFG dmmcfg[] = {
// Measure type,	FSR,	sw,		  R20, R21, R22, R23, R24, R25, R26, R27, R28, R29, R2A, R2B, R2C, R2D, R2E, R2F, R30, R31, R32, R33,  scale factor
{ DmmDCVoltage,		5e-2,	0x11,	{0xC2,0x21,0x14,0x8B,0x35,0x11,0x08,0x15,0x31,0xF8,0x00,0x00,0x00,0x00,0x08,0x81,0x80,0xC7,0x3C,0xA8}, 125e-3 / 1.8 / 0x800000 },	// 50 mV DC
{ DmmDCVoltage,		5e-1,	0x11,	{0xC2,0x21,0x14,0x8B,0x85,0x11,0x08,0x15,0x31,0xF8,0x00,0x00,0x00,0x00,0x08,0x81,0x80,0xC7,0x33,0xA8}, 125e-2 / 1.8 / 0x800000 },	// 500 mV DC
{ DmmDCVoltage,		5e0,	0x31,	{0xC2,0x21,0x14,0x8B,0x01,0x11,0x08,0x15,0x31,0xF8,0x20,0x00,0x00,0x90,0x28,0xA0,0x80,0xC7,0x33,0xA8}, 125e-1 / 1.8 / 0x800000 },	// 5 V DC
{ DmmDCVoltage,		5e1,	0x31,	{0xC2,0x21,0x14,0x8B,0x01,0x11,0x08,0x15,0x31,0xF8,0x20,0x00,0x00,0x09,0x28,0xA0,0x80,0xC7,0x33,0xA8}, 125e0  / 1.8 / 0x800000 },	// 50 V DC
{ DmmDCVoltage,		5e2,	0x31,	{0xC2,0x21,0x14,0x8B,0x01,0x11,0x08,0x15,0x31,0xF8,0x20,0x00,0x90,0x00,0x28,0xA0,0x80,0xC7,0x33,0xA8}, 125e1  / 1.8 / 0x800000 },	// 500 V DC
{ DmmDCVoltage,		1e3,	0x31,	{0xC2,0x21,0x14,0x8B,0x09,0x01,0x08,0x15,0x31,0xF8,0x20,0x00,0x90,0x00,0x28,0xA0,0x80,0xC7,0x33,0xA8}, 125e1  / 0.9 / 0x800000 },	// 1kV DC

{ DmmACVoltage,		5e-1,	0x11,	{0xC2,0x21,0x00,0x00,0x45,0x00,0x88,0x15,0x31,0xF8,0x00,0x00,0x00,0x00,0x08,0x91,0x80,0xC7,0x3C,0xA0}, 1e-5 },						// 500 mV AC
{ DmmACVoltage,		5e0,	0x31,	{0xC2,0x21,0x00,0x00,0x4D,0x00,0x88,0x15,0x31,0xF8,0x22,0x00,0x00,0xD0,0x88,0xA0,0xE9,0xC7,0x38,0x20}, 1e-4 },						// 5 V AC
{ DmmACVoltage,		5e1,	0x31,	{0xC2,0x21,0x00,0x00,0x4D,0x00,0x88,0x15,0x31,0xF8,0x22,0x00,0x00,0x09,0x28,0xA0,0xFF,0xC7,0x38,0x20}, 1e-3 },						// 50 V AC
{ DmmACVoltage,		5e2,	0x31,	{0xC2,0x21,0x00,0x00,0x4D,0x00,0x88,0x15,0x31,0xF8,0x22,0x00,0x90,0x00,0x28,0xA0,0x80,0xC7,0x38,0x20}, 1e-2 },						// 500 V AC
{ DmmACVoltage,		1e3,	0x31,	{0xC2,0x21,0x00,0x00,0x4D,0x00,0x88,0x15,0x31,0xF8,0x22,0x00,0x90,0x00,0x28,0xA0,0x80,0xC7,0x38,0x20}, 1e-1 },						// 750 V AC

{ DmmDCCurrent,		5e-4,	0x30,	{0xC2,0x21,0x14,0x8B,0x35,0x11,0x08,0x15,0x11,0xF8,0x00,0x00,0x00,0x00,0x00,0xA0,0x80,0xC7,0x3D,0xAC}, 125e-5 / 1.8 / 0x800000 },	// 500uA DC
{ DmmDCCurrent,		5e-3,	0x30,	{0xC2,0x21,0x14,0x8B,0x95,0x11,0x08,0x15,0x11,0xF8,0x00,0x00,0x00,0x00,0x00,0xA0,0x80,0xC7,0x33,0xAC}, 125e-4 / 1.8 / 0x800000 },	// 5mA DC
{ DmmDCCurrent,		5e-2,	0x32,	{0xC2,0x21,0x14,0x8B,0x35,0x11,0x08,0x15,0x11,0xF8,0x00,0x00,0x00,0x00,0x00,0xA0,0x80,0xC7,0x3D,0xAC}, 125e-3 / 1.8 / 0x800000 },	// 50mA DC
{ DmmDCCurrent,		5e-1,	0x32,	{0xC2,0x21,0x14,0x8B,0x95,0x11,0x08,0x15,0x11,0xF8,0x00,0x00,0x00,0x00,0x00,0xA0,0x80,0xC7,0x33,0xAC}, 125e-2 / 1.8 / 0x800000 },	// 500mA DC
{ DmmDCCurrent,		5e0,	0x39,	{0xC2,0x21,0x14,0x8B,0x35,0x11,0x08,0x15,0x11,0xF8,0x00,0x00,0x00,0x00,0x00,0xA0,0x80,0xC7,0x3D,0xAC}, 125e-1 / 1.8 / 0x800000 },	// 5 A DC
{ DmmDCCurrent,		10e0,	0x39,	{0xC2,0x21,0x14,0x8B,0x35,0x01,0x08,0x15,0x11,0xF8,0x00,0x00,0x00,0x00,0x00,0xA0,0x80,0xC7,0x3D,0xAC}, 125e-1 / 0.9 / 0x800000 },	// 10 A DC

{ DmmACCurrent,		5e-4,	0x30,	{0xC2,0x21,0x12,0x00,0x45,0x01,0x88,0x15,0x31,0xF8,0x00,0x00,0x00,0x00,0x00,0xA0,0x80,0xC7,0x3D,0x20}, 1e-8 / 1.08 },				// 500 uA AC
{ DmmACCurrent,		5e-3,	0x30,	{0xC2,0x21,0x12,0x00,0x45,0x00,0x88,0x15,0x31,0xF8,0x00,0x00,0x00,0x00,0x00,0xA0,0x80,0xC7,0x3D,0xA0}, 1e-7 / 2.16 },				// 5 mA AC
{ DmmACCurrent,		5e-2,	0x32,	{0xC2,0x21,0x12,0x00,0x45,0x01,0x88,0x15,0x31,0xF8,0x00,0x00,0x00,0x00,0x00,0xA0,0x80,0xC7,0x3D,0x20}, 1e-6 / 1.08 },				// 50 mA AC
{ DmmACCurrent,		5e-1,	0x32,	{0xC2,0x21,0x12,0x00,0x45,0x00,0x88,0x15,0x31,0xF8,0x00,0x00,0x00,0x00,0x00,0xA0,0x80,0xC7,0x3D,0xA0}, 1e-5 / 2.16 },				// 500 mA AC
{ DmmACCurrent,		5e0,	0x39,	{0xC2,0x21,0x12,0x00,0x45,0x01,0x88,0x15,0x31,0xF8,0x00,0x00,0x00,0x00,0x00,0xA0,0x80,0xC7,0x3D,0x20}, 1e-4 / 1.08 },				// 5 A AC
{ DmmACCurrent,		10e0,	0x39,	{0xC2,0x21,0x12,0x00,0x45,0x00,0x88,0x15,0x31,0xF8,0x00,0x00,0x00,0x00,0x00,0xA0,0x80,0xC7,0x3D,0xA0}, 1e-4 / 2.16 },				// 10 A AC

{ DmmResistance,	5e1,	0x15,	{0x76,0x62,0x13,0x83,0x85,0x01,0x08,0x15,0x00,0xF8,0x00,0x40,0x06,0x00,0x00,0x94,0x80,0xD2,0x3F,0xAC}, 1e3 / 0.9 / 0x800000 },		// 50 Ohm
{ DmmResistance,	5e2,	0x15,	{0xC2,0x21,0x14,0x83,0x85,0x01,0x08,0x15,0x00,0xF8,0x00,0x00,0x06,0x00,0x00,0x94,0x80,0xD2,0x3F,0xAC}, 1e3 / 0.9 / 0x800000 },		// 500 Ohm
{ DmmResistance,	5e3,	0x15,	{0xC2,0x21,0x14,0x83,0x85,0x01,0x08,0x15,0x00,0xF8,0x00,0x00,0x60,0x00,0x00,0x94,0x80,0xD3,0x3F,0xAC}, 1e4 / 0.9 / 0x800000 },		// 5 kOhm
{ DmmResistance,	5e4,	0x15,	{0xC2,0x21,0x14,0x83,0x85,0x01,0x08,0x15,0x00,0xF8,0x00,0x00,0x00,0x06,0x00,0x94,0x80,0xD3,0x3F,0xAC}, 1e5 / 0.9 / 0x800000 },		// 50 kOhm
{ DmmResistance,	5e5,	0x15,	{0xC2,0x21,0x14,0x83,0x85,0x01,0x08,0x15,0x00,0xF8,0x00,0x00,0x00,0x60,0x00,0x94,0x80,0xD3,0x3F,0xAC}, 6e5 / 0.9 / 0x800000 },		// 500 kOhm
{ DmmResistance,	5e6,	0x15,	{0xC2,0x21,0x14,0x93,0x85,0x01,0x08,0x15,0x55,0xF8,0x00,0x00,0x00,0x80,0x00,0x86,0x80,0xD1,0x3F,0xAC}, 6e6 / 0.9 / 0x800000 },		// 5 MOhm
{ DmmResistance,	5e7,	0x15,	{0xC2,0x21,0x14,0x93,0x85,0x01,0x08,0x15,0x55,0xF8,0x00,0x08,0x00,0x00,0x00,0x86,0x80,0xD1,0x3F,0xAC}, 6e7 / 0.9 / 0x800000 },		// 50 MOhm

{ DmmResistance4W,	5e2,	0x15,	{0xC2,0x21,0x14,0x83,0xA7,0x01,0x08,0x15,0x00,0xF8,0x00,0x00,0x06,0x00,0x00,0x94,0x80,0xD2,0x3F,0xAC}, 1e3 / 0.9 / 0x800000 },		// 500 Ohm
{ DmmResistance4W,	5e3,	0x15,	{0xC2,0x21,0x14,0x83,0xA7,0x01,0x08,0x15,0x00,0xF8,0x00,0x00,0x60,0x00,0x00,0x94,0x80,0xD3,0x3F,0xAC}, 1e4 / 0.9 / 0x800000 },		// 5 kOhm
{ DmmResistance4W,	5e4,	0x15,	{0xC2,0x21,0x14,0x83,0xA7,0x01,0x08,0x15,0x00,0xF8,0x00,0x00,0x00,0x06,0x00,0x94,0x80,0xD3,0x3F,0xAC}, 1e5 / 0.9 / 0x800000 },		// 50 kOhm

{ DmmCapacitance,	5e-8,	0x15,	{0x72,0x21,0x14,0x8B,0x01,0x11,0x08,0x15,0x31,0xF8,0x00,0x00,0x00,0x08,0x00,0x9A,0x80,0xD7,0x33,0xA8}, 1e-5 },						// 50nF (Read Mode I)
{ DmmCapacitance,	5e-7,	0x15,	{0x72,0x21,0x14,0x8B,0x01,0x11,0x08,0x15,0x31,0xF8,0x00,0x00,0x00,0x08,0x00,0x9A,0x80,0xD7,0x33,0xA8}, 1e-5 },						// 500nF
{ DmmCapacitance,	5e-6,	0x15,	{0x72,0x21,0x14,0x8B,0x01,0x11,0x08,0x15,0x31,0xF8,0x00,0x00,0x00,0x08,0x00,0x9A,0x80,0xD7,0x33,0xA8}, 1e-5 },						// 5uF
{ DmmCapacitance,	5e-5,	0x15,	{0x72,0x21,0x14,0x8B,0x01,0x11,0x08,0x15,0x31,0xF8,0x00,0x00,0x00,0x08,0x00,0x9A,0x80,0xD7,0x33,0xA8}, 1e-5 },						// 50uF
{ DmmCapacitance,	5e-4,	0x15,	{0x72,0x21,0x14,0x8B,0x01,0x11,0x08,0x15,0x31,0xF8,0x00,0x00,0x00,0x08,0x00,0x9A,0x80,0xD7,0x33,0xA8}, 1e-5 },						// 500uF
{ DmmCapacitance,	5e-3,	0x15,	{0xC0,0x21,0x07,0x93,0x85,0x11,0x08,0x15,0x55,0xF8,0x80,0x00,0x08,0x00,0x00,0x86,0x80,0xD1,0x33,0xAC}, 1e-5 },						// 5mF (Read Mode II)
{ DmmCapacitance,	5e-2,	0x15,	{0x70,0xDE,0x07,0x93,0x85,0x11,0x08,0x15,0x55,0xF8,0x80,0x00,0x08,0x00,0x00,0x8F,0x80,0xDA,0x33,0xAC}, 1e-5 },						// 50mF

{ DmmFrequency,		1e0,	0x05,	{0xC8,0xDE,0x07,0x93,0x85,0x11,0x08,0x15,0x55,0xF8,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0xD7,0x33,0xAC}, 1e0 },						// Frequency

{ DmmTemperature,	1e0,	0x11,	{0xC2,0xDE,0x14,0x8B,0x85,0x11,0x08,0x15,0x31,0xF8,0x00,0x00,0x00,0x00,0x08,0x81,0x80,0xC7,0x33,0xA8}, 125e-2 / 1.8 / 0x800000 },	// Temperature

{ DmmContinuity,	5e2,	0x15,	{0x76,0x62,0x13,0x83,0x85,0x01,0x08,0x15,0x00,0xF8,0x00,0x40,0x06,0x00,0x00,0x94,0x80,0xD2,0x3F,0xAC}, 1e3 / 0.9 / 0x800000 },		// Continuity

{ DmmDiode,			3e0,	0x15,	{0x12,0x62,0x13,0x8B,0x8D,0x11,0x08,0x15,0x11,0xF8,0x00,0x00,0x08,0x00,0x00,0x86,0x80,0xE2,0x33,0xAC}, 1e-6 / 1.08 }				// Diode
};

static const char *dmmranges[] = {
	"DC 50mV", "DC 500mV", "DC 5V", "DC 50V", "DC 500V", "DC 1kV",
	"AC 500mV", "AC 5V", "AC 50V", "AC 500V", "AC 750V",
	"DC 500uA", "DC 5mA", "DC 50mA", "DC 500mA", "DC 5A", "DC 10A",
	"AC 500uA", "AC 5mA", "AC 50mA", "AC 500mA", "AC 5A", "AC 10A",
	"50 Ohm", "500 Ohm", "5 kOhm", "50 kOhm", "500 kOhm", "5 MOhm", "50 MOhm",
	"500 Ohm 4W", "5 kOhm 4W", "50 kOhm 4W",
	"50 nF", "500 nF", "5 uF", "50 uF", "500 uF",
	"5 mF", "50 mF",
	"FREQ",
	"TEMP",
	"Continuity",
	"Diode",
};

CALIB *curCal = NULL;
DMMCFG const *curCfg = NULL;	// pointer to the current configuration
int idxCurrentScale = -1;		// stores the current selected scale
char fUseCalib = 1;				// controls if calibration coefficients should be applied in DMM_DGetStatus

double dMeasuredVal[3];

static DMMSTS curSts;

void DMM_SendCmdSPI( uint8_t cs, uint8_t bCmd, int bytesNumber, uint8_t *pbWrData );
void DMM_GetCmdSPI( uint8_t bCmd, int bytesNumber, uint8_t *pbRdData );

// retrieve value from DMM
double DMM_DGetStatus( uint8_t *pbErr );

void DMM_StartFreqMeasure( void );

// errors
uint8_t DMM_ERR_CheckIdxCalib( int idxScale );

static void DelayAprox10Us( int n )
{
	while( n-- )
	{
		uint16_t i;

		for( i = 0; i < 10; ++i )
		{
			asm volatile ("nop");
		}
	}
}

static void GPIO_SetValue_CS( uint8_t rly, uint8_t state )
{
	HAL_GPIO_WritePin( GPIOB, rly == CS_RLY ? SPI_NSS_RLY : SPI_NSS_DMM, state ? GPIO_PIN_SET : GPIO_PIN_RESET );
	DelayAprox10Us( 150 );
}

static void GPIO_SetValue_CLK( uint8_t state )
{
	HAL_GPIO_WritePin( GPIOB, SPI_SCK, state ? GPIO_PIN_SET : GPIO_PIN_RESET );
	DelayAprox10Us( 30 );
}

static void GPIO_SetValue_MOSI( uint8_t state )
{
	HAL_GPIO_WritePin( GPIOB, SPI_MOSI, state ? GPIO_PIN_SET : GPIO_PIN_RESET );
	DelayAprox10Us( 30 );
}

static uint8_t SPI_CoreTransferByte( uint8_t bCmd )
{
	uint8_t i, val = 0;

	for( i = 0; i < 8; ++i )
	{
		GPIO_SetValue_MOSI( bCmd & 0x80 );
		bCmd <<= 1;
		val = ( val << 1 ) | HAL_GPIO_ReadPin( GPIOB, SPI_MISO );

		GPIO_SetValue_CLK( 1 );

		GPIO_SetValue_CLK( 0 );
	}

	return val;
}

/***	DMM_SendCmdSPI
 **
 **	Parameters:
 **		uint8_t bCmd      - the command byte to be transmitted over SPI
 **		int bytesNumber   - the number of data bytes to be transmitted over SPI
 **		uint8_t *pbWrData - the array of bytes to be transmitted over SPI
 **
 **	Return Value:
 **		none
 **
 **	Description:
 **		This function sends data on a DMM command over the SPI.
 **      It activates DMM Slave Select pin, sends the command byte, and the specified
 **      number of bytes from pbWrData, using the SPI_CoreTransferByte function.
 **      Finally it deactivates the DMM Slave Select pin.
 **
 */
void DMM_SendCmdSPI( uint8_t cs, uint8_t bCmd, int bytesNumber, uint8_t *pbWrData )
{
	if( cs != CS_RLY )
	{
		// Activate CS
		GPIO_SetValue_CS( cs, 0 );
		bCmd <<= 1;				// register number must be sent as (regno << 1 | 0), 0 for read
	}

	int i;
	for( i = 0; i <= bytesNumber; i++ )
	{
		SPI_CoreTransferByte( bCmd );	// Send byte
		bCmd = *pbWrData++;				// Load next byte

		if( cs == CS_RLY )
		{
			GPIO_SetValue_CS( cs, 1 );	// latch strobe HC595
			GPIO_SetValue_CS( cs, 0 );
		}
	}

	if( cs != CS_RLY )
	{
		// Deactivate CS
		GPIO_SetValue_CS( cs, 1 );
	}
}

/***	DMM_GetCmdSPI
 **
 **	Parameters:
 **		uint8_t bCmd      - the command byte to be transmitted over SPI
 **		int bytesNumber         - the number of data bytes to be received over SPI
 **		uint8_t *pbRdData - the array of bytes to store the bytes received over SPI**
 **	Return Value:
 **		none

 **
 **	Description:
 **		This function retrieves data on a DMM command over the SPI.
 **      It activates DMM Slave Select pin, sends the command byte,
 **      and then retrieves the specified number of bytes into pbRdData, using the SPI_CoreTransferByte function.
 **      Finally it deactivates the DMM Slave Select pin.
 **
 */
void DMM_GetCmdSPI( uint8_t bCmd, int bytesNumber, uint8_t *pbRdData )
{
	GPIO_SetValue_CS( CS_DMM, 0 );	// Activate CS_DMM

	// Send command byte
	SPI_CoreTransferByte( bCmd << 1 | 1 );

	// Generate an extra clock (called SPI Read Period) with MOSI = L
	GPIO_SetValue_MOSI( 0 );
	GPIO_SetValue_CLK( 1 );			// set the clock line
	GPIO_SetValue_CLK( 0 );			// reset the clock line

	// Receive the requested number of bytes
	int i;
	for( i = 0; i < bytesNumber; i++ )
	{
		pbRdData[ i ] = SPI_CoreTransferByte( 0 );
	}

	GPIO_SetValue_CS( CS_DMM, 1 );	// Deactivate CS_DMM
}

/***	DMM_ConfigSwitches
 **
 **	Parameters:
 **      uint8_t sw		- byte containing the switches configuration
 **
 **	Return Value:
 **		none
 **
 **	Description:
 **		This function configures the 5 on-board switches/relays according to the
 **		bit pattern from the provided parameter sw.:
 **
 **		Bit 0: U1  (10A / A - input resistors)
 **		Bit 1: RL2 (mA / uA)
 **		Bit 2: RL1 (VA / RLD)
 **		Bit 3: U2  (10A / A - ADC)
 **		Bit 5: U19 (PB0 connected / disconnected)
 **
 */
void DMM_ConfigSwitches( uint8_t sw )
{
	DMM_SendCmdSPI( CS_RLY, sw, 1, &sw );		// send value twice
}

/***	DMM_SetScale
 **	Parameters:
 **      uint8_t idxScale		- the scale index
 **	Return Value:
 **		uint8_t
 **          ERRVAL_SUCCESS            0      // success
 **          ERRVAL_DMM_IDXCONFIG     0xFC    // error, wrong scale index
 **          ERRVAL_DMM_CFGVERIFY     0xF5    // DMM Configuration verify error
 **	Description:
 **		This function configures a specific scale as the current scale.
 **      According to this scale, it uses data defined in dmmcfg structure to configure the switches and
 **      to set the value of the registers (24 registers starting at 0x1F address).
 **      It also verifies the configuration setting success status by reading the values of these registers.
 **      It returns ERRVAL_SUCCESS if the operation is successful.
 **      It returns ERRVAL_DMM_CFGVERIFY if verifying fails.
 **      It returns ERRVAL_DMM_IDXCONFIG if the scale index is not valid.
 */
uint8_t DMM_SetScale( int idxScale )
{
	memset( &curSts, 0, sizeof(curSts) );

	idxCurrentScale = -1;		// invalidate current scale
	curCfg = NULL;				// invalidate pointer to the current configuration
	curCal = NULL;

	// Verify index
	uint8_t bResult = DMM_ERR_CheckIdxCalib( idxScale );
	if( bResult != ERRVAL_SUCCESS ) return bResult;

	// Reset the DMM
#if 1
	curSts.r[0x17] = 0x60;
	DMM_SendCmdSPI( CS_DMM, 0x37, 1, &curSts.r[0x17] );				// write R37=60h ???? This is done by DIGILENT
#endif
	DMM_GetCmdSPI( 0x20, 1, curSts.r );							// read R20
	curSts.r[0] = 0x48;
	DMM_SendCmdSPI( CS_DMM, 0x20, 1, curSts.r );	// write R20 = 48h - SCMPI=010, ENCMP=1, ENCNTI,ENPCMPO,ENCTR=0

	// Retrieve current Scale information (register setting, switch settings, calibration, etc.)
	idxCurrentScale = idxScale;
	curCal = &calib.Dmm[ idxScale ];
	curCfg = &dmmcfg[idxScale];
	memcpy( curSts.r, curCfg->cfg, sizeof(curCfg->cfg) );			// set registers R20..R33
	curSts.ctsta = 0x40;
	curSts.cta[2] = ( DMM_GetCurrentMode() == DmmFrequency ) ? 0xF0 : 0xBF;
	/*
	 * DIGILENT additionally sets the following registers (ONLY for 5VAC !!)
	 * R34=02h - ??? Bit2 should be 0
	 * R35=50h - SDA3IP=01, SAD3IN=01, AD3IG=00 (setup ADC3)
	 * R36=0Ch - SPHACAL=0x0C
	 */

	// Set the switches
	DMM_ConfigSwitches( curCfg->sw );

	// Write INTF and R20..R33
	DMM_SendCmdSPI( CS_DMM, 0x00, sizeof(curSts), (uint8_t*)&curSts );

	// start measure
	if( idxCurrentScale == SCALE_FREQ )
		DMM_StartFreqMeasure();

	return ERRVAL_SUCCESS;
}

/***	DMM_ERR_CheckIdxCalib
 **	Parameters:
 **      uint8_t idxScale		- the scale index
 **	Return Value:
 **		uint8_t
 **          ERRVAL_SUCCESS            0       // success
 **          ERRVAL_DMM_IDXCONFIG     0xFC    // error, wrong scale index
 **	Description:
 **		This function checks the scale index.
 **      If it is valid (between 0 and maximum value of the scale index - 1), the function returns ERRVAL_SUCCESS
 **      If it is not valid, the function returns ERRVAL_DMM_IDXCONFIG.
 */
uint8_t DMM_ERR_CheckIdxCalib( int idxScale )
{
	uint8_t bResult = ( idxScale >= 0 ) && ( idxScale < DMM_CNTSCALES ) ? ERRVAL_SUCCESS : ERRVAL_DMM_IDXCONFIG;
	return bResult;
}

/***	DMM_DGetValue
 **	Parameters:
 **      uint8_t *pbErr - Pointer to the error parameter, the error can be set to:
 **          ERRVAL_SUCCESS              0       // success
 **          ERRVAL_DMM_VALIDDATATIMEOUT 0xFA    // valid data DMM timeout
 **          ERRVAL_DMM_IDXCONFIG        0xFC    // error, wrong current scale index
 **	Return Value:
 **		double
 **          the value computed according to the convertor / RMS registers values, or
 **          NAN (not a number) value if the convertor / RMS registers value is not ready or if ERRVAL_DMM_IDXCONFIG was set, or
 **          +/- INFINITY if the convertor / RMS registers values are outside the expected range.
 **	Description:
 **		This function repeatedly retrieves the value from the convertor / RMS registers
 **      by calling private private function DMM_DGetStatus, until a valid value is detected.
 **      It returns INFINITY when measured values are outside the expected convertor range.
 **      If there is no valid current scale selected, the function sets the error value to ERRVAL_DMM_IDXCONFIG and NAN value is returned.
 **      If there is no valid value retrieved within a specific timeout period, the error is set to ERRVAL_DMM_VALIDDATATIMEOUT.
 **		This function compensates the not linear behavior of VoltageDC50 scale.
 **		When no error is detected, the error is set to ERRVAL_SUCCESS.
 **      The error is copied in the byte pointed by pbErr, if pbErr is not null.
 */
double DMM_DGetValue( uint8_t *pbErr )
{
	uint8_t bErr = ERRVAL_SUCCESS;
	unsigned long cntTimeout = 0;				// valid data timeout counter
	double dVal;

	// wait until a valid value is retrieved or the timeout counter exceeds threshold
	while( DMM_IsNotANumber( dVal = DMM_DGetStatus( &bErr ) ) &&
		( cntTimeout++ < DMM_VALIDDATA_CNTTIMEOUT ) &&
		( bErr == ERRVAL_SUCCESS ) )
		;

	// detect timeout
	if( ( bErr == ERRVAL_SUCCESS ) && ( cntTimeout >= DMM_VALIDDATA_CNTTIMEOUT ) )
		bErr = ERRVAL_DMM_VALIDDATATIMEOUT;

#if 0
	// DIGILENT additionally treats the value in the 50VDC range with a cubic function...
	// "compensate the not linear scale behavior"
	if( bErr == ERRVAL_SUCCESS && DMM_GetCurrentScale() == DMMVoltageDC50Scale )
	{
		dVal = dVal * dVal * dVal * DMM_Voltage50DCLinearCoeff_P3 + dVal * DMM_Voltage50DCLinearCoeff_P1 + dVal * DMM_Voltage50DCLinearCoeff_P0;	// does this make sense?
	}
#endif

	if( pbErr ) *pbErr = bErr;					// set error
	return dVal;
}

/***	DMM_DGetAvgValue
 **	Parameters:
 **      int cbSamples           - The number of values to be used for the average value
 **      uint8_t *pbErr    - Pointer to the error parameter, the error can be set to:
 **          ERRVAL_SUCCESS              0       // success
 **          ERRVAL_DMM_VALIDDATATIMEOUT 0xFA    // valid data DMM timeout
 **          ERRVAL_DMM_IDXCONFIG        0xFC    // error, wrong current scale index
 **	Return Value:
 **		double
 **          the DMM value, or
 **          NAN (not a number) value if errors were detected
 **	Description:
 **		This function computes an average value corresponding to the DMM value
 **      returned by DMM_DGetValue, for the specified number of samples.
 **      The function uses Arithmetic mean average value method for all but AC scales,
 **      and RMS (Quadratic mean) Average value method for for AC scales.
 **      If there is no valid current scale selected, the error is set to ERRVAL_DMM_IDXCONFIG.
 **      If there is no valid value retrieved within a specific timeout period, the error is set to ERRVAL_DMM_VALIDDATATIMEOUT.
 **      It returns INFINITY when measured values are outside the expected convertor range.
 **      When no error is detected, the error is set to ERRVAL_SUCCESS.
 **      The error is copied on the byte pointed by pbErr, if pbErr is not null.
 **      When errors are detected, the function returns NAN.
 */
double DMM_DGetAvgValue( int cbSamples, uint8_t *pbErr )
{
	uint8_t fValid = 1;
	double dValAvg = 0.0, dVal;
	int i, idxScale = DMM_GetCurrentScale();

	uint8_t bErr = DMM_ERR_CheckIdxCalib( idxScale );
	if( bErr == ERRVAL_SUCCESS )
	{
		if( DMM_FACScale( idxScale ) )	// use RMS (Quadratic mean) Average value for AC
		{
			for( i = 0; ( i < cbSamples ) && fValid; ++i )
			{
				dVal = DMM_DGetValue( &bErr );
				fValid = ( bErr == ERRVAL_SUCCESS ) && ( dVal != INFINITY ) && ( dVal != -INFINITY ) && !DMM_IsNotANumber( dVal );
				if( fValid )
					dValAvg += pow( dVal, 2 );
			}
			if( fValid && cbSamples )
			{
				dValAvg /= cbSamples;
				dValAvg = sqrt( dValAvg );
			}
		}
		else if( DMM_FCapacitanceScale( idxScale ) )
		{
			DMM_DGetValue( &bErr );
			if( bErr == ERRVAL_SUCCESS )
				return 0.0;
		}
		else							// use normal (Arithmetic mean) Average value for other than AC.
		{
			for( i = 0; ( i < cbSamples ) && fValid; ++i )
			{
				dVal = DMM_DGetValue( &bErr );
				fValid = ( bErr == ERRVAL_SUCCESS ) && ( dVal != INFINITY ) && ( dVal != -INFINITY ) && !DMM_IsNotANumber( dVal );
				if( fValid )
					dValAvg += dVal;
			}
			if( fValid && cbSamples )
				dValAvg /= cbSamples;
		}
	}
	if( bErr != ERRVAL_SUCCESS ) dValAvg = NAN;
	if( pbErr ) *pbErr = bErr;
	return dValAvg;
}

/***	DMM_GetCurrentScale
 **	Parameters:
 **      none
 **	Return Value:
 **		int     - current scale index
 **              - -1 if no current scale was selected
 **	Description:
 **		This function returns the current scale index.
 **      This is the last scale index selected using the DMM_SetScale function.
 **      In case no current scale was selected, the function returns -1.
 */
int DMM_GetCurrentScale( void )
{
	return idxCurrentScale;
}

/***	DMM_GetCurrentMode
 **	Parameters:
 **      none
 **	Return Value:
 **		int     - current mode index
 **             - -1 if no current scale was selected
 **	Description:
 **		This function returns the current modee index.
 **     In case no current scale was selected, the function returns -1.
 */
int DMM_GetCurrentMode( void )
{
	if( DMM_ERR_CheckIdxCalib( idxCurrentScale ) != ERRVAL_SUCCESS ) return -1;
	return dmmcfg[idxCurrentScale].mode;
}

/***	DMM_GetMode
 **	Parameters:
 **      none
 **	Return Value:
 **		int     - current mode index
 **             - -1 if no current scale was selected
 **	Description:
 **		This function returns the current modee index.
 **     In case no current scale was selected, the function returns -1.
 */
int DMM_GetMode( int idxScale )
{
	if( DMM_ERR_CheckIdxCalib( idxScale ) != ERRVAL_SUCCESS ) return -1;
	return dmmcfg[idxScale].mode;
}

/***	DMM_GetCurrentScale
 **	Parameters:
 **      none
 **	Return Value:
 **		int     - current scale index
 **              - -1 if no current scale was selected
 **	Description:
 **		This function returns the current scale index.
 **      This is the last scale index selected using the DMM_SetScale function.
 **      In case no current scale was selected, the function returns -1.
 */
double DMM_GetRange( int idxScale )
{
	if( DMM_ERR_CheckIdxCalib( idxScale ) != ERRVAL_SUCCESS ) return 0.0;
	return dmmcfg[idxScale].range;
}

double DMM_GetCurrentRange( void )
{
	if( DMM_ERR_CheckIdxCalib( idxCurrentScale ) != ERRVAL_SUCCESS ) return 0.0;
	return dmmcfg[idxCurrentScale].range;
}

/***	DMM_SetUseCalib
 **	Parameters:
 **      uint8_t f
 **              1 if calibrations coefficients should be applied in future DMM_DGetStatus calls
 **              0 if calibrations coefficients should not be applied in future DMM_DGetStatus calls
 **	Return Value:
 **		none
 **	Description:
 **		This function sets the parameter that will determine whether or not the
 **      calibration coefficients will be applied when value is computed in
 **      subsequent DMM_DGetStatus calls.
 **      The default value for this parameter is 1.
 */
void DMM_SetUseCalib( uint8_t f )
{
	fUseCalib = f;
}

char DMM_GetUseCalib( void )
{
	return fUseCalib;
}

/***	DMM_FACScale
 **	Parameters:
 **      int idxScale  - the scale index
 **	Return Value:
 **		1 if the specified scale is a AC (alternating current) scale.
 **		0 if the specified scale is not a AC (alternating current) scale.
 **	Description:
 **		This function checks if the specified scale is an AC (alternating current) scale.
 **      It returns 1 for the AC Voltage, AC current and AC Low current type scales, and 0 otherwise.
 **      The scale type is checked using the mode field in DMMCFG structure.
 */
uint8_t DMM_FACScale( int idxScale )
{
	if( DMM_ERR_CheckIdxCalib( idxScale ) != ERRVAL_SUCCESS ) return 0;
	int mode = dmmcfg[idxScale].mode;
	return ( mode == DmmACVoltage ) || ( mode == DmmACCurrent );
}

/* ************************************************************************** */
/***	DMM_FDCScale
 **	Parameters:
 **      int idxScale  - the scale index
 **	Return Value:
 **		1 if the specified scale is a DC (direct current) type scale.
 **		0 if the specified scale is not a DC (direct current) type scale.
 **	Description:
 **		This function checks if the specified scale is a DC (direct current) type scale.
 **      It returns 1 for the DC Voltage, DC current and DC Low current type scales, and 0 otherwise.
 **      The scale type is checked using the mode field in DMMCFG structure.
 */
uint8_t DMM_FDCScale( int idxScale )
{
	if( DMM_ERR_CheckIdxCalib( idxScale ) != ERRVAL_SUCCESS ) return 0;
	int mode = dmmcfg[idxScale].mode;
	return ( mode == DmmDCVoltage ) || ( mode == DmmDCCurrent );
}

/***	DMM_FResistorScale
 **	Parameters:
 **      int idxScale  - the scale index
 **	Return Value:
 **		1 if the specified scale is a Resistor type scale.
 **		0 if the specified scale is not a Resistor type scale.
 **	Description:
 **		This function checks if the specified scale is a Resistor type scale.
 **      It returns 1 for the Resistor and Continuity type scales, and 0 otherwise.
 **      The scale type is checked using the mode field in DMMCFG structure.
 */
uint8_t DMM_FResistorScale( int idxScale )
{
	if( DMM_ERR_CheckIdxCalib( idxScale ) != ERRVAL_SUCCESS ) return 0;
	int mode = dmmcfg[idxScale].mode;
	return ( mode == DmmResistance ) || ( mode == DmmResistance4W ) || ( mode == DmmContinuity );
}

/***	DMM_FCapacitanceScale
 **	Parameters:
 **      int idxScale  - the scale index
 **	Return Value:
 **		1 if the specified scale is a Capacitance type scale.
 **		0 if the specified scale is not a Capacitance type scale.
 **	Description:
 **		This function checks if the specified scale is a Capacitance type scale.
 **      It returns 1 for the Capacitance type scales, and 0 otherwise.
 **      The scale type is checked using the mode field in DMMCFG structure.
 */
uint8_t DMM_FCapacitanceScale( int idxScale )
{
	if( DMM_ERR_CheckIdxCalib( idxScale ) != ERRVAL_SUCCESS ) return 0;
	int mode = dmmcfg[idxScale].mode;
	return ( mode == DmmCapacitance );
}

/***	DMM_FDiodeScale
 **	Parameters:
 **      int idxScale  - the scale index
 **	Return Value:
 **		1 if the specified scale is a Diode type scale.
 **		0 if the specified scale is not a Diode type scale.
 **	Description:
 **		This function checks if the specified scale is a Diode type scale.
 **      It returns 1 for the Diode and Continuity type scales, and 0 otherwise.
 **      The scale type is checked using the mode field in DMMCFG structure.
 */
uint8_t DMM_FDiodeScale( int idxScale )
{
	if( DMM_ERR_CheckIdxCalib( idxScale ) != ERRVAL_SUCCESS ) return 0;
	int mode = dmmcfg[idxScale].mode;
	return ( mode == DmmDiode );
}

/***	DMM_FContinuityScale
 **	Parameters:
 **      int idxScale  - the scale index
 **	Return Value:
 **		1 if the specified scale is a Continuity type scale.
 **		0 if the specified scale is not a Continuity type scale.
 **	Description:
 **		This function checks if the specified scale is a Continuity type scale.
 **      It returns 1 for the Continuity and Continuity type scales, and 0 otherwise.
 **      The scale type is checked using the mode field in DMMCFG structure.
 */
uint8_t DMM_FContinuityScale( int idxScale )
{
	if( DMM_ERR_CheckIdxCalib( idxScale ) != ERRVAL_SUCCESS ) return 0;
	int mode = dmmcfg[idxScale].mode;
	return ( mode == DmmContinuity );
}

/***	DMM_FDCCurrentScale
 **
 **	Parameters:
 **      < none>
 **
 **
 **	Return Value:
 **		1 if the current scale is a DC Current type scale.
 **		0 if the current scale is not a DC Current type scale.
 **
 **	Description:
 **		This function checks if the specified scale is a DC Current type scale.
 **      It returns 1 for the DC Current type scales, and 0 otherwise.
 **      The scale type is checked using the mode field in DMMCFG structure.
 **
 */
uint8_t DMM_FDCCurrentScale()
{
	if( DMM_ERR_CheckIdxCalib( idxCurrentScale ) != ERRVAL_SUCCESS ) return 0;
	int mode = curCfg->mode;
	return ( mode == DmmDCCurrent );
}

/***	DMM_IsNotANumber
 **	Parameters:
 **      double dVal  - the value to be checked
 **	Return Value:
 **		1 if the specified value is "not a number".
 **		0 if the specified scale is a valid number.
 **	Description:
 **		This function checks if the specified value contains a "not a number" value.
 **      The "not a number" values are used to identify the abnormal situations like the data not available in the DMM converters.
 **      It returns 1 if the value is a "not a number" value, and 0 if the value corresponds to a valid number.
 */
uint8_t DMM_IsNotANumber( double dVal )
{
	return ( dVal == NAN ) || isnan( dVal );
}

/***	DMM_GetScaleUnit
 **	Parameters:
 **		int idxScale        - the Scale index
 **      double *pdScaleFact - pointer to a variable to get the scale factor value
 **      char *szUnitPrefix  - string to get the Unit prefix corresponding to the multiple / submultiple
 **      char *szUnit        - string to get the Unit
 **	Return Value:
 **		uint8_t
 **          ERRVAL_SUCCESS                  0       // success
 **          ERRVAL_DMM_IDXCONFIG            0xFC    // wrong scale index
 **	Description:
 **		The function identifies the Measuring unit data (scale factor, Unit prefix and Unit) for the specified scale.
 **      It uses scale type to identify the Unit.
 **      It uses scale range to identify the Unit prefix (u, m, k, M) and the corresponding scale factor.
 **      The scale factor is the value that must multiply the value to convert from the base Unit to the prefixed unit (for example from V to mV)
 **      The function returns ERRVAL_DMM_IDXCONFIG if the provided Scale is not valid.
 */
uint8_t DMM_GetScaleUnit( int idxScale, double *pdScaleFact, char *szUnitPrefix, char *szUnit, char *szRange )
{
	uint8_t bResult = DMM_ERR_CheckIdxCalib( idxScale );
	if( bResult == ERRVAL_SUCCESS )				// valid idxScale
	{
		DMMCFG const *cfg = &dmmcfg[idxScale];

		if( pdScaleFact && szUnitPrefix )
		{
			if(		 cfg->range < 1e-9 )	{ *pdScaleFact = 1e12; strcpy( szUnitPrefix, "p" ); }
			else if( cfg->range < 1e-6 )	{ *pdScaleFact = 1e9; strcpy( szUnitPrefix, "n" ); }
			else if( cfg->range < 1e-3 )	{ *pdScaleFact = 1e6; strcpy( szUnitPrefix, "u" ); }
			else if( cfg->range < 1e0 )		{ *pdScaleFact = 1e3; strcpy( szUnitPrefix, "m" ); }
			else if( cfg->range < 1e3 )		{ *pdScaleFact = 1e0; szUnitPrefix[0] = 0; }
			else if( cfg->range < 1e6 )		{ *pdScaleFact = 1e-3; strcpy( szUnitPrefix, "k" ); }
			else if( cfg->range < 1e9 )		{ *pdScaleFact = 1e-6; strcpy( szUnitPrefix, "M" ); }
			else							{ *pdScaleFact = 1e-9; strcpy( szUnitPrefix, "G" ); }
		}

		if( szUnit )
		{
			switch( cfg->mode )
			{
			case DmmDCVoltage:		strcpy( szUnit, "V DC" ); break;
			case DmmACVoltage:		strcpy( szUnit, "V AC" ); break;
			case DmmDiode:			strcpy( szUnit, "V" ); break;
			case DmmDCCurrent:		strcpy( szUnit, "A DC" ); break;
			case DmmACCurrent:		strcpy( szUnit, "A AC" ); break;
			case DmmResistance:
			case DmmResistance4W:
			case DmmContinuity:		strcpy( szUnit, "Ohm" ); break;
			case DmmTemperature:	strcpy( szUnit, "C" ); break;
			case DmmCapacitance:	strcpy( szUnit, "F" ); break;
			case DmmFrequency:		strcpy( szUnit, "Hz" ); break;
			}
		}

		if( szRange )
		{
			strcpy( szRange, dmmranges[idxScale] );
		}
	}
	return bResult;
}

uint8_t SPrintfDouble( char *pString, double dVal, uint8_t precision )
{
	uint8_t i, lenInt;
	uint32_t fract, precisionFactor;

	// 1. Print the integer part
	lenInt = sprintf( pString, "%+d", (int)dVal );

	// 2. Print decimal point
	pString[ lenInt ] = '.';

	// 3. Print the fractional part
	// 3.1. Compute precision factor (10 power precision)
	precisionFactor = 1;
	for( i = 0; i < precision; i++ )
	{
		precisionFactor *= 10;
	}

	// 3.2. Compute fractional part as integer (fractional part multiplied by precision factor)
	if( dVal >= 0 )
		fract = ( dVal - (int)dVal ) * precisionFactor;
	else
		fract = ( (int)dVal - dVal ) * precisionFactor;

	// 3.3. Print the fractional part in the string, starting with the right part and padding left (until decimal point) with leading 0s.
	// For example if fract = 12, then the string will be filled with 000012 after decimal point
	for( i = precision; i > 0; i-- )
	{
		pString[ lenInt + i ] = '0' + ( fract % 10 );
		fract /= 10;
	}

	// 4. Place the terminating 0;
	pString[ lenInt + precision + 1 ] = 0;
	return lenInt + precision + 1;
}


/***	DMM_FormatValue
 **	Parameters:
 **		double dVal         - The value to be formatted
 **     char *pString       - The string to get the formatted value
 **     uint8_t fUnit       - flag to indicate if unit information should be added
 **                 0       - do not add unit
 **                 not 0   - add unit / subunit
 **	Return Value:
 **		uint8_t
 **          ERRVAL_SUCCESS                  0       // success
 **          ERRVAL_DMM_IDXCONFIG            0xFC    // wrong scale index
 **	Description:
 **		The function formats a value according to the current selected scale.
 **      The parameter value dVal must correspond to the base Unit (V, A or Ohm), mainly the value returned by DMM_DGetValue.
 **      The function multiplies the value according to the scale specific multiple / submultiple.
 **      It formats the value with 6 decimals.
 **      If fUnit is not 0 it adds the measure unit text (including multiple / submultiple) corresponding to the scale.
 **      If dVal is +/- INFINITY (converter values are outside expected range), then "OVERLOAD" string is used for all scales except Continuity.
 **      If dVal is +/- INFINITY (converter values are outside expected range), then "OPEN" string is used for Continuity scale.
 **      The function returns ERRVAL_DMM_IDXCONFIG if the current scale is not valid.
 **      For example it formats the value 0.0245678912 into the "24.678912 mV" if the current scale is VoltageDC50m.
 */
uint8_t DMM_FormatValue( double dVal, char *pString, uint8_t fUnit )
{
	// default 6 decimals
	double dScaleFact;
	char szUnitPrefix[ 2 ], szUnit[ 5 ], szRange[ 12 ];

	uint8_t bResult = DMM_ERR_CheckIdxCalib( idxCurrentScale );
	if( bResult == ERRVAL_SUCCESS )
	{
		if( ( dVal == INFINITY ) || ( dVal == -INFINITY ) )
		{
			strcpy( pString, ( curCfg->mode == DmmContinuity ) ? "OPEN" : "OVERLOAD" );
		}
		else if( curCfg->mode == DmmDiode && dVal > DMM_DIODEOPENTHRESHOLD )
		{
			strcpy( pString, "OPEN" );
		}
		else
		{
			bResult = DMM_GetScaleUnit( idxCurrentScale, &dScaleFact, szUnitPrefix, szUnit, szRange );
			if( bResult == ERRVAL_SUCCESS )			// valid idxScale
			{
				dVal *= dScaleFact;
				SPrintfDouble( pString, dVal, 6 );
				if( fUnit )
				{
					strcat( pString, " " );
					strcat( pString, szUnitPrefix );
					strcat( pString, szUnit );
				}
			}
		}
	}
	return bResult;
}

void DMM_StartFreqMeasure( void )
{
	uint8_t i;

	i = 0xC8;
	DMM_SendCmdSPI( CS_DMM, 0x20, 1, &i );	// write R20=C8h - SCMPI=110, ENCMP=0, ENCNTI=1, ENPCMPO=0, ENCTR=0 (stop counters, reset CTA<7:0>)

	curSts.cta[0] = 0x00;										// GateTime = (0x1000000 - CTA) / FSysClk		[FSysClk = 4.9152MHz]
	curSts.cta[1] = 0x00;										// CTA = 0x1000000 - Tgate * FsysClk			[Tgate in s]
	curSts.cta[2] = 0xB5;										// => 1s: 0xB50000
	DMM_SendCmdSPI( CS_DMM, 0x1B, 3, (uint8_t*)&curSts.cta );	// write CTA

	i = 0xCA;
	DMM_SendCmdSPI( CS_DMM, 0x20, 1, &i );	// write R20=CAh - SCMPI=110, ENCMP=0, ENCNTI=1, ENPCMPO=0, ENCTR=1 (start counting)
}

/***	DMM_DGetStatus
 **
 **	Parameters:
 **      uint8_t *pbErr - Pointer to the error parameter, the error can be set to:
 **          ERRVAL_SUCCESS           0       // success
 **          ERRVAL_DMM_IDXCONFIG     0xFC    // error, wrong current scale index
 **
 **	Return Value:
 **		double
 **          the value computed according to the convertor / RMS registers values, or
 **          NAN (not a number) value if the convertor / RMS registers value is not ready or if ERRVAL_DMM_IDXCONFIG was set, or
 **          +/- INFINITY if the convertor / RMS registers values are outside the expected range.
 **	Description:
 **		This function reads the value of the convertor / RMS registers (0-0x1F).
 **      Then, it computes the value corresponding to the convertor / RMS registers, according to the current selected scale.
 **      Depending on the parameter set by DMM_SetUseCalib (default is 1), calibration parameters will be applied on the computed value.
 **      It returns NAN (not a number) when data is not available (ready) in the convertor registers.
 **      It returns INFINITY when values are outside the expected convertor range.
 **      If there is no valid current scale selected, the function sets error to ERRVAL_DMM_IDXCONFIG and NAN value is returned.
 **      When no error is detected, the error is set to ERRVAL_SUCCESS.
 **      The error is copied on the byte pointed by pbErr, if pbErr is not null.
 **
 **
 */
double DMM_DGetStatus( uint8_t *pbErr )
{
	double v = NAN;
	uint8_t i;
	int64_t cnt;

	// Verify index
	uint8_t bResult = DMM_ERR_CheckIdxCalib( idxCurrentScale );
	if( bResult != ERRVAL_SUCCESS )
	{
		if( pbErr ) *pbErr = bResult;
		return NAN;
	}

	// Compute values, according to the specific scale
	if( DMM_FACScale( idxCurrentScale ) )					// AC uses RMS
	{
		int64_t vrms = 0;

		// read INTF register (0x1E) to check for new measurement
		DMM_GetCmdSPI( 0x1E, 1, &i );
		if( !( i & 0x10 ) )									// RMSF=0 : conversion not yet ready
		{
			if( pbErr ) *pbErr = bResult;
			return NAN;
		}

		// read registers 0x09 - 0x0D (RMS)
		DMM_GetCmdSPI( 0x09, 5, (uint8_t*)&curSts.rms );

		for( i = 4; i < 5; --i ) vrms = ( vrms << 8 ) | curSts.rms[i];

		if( fUseCalib )									// apply calibration coefficients
			v = sqrt( fabs( pow( curCfg->mul, 2 ) * (double)vrms - pow( curCal->Add, 2 ) ) ) * curCal->Mult;
		else
			v = curCfg->mul * sqrt( (double)vrms );
	}
	else if( idxCurrentScale >= SCALE_50_nF &&			// Read Mode I
			 idxCurrentScale <= SCALE_500_uF )
	{
		// read INTF register (0x1E) to check for new measurement
		DMM_GetCmdSPI( 0x1E, 1, &i );
		if( !( i & 0x01 ) )									// CTF=0 : conversion not yet ready
		{
			if( pbErr ) *pbErr = bResult;
			return NAN;
		}

		/*
		59988133 HY3131: MOSI transfers: 37 FF FF FF                 # Read CTA
		59993123 HY3131: MOSI transfers: 31 FF FF FF                 # Read CTB
		59998112 HY3131: MOSI transfers: 2B FF FF FF                 # Read CTC
		60001026 HY3131: MOSI transfers: 29 FF                       # Read CTSTA

		60004329 HY3131: MOSI transfers: 40 70                       # Write R20
		60010740 HY3131: MOSI transfers: 36 00 00 BF                 # Write CTA
		60014041 HY3131: MOSI transfers: 40 72                       # Write R20
		 */

#if 0
		DMM_GetCmdSPI( 0x1B, 3, dmmsts.cta );				// read CTA
		DMM_GetCmdSPI( 0x18, 3, dmmsts.ctb );				// read CBA
		DMM_GetCmdSPI( 0x15, 3, dmmsts.cta );				// read CTC
		DMM_GetCmdSPI( 0x14, 1, &dmmsts.ctsta );			// read CTSTA
#else
		DMM_GetCmdSPI( 0x14, 10, &curSts.ctsta );			// read CTA, CTB, CTC and STA
#endif
		// TODO - NOT WORKING !!!
		// ---
		for( cnt = 0, i = 2; i < 3; --i ) cnt = ( cnt << 8 ) | curSts.cta[i];
		v = curCfg->mul * cnt;

		if( fUseCalib )										// apply calibration coefficients
			v = v * curCal->Mult + curCal->Add;
		// ---

		// reset CTA
		i = 0x70; DMM_SendCmdSPI( CS_DMM, 0x20, 1, &i );	// write R20=70h - SCMPI=011, ENCMP=1, ENCNTI=0, ENPCMPO=0, ENCTR=0 (disable counters)
		curSts.cta[0] = 0x00;
		curSts.cta[1] = 0x00;
		curSts.cta[2] = 0xBF;
		DMM_SendCmdSPI( CS_DMM, 0x1B, 3, curSts.cta );		// write CTA=BF0000h (preset counter)
		i = 0x72; DMM_SendCmdSPI( CS_DMM, 0x20, 1, &i );	// write R20=72h - SCMPI=011, ENCMP=1, ENCNTI=0, ENPCMPO=0, ENCTR=1 (re-enable counters)

		// for dev debug
		for( cnt = 0, i = 2; i < 3; --i ) cnt = ( cnt << 8 ) | curSts.ctb[i];
		dMeasuredVal[1] = (double)cnt;

		for( cnt = 0, i = 2; i < 3; --i ) cnt = ( cnt << 8 ) | curSts.ctc[i];
		dMeasuredVal[2] = (double)cnt;
	}
	else if( idxCurrentScale >= SCALE_5_mF &&				// Read Mode II
			 idxCurrentScale <= SCALE_50_mF )
	{
		/*
		63781624 HY3131: MOSI transfers: 54 00                       # Write R2a
		63784911 HY3131: MOSI transfers: 58 00                       # Write R2c
		64284104 74HC595: MOSI transfers: 15

		64286889 HY3131: MOSI transfers: 41 FF                       # Read R20 -> 39
		64290190 HY3131: MOSI transfers: 40 4C                       # Write R20

		64293106 HY3131: MOSI transfers: 41 FF                       # Read R20 -> 26
		64296409 HY3131: MOSI transfers: 40 48                       # Write R20

		64299713 HY3131: MOSI transfers: 54 00                       # Write R2a
		64303015 HY3131: MOSI transfers: 58 00                       # Write R2c

		64305930 HY3131: MOSI transfers: 3D FF                       # Read INTF -> 02
		64310919 HY3131: MOSI transfers: 01 FF FF FF                 # Read AD1_DATA -> EC A2 00
		*/

		i = 0x00; DMM_SendCmdSPI( CS_DMM, 0x2A, 1, &i );	// write R2A=00h - P0=0000 P1=0000
		i = 0x00; DMM_SendCmdSPI( CS_DMM, 0x2C, 1, &i );	// write R2C=00h - P4=0000 P5=0000

		DMM_GetCmdSPI( 0x20, 1, curSts.r );					// read R20
		i = 0x4C; DMM_SendCmdSPI( CS_DMM, 0x20, 1, &i );	// write R20=4Ch - SCMPI=000, ENCMP=1, ENCNTI=1, ENPCMPO=1, ENCTR=0

		DMM_GetCmdSPI( 0x20, 1, curSts.r );					// read R20
		i = 0x48; DMM_SendCmdSPI( CS_DMM, 0x20, 1, &i );	// write R20=48h - SCMPI=000, ENCMP=1, ENCNTI=1, ENPCMPO=0, ENCTR=0

		i = 0x00; DMM_SendCmdSPI( CS_DMM, 0x2A, 1, &i );	// write R2A=00h again. TODO: do we need to do this?
		i = 0x00; DMM_SendCmdSPI( CS_DMM, 0x2C, 1, &i );	// write R2C=00h again. TODO: do we need to do this?

		// TODO - NOT WORKING !!!
		// ---
		// read INTF register (0x1E) to check for new measurement
		for(;;)
		{
			DMM_GetCmdSPI( 0x1E, 1, &i );
			if( ( i & 0x04 ) )								// conversion ready ?
				break;
		}

		DMM_GetCmdSPI( 0x00, 3, (uint8_t*)&curSts );		// read AD1

		// AD1 is a 24-bit SIGNED value, so load it left justified and then divide by 256 to keep the sign
		int32_t vad1 = ( (int32_t)curSts.ad1[ 2 ] << 24 )
					 | ( (int32_t)curSts.ad1[ 1 ] << 16 )
					 | ( (int32_t)curSts.ad1[ 0 ] << 8 );
		vad1 /= 256;

		if( vad1 >= 0x7FFFFE )		 v = +INFINITY;		// value outside convertor range
		else if( vad1 <= -0x7FFFFE ) v = -INFINITY;		// value outside convertor range
		else
		{
			v = curCfg->mul * vad1;

			if( fUseCalib )					// apply calibration coefficients
				v = v * curCal->Mult + curCal->Add;
		}
		// ---
	}
	else if( idxCurrentScale == SCALE_FREQ )
	{
		uint32_t val;

		// read INTF register (0x1E) to check for new measurement
		DMM_GetCmdSPI( 0x1E, 1, &i );
		if( !( i & 0x01 ) )									// CTF=0 : conversion not yet ready
		{
			if( pbErr ) *pbErr = bResult;
			return NAN;
		}

#if 0
		DMM_GetCmdSPI( 0x1B, 3, dmmsts.cta );				// read CTA
		DMM_GetCmdSPI( 0x18, 3, dmmsts.ctb );				// read CBA
		DMM_GetCmdSPI( 0x15, 3, dmmsts.cta );				// read CTC
		DMM_GetCmdSPI( 0x14, 1, &dmmsts.ctsta );			// read CTSTA
#else
		DMM_GetCmdSPI( 0x14, 10, &curSts.ctsta );			// read CTA, CTB, CTC and STA
#endif
		if( curSts.ctsta & 1 )								// CTBOV=1 : overflow
		{
			if( pbErr ) *pbErr = bResult;
			return NAN;
		}

		for( val = 0, i = 2; i < 3; --i ) val = ( val << 8 ) | curSts.cta[i];	// assemble CTA value
		uint32_t T1 = 0x4B0000 + val;

		if( T1 == 0 ) T1 = 1;

		for( val = 0, i = 2; i < 3; --i ) val = ( val << 8 ) | curSts.ctb[i];	// assemble CTB value
		v = (uint64_t)val * 4915200 / T1;

		for( val = 0, i = 2; i < 3; --i ) val = ( val << 8 ) | curSts.ctc[i];	// assemble CTC value
		dMeasuredVal[1] = (double)val / T1;				// duty cycle

		DMM_StartFreqMeasure();
	}
	else												// DC, RES, DIODE, CONT, TEMP use AD1
	{
		// read INTF register (0x1E) to check for new measurement
		DMM_GetCmdSPI( 0x1E, 1, &i );
		if( !( i & 0x04 ) )								// conversion not yet ready
		{
			if( pbErr ) *pbErr = bResult;
			return NAN;
		}

		// read registers 0x00 - 0x03 (AD1)
		DMM_GetCmdSPI( 0x00, 3, (uint8_t*)&curSts );

		// AD1 is a 24-bit SIGNED value, so load it left justified and then divide by 256 to keep the sign
		int32_t vad1 = ( (int32_t)curSts.ad1[ 2 ] << 24 )
					 | ( (int32_t)curSts.ad1[ 1 ] << 16 )
					 | ( (int32_t)curSts.ad1[ 0 ] << 8 );
		vad1 /= 256;

		if( vad1 >= 0x7FFFFE )		 v = +INFINITY;		// value outside convertor range
		else if( vad1 <= -0x7FFFFE ) v = -INFINITY;		// value outside convertor range
		else
		{
			v = curCfg->mul * vad1;

			if( fUseCalib )					// apply calibration coefficients
				v = v * curCal->Mult + curCal->Add;
		}
	}

	if( pbErr ) *pbErr = ERRVAL_SUCCESS;
	return v;
}

/***	DMMCMD_CmdMeasure
 **
 **	Parameters:
 **     uint8_t	fRaw - take raw measurement
 **     uint8_t fAvg - average mesurement
 **
 **	Return Value:
 **		uint8_t     - the error code
 **          ERRVAL_SUCCESS              0       // success
 **          ERRVAL_DMM_VALIDDATATIMEOUT 0xFA    // valid data DMM timeout
 **          ERRVAL_DMM_IDXCONFIG        0xFC    // error, wrong current scale index
 **
 **	Description:
 **		This function implements the repeated session functionality for DMMMeasureRep and DMMMeasureRaw text commands of DMMCMD module.
 **		The function calls the DMM_DGetValue, eventually without calibration parameters being applied for DMMMeasureRaw.
 */
uint8_t DMM_Measure( uint8_t fRaw, uint8_t fAvg )
{
	uint8_t bErrCode = ERRVAL_SUCCESS;

	char fOldUseCalib = fUseCalib;
	if( fRaw ) fUseCalib = 0;

	dMeasuredVal[0] = fAvg > 1 ? DMM_DGetAvgValue( fAvg, &bErrCode ) : DMM_DGetValue( &bErrCode );

	fUseCalib = fOldUseCalib;

	return bErrCode;
}

void DMM_Init( void )
{
	GPIO_InitTypeDef GPIO_InitStruct = { .Speed = GPIO_SPEED_FREQ_HIGH, .Pull = GPIO_NOPULL };

	/* Configure all SPI GPIOs as outputs, except MISO (PB14) */
	GPIO_InitStruct.Pin = SPI_NSS_DMM | SPI_NSS_RLY | SPI_SCK | SPI_MOSI;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );

	HAL_GPIO_WritePin( GPIOB, SPI_NSS_DMM, GPIO_PIN_SET );
	HAL_GPIO_WritePin( GPIOB, SPI_NSS_RLY, GPIO_PIN_RESET );
	HAL_GPIO_WritePin( GPIOB, SPI_MOSI, GPIO_PIN_RESET );
	HAL_GPIO_WritePin( GPIOB, SPI_SCK, GPIO_PIN_RESET );

	/* Configure MISO as input */
	GPIO_InitStruct.Pin = SPI_MISO;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );

	CALIB_Init();
}

/* *****************************************************************************
 End of File
 */
