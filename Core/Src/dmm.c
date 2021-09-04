#include <dmm.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "stm32f1xx_hal.h"

#include "scpi.h"
#include "calib.h"

#define CALIB_ACCEPTANCE_DEFAULT    0.2

#define SPI_NSS_RLY		GPIO_PIN_11		// SPI2_NSS2 (relay latch)
#define SPI_NSS_DMM		GPIO_PIN_12		// SPI2_NSS1 (DMM HY3131)
#define SPI_SCK			GPIO_PIN_13		// SPI2_SCK
#define SPI_MISO		GPIO_PIN_14		// SPI2_MISO
#define SPI_MOSI		GPIO_PIN_15		// SPI2_MOSI

#define CS_DMM			0
#define CS_RLY			1

const DMMCFG dmmcfg[] = {
// Measure type,	FSR,	fmt,		sw,		 INTE  R20, R21, R22, R23, R24, R25, R26, R27, R28, R29,  R2A, R2B, R2C, R2D, R2E,  R2F, R30, R31, R32, R33,   mult
{ DmmDCVoltage,		5e-2,	"%+7.3f",	0x01,	{0x04, 0xC0,0x21,0x14,0x8B,0x35,0x11,0x08,0x15,0x31,0xF8, 0x00,0x00,0x00,0x00,0x08, 0x81,0x80,0xC7,0x3C,0xA8}, 125e-3 / 1.8 / 0x800000 },	// 50 mV DC
{ DmmDCVoltage,		5e-1,	"%+7.2f",	0x01,	{0x04, 0xC0,0x21,0x14,0x8B,0x85,0x11,0x08,0x15,0x31,0xF8, 0x00,0x00,0x00,0x00,0x08, 0x81,0x80,0xC7,0x33,0xA8}, 125e-2 / 1.8 / 0x800000 },	// 500 mV DC
{ DmmDCVoltage,		5e0,	"%+7.4f",	0x21,	{0x04, 0xC0,0x21,0x14,0x8B,0x01,0x11,0x08,0x15,0x31,0xF8, 0x20,0x00,0x00,0x90,0x28, 0xA0,0x80,0xC7,0x33,0xA8}, 125e-1 / 1.8 / 0x800000 },	// 5 V DC
{ DmmDCVoltage,		5e1,	"%+7.3f",	0x21,	{0x04, 0xC0,0x21,0x14,0x8B,0x01,0x11,0x08,0x15,0x31,0xF8, 0x20,0x00,0x00,0x09,0x28, 0xA0,0x80,0xC7,0x33,0xA8}, 125e0  / 1.8 / 0x800000 },	// 50 V DC
{ DmmDCVoltage,		5e2,	"%+7.2f",	0x21,	{0x04, 0xC0,0x21,0x14,0x8B,0x01,0x11,0x08,0x15,0x31,0xF8, 0x20,0x00,0x90,0x00,0x28, 0xA0,0x80,0xC7,0x33,0xA8}, 125e1  / 1.8 / 0x800000 },	// 500 V DC
{ DmmDCVoltage,		1e3,	"%+7.4f",	0x21,	{0x04, 0xC0,0x21,0x14,0x8B,0x09,0x01,0x08,0x15,0x31,0xF8, 0x20,0x00,0x90,0x00,0x28, 0xA0,0x80,0xC7,0x33,0xA8}, 125e1  / 0.9 / 0x800000 },	// 1kV DC

{ DmmACVoltage,		5e-1,	"%7.2f",	0x01,	{0x11, 0x10,0xDD,0x00,0x00,0x45,0x00,0x88,0x15,0x31,0xF8, 0x00,0x00,0x00,0x00,0x08, 0x91,0x80,0xC7,0x3C,0xA0}, 1e-5 },						// 500 mV AC
{ DmmACVoltage,		5e0,	"%7.4f",	0x21,	{0x11, 0x10,0xDD,0x00,0x00,0x4D,0x00,0x88,0x15,0x31,0xF8, 0x22,0x00,0x00,0xD0,0x88, 0xA0,0xE9,0xC7,0x38,0x20}, 1e-4 },						// 5 V AC
{ DmmACVoltage,		5e1,	"%7.3f",	0x21,	{0x11, 0x10,0xDD,0x00,0x00,0x4D,0x00,0x88,0x15,0x31,0xF8, 0x22,0x00,0x00,0x09,0x28, 0xA0,0xFF,0xC7,0x38,0x20}, 1e-3 },						// 50 V AC
{ DmmACVoltage,		5e2,	"%7.2f",	0x21,	{0x11, 0x10,0xDD,0x00,0x00,0x4D,0x00,0x88,0x15,0x31,0xF8, 0x22,0x00,0x90,0x00,0x28, 0xA0,0x80,0xC7,0x38,0x20}, 1e-2 },						// 500 V AC
{ DmmACVoltage,		1e3,	"%7.2f",	0x21,	{0x11, 0x10,0xDD,0x00,0x00,0x4D,0x00,0x88,0x15,0x31,0xF8, 0x22,0x00,0x90,0x00,0x28, 0xA0,0x80,0xC7,0x38,0x20}, 1e-1 },						// 750 V AC

{ DmmDCCurrent,		5e-4,	"%+7.2f",	0x20,	{0x04, 0xC0,0x21,0x14,0x8B,0x35,0x11,0x08,0x15,0x11,0xF8, 0x00,0x00,0x00,0x00,0x00, 0xA0,0x80,0xC7,0x3D,0xAC}, 125e-5 / 1.8 / 0x800000 },	// 500uA DC
{ DmmDCCurrent,		5e-3,	"%+7.4f",	0x20,	{0x04, 0xC0,0x21,0x14,0x8B,0x95,0x11,0x08,0x15,0x11,0xF8, 0x00,0x00,0x00,0x00,0x00, 0xA0,0x80,0xC7,0x33,0xAC}, 125e-4 / 1.8 / 0x800000 },	// 5mA DC
{ DmmDCCurrent,		5e-2,	"%+7.3f",	0x22,	{0x04, 0xC0,0x21,0x14,0x8B,0x35,0x11,0x08,0x15,0x11,0xF8, 0x00,0x00,0x00,0x00,0x00, 0xA0,0x80,0xC7,0x3D,0xAC}, 125e-3 / 1.8 / 0x800000 },	// 50mA DC
{ DmmDCCurrent,		5e-1,	"%+7.2f",	0x22,	{0x04, 0xC0,0x21,0x14,0x8B,0x95,0x11,0x08,0x15,0x11,0xF8, 0x00,0x00,0x00,0x00,0x00, 0xA0,0x80,0xC7,0x33,0xAC}, 125e-2 / 1.8 / 0x800000 },	// 500mA DC
{ DmmDCCurrent,		5e0,	"%+7.4f",	0x29,	{0x04, 0xC0,0x21,0x14,0x8B,0x35,0x11,0x08,0x15,0x11,0xF8, 0x00,0x00,0x00,0x00,0x00, 0xA0,0x80,0xC7,0x3D,0xAC}, 125e-1 / 1.8 / 0x800000 },	// 5 A DC
{ DmmDCCurrent,		10e0,	"%+7.3f",	0x29,	{0x04, 0xC0,0x21,0x14,0x8B,0x35,0x01,0x08,0x15,0x11,0xF8, 0x00,0x00,0x00,0x00,0x00, 0xA0,0x80,0xC7,0x3D,0xAC}, 125e-1 / 0.9 / 0x800000 },	// 10 A DC

{ DmmACCurrent,		5e-4,	"%7.2f",	0x20,	{0x11, 0x10,0xDD,0x12,0x00,0x45,0x01,0x88,0x15,0x31,0xF8, 0x00,0x00,0x00,0x00,0x00, 0xA0,0x80,0xC7,0x3D,0x20}, 1e-8 / 1.08 },				// 500 uA AC
{ DmmACCurrent,		5e-3,	"%7.4f",	0x20,	{0x11, 0x10,0xDD,0x12,0x00,0x45,0x00,0x88,0x15,0x31,0xF8, 0x00,0x00,0x00,0x00,0x00, 0xA0,0x80,0xC7,0x3D,0xA0}, 1e-7 / 2.16 },				// 5 mA AC
{ DmmACCurrent,		5e-2,	"%7.3f",	0x22,	{0x11, 0x10,0xDD,0x12,0x00,0x45,0x01,0x88,0x15,0x31,0xF8, 0x00,0x00,0x00,0x00,0x00, 0xA0,0x80,0xC7,0x3D,0x20}, 1e-6 / 1.08 },				// 50 mA AC
{ DmmACCurrent,		5e-1,	"%7.2f",	0x22,	{0x11, 0x10,0xDD,0x12,0x00,0x45,0x00,0x88,0x15,0x31,0xF8, 0x00,0x00,0x00,0x00,0x00, 0xA0,0x80,0xC7,0x3D,0xA0}, 1e-5 / 2.16 },				// 500 mA AC
{ DmmACCurrent,		5e0,	"%7.4f",	0x29,	{0x11, 0x10,0xDD,0x12,0x00,0x45,0x01,0x88,0x15,0x31,0xF8, 0x00,0x00,0x00,0x00,0x00, 0xA0,0x80,0xC7,0x3D,0x20}, 1e-4 / 1.08 },				// 5 A AC
{ DmmACCurrent,		10e0,	"%7.3f",	0x29,	{0x11, 0x10,0xDD,0x12,0x00,0x45,0x00,0x88,0x15,0x31,0xF8, 0x00,0x00,0x00,0x00,0x00, 0xA0,0x80,0xC7,0x3D,0xA0}, 1e-4 / 2.16 },				// 10 A AC

{ DmmResistance,	5e1,	"%7.3f",	0x05,	{0x04, 0x60,0x62,0x13,0x83,0x85,0x01,0x08,0x15,0x00,0xF8, 0x00,0x40,0x06,0x00,0x00, 0x94,0x80,0xD2,0x3F,0xAC}, 1e3 / 0.9 / 0x800000 },		// 50 Ohm
{ DmmResistance,	5e2,	"%7.2f",	0x05,	{0x04, 0xC0,0x21,0x14,0x83,0x85,0x01,0x08,0x15,0x00,0xF8, 0x00,0x00,0x06,0x00,0x00, 0x94,0x80,0xD2,0x3F,0xAC}, 1e3 / 0.9 / 0x800000 },		// 500 Ohm
{ DmmResistance,	5e3,	"%7.4f",	0x05,	{0x04, 0xC0,0x21,0x14,0x83,0x85,0x01,0x08,0x15,0x00,0xF8, 0x00,0x00,0x60,0x00,0x00, 0x94,0x80,0xD3,0x3F,0xAC}, 1e4 / 0.9 / 0x800000 },		// 5 kOhm
{ DmmResistance,	5e4,	"%7.3f",	0x05,	{0x04, 0xC0,0x21,0x14,0x83,0x85,0x01,0x08,0x15,0x00,0xF8, 0x00,0x00,0x00,0x06,0x00, 0x94,0x80,0xD3,0x3F,0xAC}, 1e5 / 0.9 / 0x800000 },		// 50 kOhm
{ DmmResistance,	5e5,	"%7.2f",	0x05,	{0x04, 0xC0,0x21,0x14,0x83,0x85,0x01,0x08,0x15,0x00,0xF8, 0x00,0x00,0x00,0x60,0x00, 0x94,0x80,0xD3,0x3F,0xAC}, 1e6 / 0.9 / 0x800000 },		// 500 kOhm
{ DmmResistance,	5e6,	"%7.4f",	0x05,	{0x04, 0xC0,0x21,0x14,0x93,0x85,0x01,0x08,0x15,0x55,0xF8, 0x00,0x00,0x00,0x80,0x00, 0x86,0x80,0xD1,0x3F,0xAC}, 6e6 / 0.9 / 0x800000 },		// 5 MOhm
{ DmmResistance,	5e7,	"%7.3f",	0x05,	{0x04, 0xC0,0x21,0x14,0x93,0x85,0x01,0x08,0x15,0x55,0xF8, 0x00,0x08,0x00,0x00,0x00, 0x86,0x80,0xD1,0x3F,0xAC}, 6e7 / 0.9 / 0x800000 },		// 50 MOhm

{ DmmResistance4W,	5e2,	"%7.2f",	0x05,	{0x04, 0xC0,0x21,0x14,0x83,0xA7,0x01,0x08,0x15,0x00,0xF8, 0x00,0x00,0x06,0x00,0x00, 0x94,0x80,0xD2,0x3F,0xAC}, 1e3 / 0.9 / 0x800000 },		// 500 Ohm
{ DmmResistance4W,	5e3,	"%7.4f",	0x05,	{0x04, 0xC0,0x21,0x14,0x83,0xA7,0x01,0x08,0x15,0x00,0xF8, 0x00,0x00,0x60,0x00,0x00, 0x94,0x80,0xD3,0x3F,0xAC}, 1e4 / 0.9 / 0x800000 },		// 5 kOhm
{ DmmResistance4W,	5e4,	"%7.3f",	0x05,	{0x04, 0xC0,0x21,0x14,0x83,0xA7,0x01,0x08,0x15,0x00,0xF8, 0x00,0x00,0x00,0x06,0x00, 0x94,0x80,0xD3,0x3F,0xAC}, 1e5 / 0.9 / 0x800000 },		// 50 kOhm

{ DmmCapacitance,	5e-8,	"%7.3f",	0x05,	{0x01, 0x70,0x21,0x14,0x8B,0x01,0x11,0x08,0x15,0x31,0xF8, 0x00,0x00,0x00,0x08,0x00, 0x9A,0x80,0xD7,0x33,0xA8}, 2.5e-12 },	// CAP LOW		// 50nF (Mode I - CTx)
{ DmmCapacitance,	5e-7,	"%7.2f",	0x05,	{0x01, 0x70,0x21,0x14,0x8B,0x01,0x11,0x08,0x15,0x31,0xF8, 0x00,0x00,0x00,0x08,0x00, 0x9A,0x80,0xD7,0x33,0xA8}, 2.5e-12 },	// have all		// 500nF
{ DmmCapacitance,	5e-6,	"%7.4f",	0x05,	{0x01, 0x70,0x21,0x14,0x8B,0x01,0x11,0x08,0x15,0x31,0xF8, 0x00,0x00,0x00,0x08,0x00, 0x9A,0x80,0xD7,0x33,0xA8}, 2.5e-12 },	// the same		// 5uF
{ DmmCapacitance,	5e-5,	"%7.3f",	0x05,	{0x01, 0x70,0x21,0x14,0x8B,0x01,0x11,0x08,0x15,0x31,0xF8, 0x00,0x00,0x00,0x08,0x00, 0x9A,0x80,0xD7,0x33,0xA8}, 2.5e-12 },	// register		// 50uF
{ DmmCapacitance,	5e-4,	"%7.2f",	0x05,	{0x01, 0x70,0x21,0x14,0x8B,0x01,0x11,0x08,0x15,0x31,0xF8, 0x00,0x00,0x00,0x08,0x00, 0x9A,0x80,0xD7,0x33,0xA8}, 2.5e-12 },	// settings		// 500uF

{ DmmCapacitance,	5e-3,	"%7.4f",	0x05,	{0x04, 0xC0,0x21,0x07,0x93,0x85,0x11,0x08,0x15,0x55,0xF8, 0x80,0x00,0x08,0x00,0x00, 0x86,0x80,0xD1,0x33,0xAC}, 1e-5 },						// 5mF (Mode II - AD1)
{ DmmCapacitance,	5e-2,	"%7.3f",	0x05,	{0x04, 0x70,0xDE,0x07,0x93,0x85,0x11,0x08,0x15,0x55,0xF8, 0x80,0x00,0x08,0x00,0x00, 0x8F,0x80,0xDA,0x33,0xAC}, 1e-5 },						// 50mF

{ DmmFrequency,		6e7,	"%8.0f",	0x05,	{0x01, 0xC8,0xDE,0x07,0x93,0x85,0x11,0x08,0x15,0x55,0xF8, 0x00,0x00,0x00,0x00,0x00, 0x80,0x80,0xD7,0x33,0xAC}, 1e0 },						// Frequency

{ DmmTemperature,	8e2,	"%+7.2f",	0x01,	{0x04, 0xC0,0xDE,0x14,0x8B,0x85,0x11,0x08,0x15,0x31,0xF8, 0x00,0x00,0x00,0x00,0x08, 0x81,0x80,0xC7,0x33,0xA8}, 125e3 / 1.8 / 0x800000 },	// Temperature (thermo couple)

{ DmmContinuity,	5e2,	"%7.3f",	0x05,	{0x04, 0x60,0x62,0x13,0x83,0x85,0x01,0x08,0x15,0x00,0xF8, 0x00,0x40,0x06,0x00,0x00, 0x94,0x80,0xD2,0x3F,0xAC}, 1e3 / 0.9 / 0x800000 },		// Continuity

{ DmmDiode,			3e0,	"%7.4f",	0x05,	{0x04, 0x10,0x62,0x13,0x8B,0x8D,0x11,0x08,0x15,0x11,0xF8, 0x00,0x00,0x08,0x00,0x00, 0x86,0x80,0xE2,0x33,0xAC}, 1e-6 / 1.08 }				// Diode
};

#if 0
struct _THERMO {
	char Type;		// thermocouple type (E,J,K,N)
	double V800;	// thermo voltage @ 800°C (0V @ 0°C)
} thermo[] = {
	{ 'E', 61.017372 },
	{ 'J', 45.494394 },
	{ 'K', 33.275380 },
	{ 'N', 28.454520 },
};
#endif

/*
 * PT100:	100 Ohm @ 0°C, 375.704 Ohm @ 800°C
 * PT500:	PT100 * 5
 * PT1000:	PT100 * 10
 */

const char * const dmmranges[] = {
	"50mV DC", "500mV DC", "5V DC", "50V DC", "500V DC", "1kV DC",
	"500mV AC", "5V AC", "50V AC", "500V AC", "750V AC",
	"500uA DC", "5mA DC", "50mA DC", "500mA DC", "5A DC", "10A DC",
	"500uA AC", "5mA AC", "50mA AC", "500mA AC", "5A AC", "10A AC",
	"50 Ohm", "500 Ohm", "5 kOhm", "50 kOhm", "500 kOhm", "5 MOhm", "50 MOhm",
	"500 Ohm 4W", "5 kOhm 4W", "50 kOhm 4W",
	"50 nF", "500 nF", "5 uF", "50 uF", "500 uF", "5 mF", "50 mF",
	"FREQ", "TEMP", "CONT", "DIODE"
};

enum {
	RESULT_CT	= 1,
	RESULT_RMS	= 2,
	RESULT_AD1	= 4,
	RESULT_AD2	= 8
};

double dMeasuredVal[3];
uint32_t currCTA, currCTB, currCTC;
int32_t currAD1, currAD2, currAD3;
int64_t currRMS;
uint8_t DMM_Status = 0;

static DMMREGISTERS dmmRegs;
static CALIBDATA const *curCal = NULL;
static DMMCFG const *curCfg = NULL;						// pointer to the current configuration
static int idxCurrentScale[NUM_CHANNELS] = {-1,-1,-1};	// stores the current selected scale
static double dValAvg[NUM_CHANNELS];					// value sum

// GateTime = ( 1000000h - CTA ) / SysFreq => CTA = 1000000h - GateTime * SysFreq	[GateTime in s]
static uint16_t freqGateTime = 1000;					// 1000ms
static uint32_t CTA_Initial = 0x1000000 - CRYSTAL;		// frequency counter CTA preload value for 1s gatetime

static uint8_t fUseCalib[NUM_CHANNELS];					// controls if calibration coefficients should be applied in DMM_DGetStatus
static uint8_t nAvgPasses[NUM_CHANNELS];				// total number of averaging passes to do
static uint8_t nAvgCount[NUM_CHANNELS];					// 0: current measurement finished, else number of passes still to go
static uint32_t measurement_start;

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
	DelayAprox10Us( 50 );
}

static void GPIO_SetValue_MOSI( uint8_t state )
{
	HAL_GPIO_WritePin( GPIOB, SPI_MOSI, state ? GPIO_PIN_SET : GPIO_PIN_RESET );
	DelayAprox10Us( 50 );
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
static void DMM_SendCmdSPI( uint8_t cs, uint8_t bCmd, uint8_t bytesNumber, const uint8_t *pbWrData )
{
	if( cs != CS_RLY )
	{
		// Activate CS
		GPIO_SetValue_CS( cs, 0 );
		bCmd <<= 1;				// register number must be sent as (regno << 1 | 0), 0 for write
	}

	uint8_t i;
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
static void DMM_GetCmdSPI( uint8_t bCmd, int bytesNumber, uint8_t *pbRdData )
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
 **		Bit 0: U1  (10A / A - input shunts)
 **		Bit 1: RL2 (mA / uA)
 **		Bit 2: RL1 (VA / RLD)
 **		Bit 3: U2  (10A / A - ADC)
 **		Bit 4:     unused
 **		Bit 5: U19 (PB0 connected / disconnected)
 **
 */
void DMM_ConfigSwitches( uint8_t sw )
{
	DMM_SendCmdSPI( CS_RLY, sw, 1, &sw );		// send value twice
}

/***	DMM_isScale
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
uint8_t DMM_isScale( int idxScale )
{
	uint8_t bResult = ( idxScale >= 0 ) && ( idxScale < DMM_CNTSCALES ) ? ERRVAL_SUCCESS : ERRVAL_DMM_IDXCONFIG;
	return bResult;
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
uint8_t DMM_SetScale( uint8_t channel, int idxScale )
{
	if( --channel >= NUM_CHANNELS )
		return ERRVAL_CMD_WRONGPARAMS;

	idxCurrentScale[channel] = -1;		// invalidate current scale
	curCfg = NULL;						// invalidate pointer to the current configuration
	curCal = NULL;

	// Verify index
	uint8_t bResult = DMM_isScale( idxScale );
	if( bResult != ERRVAL_SUCCESS ) return bResult;

	// Reset the DMM
	dmmRegs.R37 = 0x60;												// "If register Address=37h, write-in Data=60h, the IC will Reset."
	DMM_SendCmdSPI( CS_DMM, REG_37, 1, &dmmRegs.R37 );

//	DelayAprox10Us( 1000 );											// 10ms

	// Retrieve current Scale information (register setting, switch settings, calibration, etc.)
	idxCurrentScale[channel] = idxScale;
	curCal = &calib[idxScale];
	curCfg = &dmmcfg[idxScale];

	if( DMM_isAC( idxScale ) )
		idxScale = SCALE_FREQ;

	// Set CTA start values
	switch( idxScale )
	{
	// GateTime = ( 0x1000000 - CTA ) / SysFreq => CTA = 1000000h - GateTime * SysFreq	[GateTime in s]
//	case SCALE_FREQ:	CTA_Initial = 0x1000000 - CRYSTAL; break;		// 1s
	case SCALE_FREQ:	CTA_Initial = 0x1000000 - CRYSTAL / 10; break;	// 0.1s
//	case SCALE_FREQ:	CTA_Initial = 0x1000000 - CRYSTAL / 100; break;	// 10ms

	case SCALE_50_nF:
	case SCALE_500_nF:	CTA_Initial = 0x1000000 - CRYSTAL; break;		// 1s

	case SCALE_5_uF:
	case SCALE_50_uF:	CTA_Initial = 0x1000000 - CRYSTAL/10; break;	// 100ms

	case SCALE_500_uF:	CTA_Initial = 0x1000000 - CRYSTAL/100; break;	// 10ms
	}

	// clear all shadow registers to zero
	memset( &dmmRegs, 0, sizeof(dmmRegs) );

	// copy configuration values for INTE and R20 thru R33
	memcpy( &dmmRegs.INTE.reg, curCfg->cfg, sizeof(curCfg->cfg) );

	// copy complete register set to HY3131 (AD1..R37), clearing ADCs, counters and interrupt flags
	DMM_SendCmdSPI( CS_DMM, REG_AD1, sizeof(dmmRegs), (uint8_t*)&dmmRegs );

	// set the relays and switches
	DMM_ConfigSwitches( curCfg->sw );

	return ERRVAL_SUCCESS;
}

int DMM_GetScale( uint8_t channel )
{
	if( --channel >= NUM_CHANNELS )
		return -1;
	return idxCurrentScale[channel];	// stores the current selected scale
}

int DMM_FindScale( int mode, double range )
{
	if( mode < 0 )		// oops
		return -1;

	uint8_t idxScale;
	for( idxScale = 0; idxScale < DMM_CNTSCALES; ++idxScale )
	{
		if( DMM_GetMode( idxScale ) == mode &&
			DMM_GetRange( idxScale ) == range )
		{
			return idxScale;
		}
	}

	return -1;
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
	if( DMM_isScale( idxScale ) != ERRVAL_SUCCESS ) return -1;
	return dmmcfg[idxScale].mode;
}

/***	DMM_GetRange
 **	Parameters:
 **      none
 **	Return Value:
 **		int     - current scale index
 **             - -1 if no current scale was selected
 **	Description:
 **		This function returns the current range.
 **      This is the last scale index selected using the DMM_SetScale function.
 **      In case no current scale was selected, the function returns -1.
 */
double DMM_GetRange( int idxScale )
{
	if( DMM_isScale( idxScale ) != ERRVAL_SUCCESS ) return 0.0;
	return dmmcfg[idxScale].range;
}

const char *DMM_GetFormat( int idxScale )
{
	if( DMM_isScale( idxScale ) != ERRVAL_SUCCESS ) return "";
	return dmmcfg[idxScale].fmt;
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
void DMM_SetUseCalib( uint8_t channel, uint8_t f )
{
	if( --channel < NUM_CHANNELS )
		fUseCalib[channel] = f;
}

char DMM_GetUseCalib( uint8_t channel )
{
	return ( --channel < NUM_CHANNELS ) ? fUseCalib[channel] : 0;
}

void DMM_SetAveraging( uint8_t channel, uint8_t nAvg )
{
	if( --channel < NUM_CHANNELS )
		nAvgPasses[channel] = ( nAvg < 1 ) ? 1 : nAvg;
}

uint8_t	DMM_GetAveraging( uint8_t channel )
{
	return ( --channel < NUM_CHANNELS ) ? nAvgPasses[channel] : 0;
}

void DMM_SetGateTime( uint16_t ms )
{
	freqGateTime = ms;
	CTA_Initial = 0x1000000ul - ( (uint64_t)ms * CRYSTAL ) / 1000;	// frequency counter CTA preload value
}

uint16_t DMM_GetGateTime( void )
{
	return freqGateTime;
}

/***	DMM_isAC
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
uint8_t DMM_isAC( int idxScale )
{
	if( DMM_isScale( idxScale ) != ERRVAL_SUCCESS ) return 0;
	int mode = dmmcfg[idxScale].mode;
	return ( mode == DmmACVoltage ) || ( mode == DmmACCurrent );
}

/***	DMM_isVOLT
 **	Parameters:
 **      int idxScale  - the scale index
 **	Return Value:
 **		1 if the specified scale is a voltage scale.
 **		0 if the specified scale is not a voltage cale.
 **	Description:
 **		This function checks if the specified scale is a voltage scale.
 **      It returns 1 for the DC voltage and AC voltage scales, and 0 otherwise.
 **      The scale type is checked using the mode field in DMMCFG structure.
 */
uint8_t DMM_isVOLT( int idxScale )
{
	if( DMM_isScale( idxScale ) != ERRVAL_SUCCESS ) return 0;
	int mode = dmmcfg[idxScale].mode;
	return ( mode == DmmDCVoltage ) || ( mode == DmmACVoltage );
}

/***	DMM_isCURR
 **	Parameters:
 **      int idxScale  - the scale index
 **	Return Value:
 **		1 if the specified scale is a current scale.
 **		0 if the specified scale is not a current scale.
 **	Description:
 **		This function checks if the specified scale is a current scale.
 **      It returns 1 for the DC Current and AC Current scales, and 0 otherwise.
 **      The scale type is checked using the mode field in DMMCFG structure.
 */
uint8_t DMM_isCURR( int idxScale )
{
	if( DMM_isScale( idxScale ) != ERRVAL_SUCCESS ) return 0;
	int mode = dmmcfg[idxScale].mode;
	return ( mode == DmmDCCurrent ) || ( mode == DmmACCurrent );
}

/* ************************************************************************** */
/***	DMM_isDC
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
uint8_t DMM_isDC( int idxScale )
{
	if( DMM_isScale( idxScale ) != ERRVAL_SUCCESS ) return 0;
	int mode = dmmcfg[idxScale].mode;
	return ( mode == DmmDCVoltage ) || ( mode == DmmDCCurrent );
}

/***	DMM_isFRES
 **	Parameters:
 **      int idxScale  - the scale index
 **	Return Value:
 **		1 if the specified scale is a 4 wire Resistor type scale.
 **		0 if the specified scale is not a 4 wire Resistor type scale.
 **	Description:
 **		This function checks if the specified scale is a 4 wire Resistor type scale.
 **      It returns 1 for the 4 wire Resistor type scales, and 0 otherwise.
 **      The scale type is checked using the mode field in DMMCFG structure.
 */
uint8_t DMM_isFRES( int idxScale )
{
	if( DMM_isScale( idxScale ) != ERRVAL_SUCCESS ) return 0;
	int mode = dmmcfg[idxScale].mode;
	return ( mode == DmmResistance4W );
}

/***	DMM_isRES
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
uint8_t DMM_isRES( int idxScale )
{
	if( DMM_isScale( idxScale ) != ERRVAL_SUCCESS ) return 0;
	int mode = dmmcfg[idxScale].mode;
	return ( mode == DmmResistance ) || ( mode == DmmResistance4W ) || ( mode == DmmContinuity );
}

/***	DMM_isCAP
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
uint8_t DMM_isCAP( int idxScale )
{
	if( DMM_isScale( idxScale ) != ERRVAL_SUCCESS ) return 0;
	int mode = dmmcfg[idxScale].mode;
	return ( mode == DmmCapacitance );
}

/***	DMM_isDIOD
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
uint8_t DMM_isDIOD( int idxScale )
{
	if( DMM_isScale( idxScale ) != ERRVAL_SUCCESS ) return 0;
	int mode = dmmcfg[idxScale].mode;
	return ( mode == DmmDiode );
}

/***	DMM_isCONT
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
uint8_t DMM_isCONT( int idxScale )
{
	if( DMM_isScale( idxScale ) != ERRVAL_SUCCESS ) return 0;
	int mode = dmmcfg[idxScale].mode;
	return ( mode == DmmContinuity );
}

/***	DMM_isNAN
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
uint8_t DMM_isNAN( double dVal )
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
uint8_t DMM_GetScaleUnit( uint8_t scale, double *pdScaleFact, char *szUnitPrefix, char *szUnit, char *szRange )
{
	uint8_t bResult = DMM_isScale( scale );
	if( bResult == ERRVAL_SUCCESS )				// valid idxScale
	{
		DMMCFG const *cfg = &dmmcfg[scale];

		if( pdScaleFact && szUnitPrefix )
		{
			if( scale == SCALE_FREQ ||
				scale == SCALE_TEMP )	{ *pdScaleFact = 1e0;	szUnitPrefix[0] = 0; }
			else if( cfg->range < 1e-9 )	{ *pdScaleFact = 1e12;	szUnitPrefix[0] = 'p'; }
			else if( cfg->range < 1e-6 )	{ *pdScaleFact = 1e9;	szUnitPrefix[0] = 'n'; }
			else if( cfg->range < 1e-3 )	{ *pdScaleFact = 1e6;	szUnitPrefix[0] = 'u'; }
			else if( cfg->range < 1e0 )		{ *pdScaleFact = 1e3;	szUnitPrefix[0] = 'm'; }
			else if( cfg->range < 1e3 )		{ *pdScaleFact = 1e0;	szUnitPrefix[0] =  0 ; }
			else if( cfg->range < 1e6 )		{ *pdScaleFact = 1e-3;	szUnitPrefix[0] = 'k'; }
			else if( cfg->range < 1e9 )		{ *pdScaleFact = 1e-6;	szUnitPrefix[0] = 'M'; }
			else							{ *pdScaleFact = 1e-9;	szUnitPrefix[0] = 'G'; }
			szUnitPrefix[1] = 0;
		}

		if( szUnit )
		{
			switch( cfg->mode )
			{
			case DmmDiode:			strcpy( szUnit, "V" ); break;
			case DmmResistance:
			case DmmResistance4W:
			case DmmContinuity:		strcpy( szUnit, "Ohm" ); break;
			case DmmTemperature:	strcpy( szUnit, "'C" ); break;
			case DmmCapacitance:	strcpy( szUnit, "'F" ); break;
			case DmmDCVoltage:		strcpy( szUnit, "V DC" ); break;
			case DmmDCCurrent:		strcpy( szUnit, "A DC" ); break;
			case DmmACVoltage:		strcpy( szUnit, "V AC" ); break;
			case DmmACCurrent:		strcpy( szUnit, "A AC" ); break;
			case DmmFrequency:		strcpy( szUnit, "Hz" ); break;
			}
		}

		if( szRange )
		{
			strcpy( szRange, dmmranges[scale] );
		}
	}
	return bResult;
}

static void DMM_ReloadCounters( uint8_t R20, uint32_t CTA )
{
	R20 &= ~2;
	DMM_SendCmdSPI( CS_DMM, REG_20, 1, &R20 );			// write R20=X0h - ENCTR=0 (disable counters, reset CTB and CTC)

	dmmRegs.CTA[0] = CTA & 0xFF;
	dmmRegs.CTA[1] = ( CTA >> 8 ) & 0xFF;
	dmmRegs.CTA[2] = ( CTA >> 16 ) & 0xFF;
	DMM_SendCmdSPI( CS_DMM, REG_CTA, 3, dmmRegs.CTA );	// write CTA (preset counter)

	R20 |= 2;
	DMM_SendCmdSPI( CS_DMM, REG_20, 1, &R20 );			// write R20=X2h - ENCTR=1 (re-enable counters)
}

static void DMM_StartMeasurement( int scale )
{
	if( scale == SCALE_FREQ || DMM_isAC( scale ) ||
		( scale >= SCALE_50_nF && scale <= SCALE_500_uF ) )	// mode I -> using counters
	{
		DMM_ReloadCounters( curCfg->cfg[1], CTA_Initial );
	}
	else if( scale >= SCALE_5_mF && scale <= SCALE_50_mF )	// mode II -> using AD1
	{
		uint8_t i;
#if 1
		i = 0x00; DMM_SendCmdSPI( CS_DMM, 0x2A, 1, &i );	// write R2A=00h - P0=0000 P1=0000	(is 0x80 from init : PS1=1)
		i = 0x00; DMM_SendCmdSPI( CS_DMM, 0x2C, 1, &i );	// write R2C=00h - P4=0000 P5=0000	(is 0x08 from init : PS4=1)
		DelayAprox10Us( 1000 );
#endif
		i = 0x4C; DMM_SendCmdSPI( CS_DMM, 0x20, 1, &i );	// write R20=4Ch - SCMPI=000, ENCMP=1, ENCNTI=1, ENPCMPO=1, ENCTR=0 (enable PCMP output -> discharge?)
		DelayAprox10Us( 1000 );
		i = 0x48; DMM_SendCmdSPI( CS_DMM, 0x20, 1, &i );	// write R20=48h - SCMPI=000, ENCMP=1, ENCNTI=1, ENPCMPO=0, ENCTR=0 (disable PCMP output)
#if 0
		i = 0x00; DMM_SendCmdSPI( CS_DMM, 0x2A, 1, &i );	// write R2A=00h again. TODO: do we need to do this?
		i = 0x00; DMM_SendCmdSPI( CS_DMM, 0x2C, 1, &i );	// write R2C=00h again. TODO: do we need to do this?
#endif
	}

	dMeasuredVal[0] = NAN;
	dMeasuredVal[1] = NAN;
	dMeasuredVal[2] = NAN;

	DMM_Status = 0;
}

void DMM_Trigger( uint8_t channel )
{
	if( --channel >= NUM_CHANNELS ) return;

	DMM_Status = 0;

	// clear summing buffer and set number of loops
	dValAvg[channel] = 0.0;
	nAvgCount[channel] = nAvgPasses[channel];

	// start 1st measurement
	DMM_StartMeasurement( idxCurrentScale[channel] );

	// init timeout counter
	measurement_start = HAL_GetTick();
}

static uint8_t DMM_GetResults( uint8_t scale )
{
	if( HAL_GPIO_ReadPin( GPIOB, SPI_MISO ) == GPIO_PIN_SET )		// MISO pin goes high if any interrupt flag in INTF becomes set
	{
		DMM_GetCmdSPI( REG_INTF, 1, &dmmRegs.INTF.reg );			// read INTF register to see which flag (register gets reset after read)

		if( dmmRegs.INTF.CTF )
		{
			DMM_GetCmdSPI( REG_CTSTA, 10, &dmmRegs.CTSTA.reg );		// read CTSTA, CTC, CTB and CTA

			currCTA = currCTB = currCTC = 0;

			uint8_t i;
			for( i = 0; i < 3; ++i ) currCTA = ( currCTA << 8 ) | dmmRegs.CTA[2-i];
			for( i = 0; i < 3; ++i ) currCTB = ( currCTB << 8 ) | dmmRegs.CTB[2-i];
			for( i = 0; i < 3; ++i ) currCTC = ( currCTC << 8 ) | dmmRegs.CTC[2-i];

			if( dmmRegs.CTSTA.CTBOV == 0 )
				DMM_Status |= RESULT_CT;
		}

		if( dmmRegs.INTF.AD1F )
		{
			DMM_GetCmdSPI( REG_AD1, 3, dmmRegs.AD1 );				// read AD1 register

			currAD1 = ( (int32_t)dmmRegs.AD1[ 2 ] << 24 )
					| ( (int32_t)dmmRegs.AD1[ 1 ] << 16 )
					| ( (int32_t)dmmRegs.AD1[ 0 ] << 8 );
			currAD1 /= 0x100;										// sign extend
			DMM_Status |= RESULT_AD1;
		}

		if( dmmRegs.INTF.RMSF )
		{
			DMM_GetCmdSPI( REG_RMS, 5, dmmRegs.RMS );				// read RMS register (RMS is 40-bits wide, with  !)
//			dmmRegs.RMS[0] &= ~0x0F;								// mute noise on 4 LSBs

			currRMS = ( (int64_t)dmmRegs.RMS[ 4 ] << 56 )
					| ( (int64_t)dmmRegs.RMS[ 3 ] << 48 )
					| ( (int64_t)dmmRegs.RMS[ 2 ] << 40 )
					| ( (int64_t)dmmRegs.RMS[ 1 ] << 32 )
					| ( (int64_t)dmmRegs.RMS[ 0 ] << 24 );
			currRMS >>= 24;
			DMM_Status |= RESULT_RMS;
		}
	}

	return DMM_Status;
}

/***	DMM_Measure
 **
 **	Parameters:
 **     uint8_t	channel - channel to take the measurement on
 **
 **	Return Value:
 **		uint8_t     - the error code
 **          ERRVAL_SUCCESS              0       // success
 **          ERRVAL_DMM_VALIDDATATIMEOUT 0xFA    // valid data DMM timeout
 **          ERRVAL_DMM_IDXCONFIG        0xFC    // error, wrong current scale index
 **
 **	Description:
 **		This function implements the Measurement
 */
uint8_t DMM_Measure( uint8_t channel )
{
	uint8_t bResult = ERRVAL_CALIB_NANDOUBLE;
	double dVal = NAN;

	int scale = DMM_GetScale( channel-- );
	if( DMM_isScale( scale ) != ERRVAL_SUCCESS )
		return ERRVAL_CMD_WRONGPARAMS;

	if( nAvgCount[channel] == 0 )
		return ERRVAL_CMD_NO_TRIGGER;

	if( ( HAL_GetTick() - measurement_start ) > 2000 )			// time out after ~2s
	{
		// TODO - inc/dec range

		switch( DMM_GetMode( scale ) )
		{
		case DmmTemperature:
		case DmmContinuity:
		case DmmDiode:
		case DmmResistance:
		case DmmResistance4W:	dMeasuredVal[channel] = +INFINITY; break;
		default:				dMeasuredVal[channel] = 0; break;
		}
		nAvgCount[channel] = 0;
		return ERRVAL_SUCCESS;
	}

	if( !DMM_GetResults( scale ) )
		return ERRVAL_CMD_BUSY;

	// Compute result, according to the specific scale
	if( scale == SCALE_FREQ )
	{
		if( DMM_Status & RESULT_CT )
		{
			/*
			 * Tgate = ( 0x1000000 - CTA_Init ) / Fsys
			 * => CTA_init = 0x1000000 - Tgate * Fsys			// Tgate in [s]
			 *
			 * T = ( 0x1000000 - CTA_init + CTA ) / Fsys
			 * F = CTB / T
			 * D = CTC / ( 0x1000000 - CTA_init + CTA )
			 *
			 * T = ( 0x1000000 - ( 0x1000000 - Tgate * Fsys ) ) / Fsys
			 * T = ( 0x1000000 - 0x1000000 + Tgate * Fsys ) / Fsys
			 * T = ( Tgate * Fsys ) / Fsys
			 * T = Tgate
			 *
			 * rearranged to
			 *
			 * T1 = 0x1000000 - CTA_init + CTA
			 * F = CTB / ( T1 / Fsys ) = TCB * Fsys / T1
			 * D = CTC / T1
			 *
			 * T1 = 0x1000000 - ( 0x1000000 - Tgate * Fsys ) + CTA
			 * T1 = 0x1000000 - 0x1000000 + Tgate * Fsys + CTA
			 * T1 = Tgate * Fsys + CTA
			 *
			 * F = CTB * Fsys / ( Tgate * Fsys + CTA )
			 */
			uint32_t T1 = 0x1000000 - CTA_Initial + currCTA;
			if( T1 != 0 )
			{
				dVal = (double)currCTB * CRYSTAL / T1;				// frequency [Hz]
				dMeasuredVal[1] = 100.0 * currCTC / T1;				// duty cycle [%]
			}
			bResult = ERRVAL_SUCCESS;
		}
	}
	else if( scale >= SCALE_50_nF && scale <= SCALE_500_uF )		// Read Mode I
	{
		if( DMM_Status & RESULT_CT )
		{
			if( currCTB != 0 )
			{
				dVal = (double)currCTC / currCTB;

				if( fUseCalib[channel] )							// apply calibration coefficients
					dVal = dVal * curCal->Mult + curCal->Add;

				dVal *= curCfg->mul;								// apply scaling factor
			}
			bResult = ERRVAL_SUCCESS;
		}
	}
	else if( scale >= SCALE_5_mF && scale <= SCALE_50_mF )			// Read Mode II
	{
		if( DMM_Status & RESULT_AD1 )
		{
			if(		 currAD1 >= 0x7FFFFE )	dVal = +INFINITY;		// value outside convertor range
			else if( currAD1 <= -0x7FFFFE )	dVal = -INFINITY;		// value outside convertor range
			else
			{
				dVal = currAD1;

				if( fUseCalib[channel] )							// apply calibration coefficients
					dVal = dVal * curCal->Mult + curCal->Add;

				dVal *= curCfg->mul;								// apply scaling factor
			}
			bResult = ERRVAL_SUCCESS;
		}
	}
	else if( DMM_isAC( scale ) )									// AC uses RMS & CT
	{
		if( DMM_Status & RESULT_CT )
		{
			uint32_t T1 = 0x1000000 - CTA_Initial + currCTA;
			if( T1 != 0 )
			{
				dMeasuredVal[1] = (double)currCTB * CRYSTAL / T1;	// frequency [Hz]
//				dMeasuredVal[2] = 100.0 * currCTC / T1;				// duty cycle [%]
			}
		}

		if( DMM_Status & RESULT_RMS )
		{
			dVal = currRMS;

			if( fUseCalib[channel] )								// apply calibration coefficients
				dVal = fabs( pow( curCal->Mult, 2 ) * dVal - pow( curCal->Add, 2 ) );

			dVal *= pow( curCfg->mul, 2 );							// apply scaling factor, dVal is already squared
		}

//		if( ( DMM_Status & (RESULT_RMS|RESULT_CT) ) == (RESULT_RMS|RESULT_CT) )
			bResult = ERRVAL_SUCCESS;
	}
	else															// DC, RES, DIODE, CONT, TEMP use AD1
	{
		if( DMM_Status & RESULT_AD1 )
		{
			if(		 currAD1 >= 0x7FFFFE )	dVal = +INFINITY;		// value outside convertor range
			else if( currAD1 <= -0x7FFFFE )	dVal = -INFINITY;		// value outside convertor range
			else
			{
				dVal = currAD1;

				if( fUseCalib[channel] )							// apply calibration coefficients
					dVal = dVal * curCal->Mult + curCal->Add;

				dVal *= curCfg->mul;								// apply scaling factor
			}

			bResult = ERRVAL_SUCCESS;
		}
	}

	// TODO - inc/dec range
	if( DMM_isNAN( dVal ) || dVal == +INFINITY || dVal == -INFINITY )
		bResult = ERRVAL_CALIB_NANDOUBLE;

	if( bResult == ERRVAL_SUCCESS )
	{
		dValAvg[channel] += dVal;									// sum up (AC is already squared)
		nAvgCount[channel] -= 1;									// decrement averaging loop counter
	}

	if( nAvgCount[channel] )										// more averaging passes to go?
	{
		DMM_StartMeasurement( scale );								// start next measurement
		return ERRVAL_CMD_BUSY;
	}

	// measurement done
	dVal = dValAvg[channel] / nAvgPasses[channel];

	if( DMM_isAC( scale ) )
		dVal = sqrt( dVal );

	dMeasuredVal[channel] = dVal;
	return bResult;
}

uint8_t DMM_Ready( uint8_t channel )
{
	if( --channel >= NUM_CHANNELS ) return 0;
	return nAvgCount[channel] == 0;
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

	memset( fUseCalib, 1, NUM_CHANNELS );					// controls if calibration coefficients should be applied in DMM_DGetStatus
	memset( nAvgPasses, 1, NUM_CHANNELS );					// total number of averaging passes to do
	memset( nAvgCount, 0, NUM_CHANNELS );					// 0: current measurement finished, else number of passes still to go

	CALIB_Init();
}

/* *****************************************************************************
 End of File
 */
