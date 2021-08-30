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
{ DmmResistance,	5e5,	0x15,	{0xC2,0x21,0x14,0x83,0x85,0x01,0x08,0x15,0x00,0xF8,0x00,0x00,0x00,0x60,0x00,0x94,0x80,0xD3,0x3F,0xAC}, 1e6 / 0.9 / 0x800000 },		// 500 kOhm
{ DmmResistance,	5e6,	0x15,	{0xC2,0x21,0x14,0x93,0x85,0x01,0x08,0x15,0x55,0xF8,0x00,0x00,0x00,0x80,0x00,0x86,0x80,0xD1,0x3F,0xAC}, 6e6 / 0.9 / 0x800000 },		// 5 MOhm
{ DmmResistance,	5e7,	0x15,	{0xC2,0x21,0x14,0x93,0x85,0x01,0x08,0x15,0x55,0xF8,0x00,0x08,0x00,0x00,0x00,0x86,0x80,0xD1,0x3F,0xAC}, 6e7 / 0.9 / 0x800000 },		// 50 MOhm

{ DmmResistance4W,	5e2,	0x15,	{0xC2,0x21,0x14,0x83,0xA7,0x01,0x08,0x15,0x00,0xF8,0x00,0x00,0x06,0x00,0x00,0x94,0x80,0xD2,0x3F,0xAC}, 1e3 / 0.9 / 0x800000 },		// 500 Ohm
{ DmmResistance4W,	5e3,	0x15,	{0xC2,0x21,0x14,0x83,0xA7,0x01,0x08,0x15,0x00,0xF8,0x00,0x00,0x60,0x00,0x00,0x94,0x80,0xD3,0x3F,0xAC}, 1e4 / 0.9 / 0x800000 },		// 5 kOhm
{ DmmResistance4W,	5e4,	0x15,	{0xC2,0x21,0x14,0x83,0xA7,0x01,0x08,0x15,0x00,0xF8,0x00,0x00,0x00,0x06,0x00,0x94,0x80,0xD3,0x3F,0xAC}, 1e5 / 0.9 / 0x800000 },		// 50 kOhm

{ DmmCapacitance,	5e-8,	0x15,	{0x72,0x21,0x14,0x8B,0x01,0x11,0x08,0x15,0x31,0xF8,0x00,0x00,0x00,0x08,0x00,0x9A,0x80,0xD7,0x33,0xA8}, 1e-5 },	// CAP LOW ranges	// 50nF (Read Mode I)
{ DmmCapacitance,	5e-7,	0x15,	{0x72,0x21,0x14,0x8B,0x01,0x11,0x08,0x15,0x31,0xF8,0x00,0x00,0x00,0x08,0x00,0x9A,0x80,0xD7,0x33,0xA8}, 1e-5 },	// have all			// 500nF
{ DmmCapacitance,	5e-6,	0x15,	{0x72,0x21,0x14,0x8B,0x01,0x11,0x08,0x15,0x31,0xF8,0x00,0x00,0x00,0x08,0x00,0x9A,0x80,0xD7,0x33,0xA8}, 1e-5 },	// the same			// 5uF
{ DmmCapacitance,	5e-5,	0x15,	{0x72,0x21,0x14,0x8B,0x01,0x11,0x08,0x15,0x31,0xF8,0x00,0x00,0x00,0x08,0x00,0x9A,0x80,0xD7,0x33,0xA8}, 1e-5 },	// register			// 50uF
{ DmmCapacitance,	5e-4,	0x15,	{0x72,0x21,0x14,0x8B,0x01,0x11,0x08,0x15,0x31,0xF8,0x00,0x00,0x00,0x08,0x00,0x9A,0x80,0xD7,0x33,0xA8}, 1e-5 },	// settings			// 500uF

{ DmmCapacitance,	5e-3,	0x15,	{0xC0,0x21,0x07,0x93,0x85,0x11,0x08,0x15,0x55,0xF8,0x80,0x00,0x08,0x00,0x00,0x86,0x80,0xD1,0x33,0xAC}, 1e-5 },						// 5mF (Read Mode II)
{ DmmCapacitance,	5e-2,	0x15,	{0x70,0xDE,0x07,0x93,0x85,0x11,0x08,0x15,0x55,0xF8,0x80,0x00,0x08,0x00,0x00,0x8F,0x80,0xDA,0x33,0xAC}, 1e-5 },						// 50mF

{ DmmFrequency,		1e0,	0x05,	{0xC8,0xDE,0x07,0x93,0x85,0x11,0x08,0x15,0x55,0xF8,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0xD7,0x33,0xAC}, 1e0 },						// Frequency

{ DmmTemperature,	1e0,	0x11,	{0xC2,0xDE,0x14,0x8B,0x85,0x11,0x08,0x15,0x31,0xF8,0x00,0x00,0x00,0x00,0x08,0x81,0x80,0xC7,0x33,0xA8}, 125e-2 / 1.8 / 0x800000 },	// Temperature

{ DmmContinuity,	5e2,	0x15,	{0x76,0x62,0x13,0x83,0x85,0x01,0x08,0x15,0x00,0xF8,0x00,0x40,0x06,0x00,0x00,0x94,0x80,0xD2,0x3F,0xAC}, 1e3 / 0.9 / 0x800000 },		// Continuity

{ DmmDiode,			3e0,	0x15,	{0x12,0x62,0x13,0x8B,0x8D,0x11,0x08,0x15,0x11,0xF8,0x00,0x00,0x08,0x00,0x00,0x86,0x80,0xE2,0x33,0xAC}, 1e-6 / 1.08 }				// Diode
};


/*
 * CAP low (50nF .. 500uF), using counters
 *
 * ACC=0000000
 *
 * AD1CHOP=10
 * AD1IG=01
 * AD1INBUF=1
 * AD1IPBUF=1
 * AD1OS=000
 * AD1OSR=100
 * AD1RG=0
 * AD1RHBUF=1
 * AD1RLBUF=0
 *
 * AD2IG=00
 * AD2OSR=000
 * AD2RG=0
 *
 * ENAD1=1
 * ENAD2=0
 * ENBIAS=1
 * ENCHOPAD1=0
 * ENCMP=1
 * ENCNTI=0
 * ENCTR=1
 * ENLPF=1
 * ENOP1=0
 * ENOP2=0
 * ENOSC=1
 * ENPCMPO=0
 * ENPKH=0
 * ENREFO=1
 * ENRMS=1
 * ENVS=1
 * ENXI=0
 *
 * LPFBW=111
 * OP1CHOP=10
 * OPS1=1
 * OPS2=0
 * PKHSEL=11
 *
 * SACM=00
 * SAD1FN=001
 * SAD1FP=0000
 * SAD1I=00
 * SAD1RH=011
 * SAD1RL=001
 * SAD2CLK=1
 * SAD2IN=01
 * SAD2IP=00
 * SAD2RH=01
 * SAD2RL=01
 * SAGND=01
 * SCMPI=011
 * SCMPRH=0010
 * SCMPRL=0001
 * SDIO=0
 * SFT1=10
 * SFUVR=0111
 * SMODE=0011010
 * SOP1P=011
 * SOP2P=011
 * SREFO=1
 *
 * PS1=0 DS1=0 FS1=0 SS1=0
 * PS0=0 DS0=0 FS0=0 SS0=0			PA0..9 all disconnected ???
 * PS3=0 DS3=0 FS3=0 SS3=0
 * PS2=0 DS2=0 FS2=0 SS2=0
 * PS5=0 DS5=0 FS5=0 SS5=0
 * PS4=0 DS4=0 FS4=0 SS4=0
 * PS7=0 DS7=0 FS7=0 SS7=0
 * PS6=0 DS6=0 FS6=0 SS6=0
 * PS9=0 DS9=0 FS9=0 SS9=0
 * PS8=0 DS8=0 FS8=0 SS8=0
 */

const char * const dmmranges[] = {
	"DC 50mV", "DC 500mV", "DC 5V", "DC 50V", "DC 500V", "DC 1kV",
	"AC 500mV", "AC 5V", "AC 50V", "AC 500V", "AC 750V",
	"DC 500uA", "DC 5mA", "DC 50mA", "DC 500mA", "DC 5A", "DC 10A",
	"AC 500uA", "AC 5mA", "AC 50mA", "AC 500mA", "AC 5A", "AC 10A",
	"50 Ohm", "500 Ohm", "5 kOhm", "50 kOhm", "500 kOhm", "5 MOhm", "50 MOhm",
	"500 Ohm 4W", "5 kOhm 4W", "50 kOhm 4W",
	"50 nF", "500 nF", "5 uF", "50 uF", "500 uF", "5 mF", "50 mF",
	"FREQ",
	"TEMP",
	"CONT",
	"DIODE"
};

double dMeasuredVal[3];
double dRawVal[3];

static CALIBDATA const *curCal = NULL;
static DMMCFG const *curCfg = NULL;						// pointer to the current configuration
static int idxCurrentScale[NUM_CHANNELS] = {-1,-1,-1};	// stores the current selected scale
static double dValAvg[NUM_CHANNELS];					// value sum

// GateTime = ( 1000000h - CTA ) / SysFreq => CTA = 1000000h - GateTime * SysFreq	[GateTime in s]
static uint16_t freqGateTime = 1000;					// 1000ms = 1s
static uint32_t CTA_Initial = 0x1000000ul - CRYSTAL;	// frequency counter CTA preload value, default to 1s

static uint8_t fUseCalib[NUM_CHANNELS];					// controls if calibration coefficients should be applied in DMM_DGetStatus
static uint8_t nAvgPasses[NUM_CHANNELS];				// total number of averaging passes to do
static uint8_t nAvgCount[NUM_CHANNELS];					// 0: current measurement finished, else number of passes still to go

static double DMM_DGetStatus( uint8_t channel, uint8_t *pbErr );	// retrieve value from DMM
static void DMM_StartFreqMeasure( void );
static void DMM_StartCapMeasurement( void );

// errors
uint8_t DMM_isScale( int idxScale );

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
static void DMM_SendCmdSPI( uint8_t cs, uint8_t bCmd, int bytesNumber, const uint8_t *pbWrData )
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
uint8_t DMM_SetScale( uint8_t channel, int idxScale )
{
	DMMREGISTERS dmmRegs;

	if( --channel >= NUM_CHANNELS )
		return ERRVAL_CMD_WRONGPARAMS;

	idxCurrentScale[channel] = -1;		// invalidate current scale
	curCfg = NULL;						// invalidate pointer to the current configuration
	curCal = NULL;

	// Verify index
	uint8_t bResult = DMM_isScale( idxScale );
	if( bResult != ERRVAL_SUCCESS ) return bResult;

	// Reset the DMM
	dmmRegs.R37.reg = 0x60;											// "If register Address=37h, write-in Data=60h, the IC will Reset."
	DMM_SendCmdSPI( CS_DMM, REG_37, 1, &dmmRegs.R37.reg );

	DelayAprox10Us( 1000 );											// 10ms

	// Retrieve current Scale information (register setting, switch settings, calibration, etc.)
	idxCurrentScale[channel] = idxScale;
	curCal = &calib[ idxScale ];
	curCfg = &dmmcfg[idxScale];

	if( idxScale == SCALE_FREQ )
	{
		// GateTime = ( 1000000h - CTA ) / SysFreq => CTA = 1000000h - GateTime * SysFreq	[GateTime in s]
		CTA_Initial = 0x1000000ul - ( (uint32_t)CRYSTAL * freqGateTime ) / 1000ul;	// frequency counter CTA preload value
	}

	// Write configuration values to registers R20..33
	DMM_SendCmdSPI( CS_DMM, REG_20, 0x14, curCfg->cfg );

	// And set the switches
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

void	DMM_SetAveraging( uint8_t channel, uint8_t nAvg )
{
	if( --channel < NUM_CHANNELS )
		nAvgPasses[channel] = ( nAvg < 1 ) ? 1 : nAvg;
}

uint8_t	DMM_GetAveraging( uint8_t channel )
{
	return ( --channel < NUM_CHANNELS ) ? nAvgPasses[channel] : 0;
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
uint8_t DMM_GetScaleUnit( uint8_t channel, double *pdScaleFact, char *szUnitPrefix, char *szUnit, char *szRange )
{
	int idxScale = DMM_GetScale( 1 );			// always use scale of 1st channel!
	uint8_t bResult = DMM_isScale( idxScale );
	if( bResult == ERRVAL_SUCCESS )				// valid idxScale
	{
		DMMCFG const *cfg = &dmmcfg[idxScale];

		if( pdScaleFact && szUnitPrefix )
		{
			if(		 cfg->range < 1e-9 )	{ *pdScaleFact = 1e12;	szUnitPrefix[0] = 'p'; }
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
			case DmmCapacitance:	strcpy( szUnit, "F" ); break;
			case DmmDCVoltage:		strcpy( szUnit, "V DC" ); break;
			case DmmDCCurrent:		strcpy( szUnit, "A DC" ); break;
			case DmmACVoltage:		strcpy( szUnit, ( channel == 1 ) ? "V AC" : "Hz" ); break;
			case DmmACCurrent:		strcpy( szUnit, ( channel == 1 ) ? "A AC" : "Hz" ); break;
			case DmmFrequency:		strcpy( szUnit, ( channel == 1 ) ? "Hz" : "%%" ); break;
			}
		}

		if( szRange )
		{
			strcpy( szRange, dmmranges[idxScale] );
		}
	}
	return bResult;
}

int DMM_Ready( uint8_t channel )
{
	if( --channel >= NUM_CHANNELS ) return -1;
	return nAvgCount[channel] == 0;
}

void DMM_Trigger( uint8_t channel )
{
	if( --channel >= NUM_CHANNELS ) return;
	dValAvg[channel] = 0.0;
	nAvgCount[channel] = nAvgPasses[channel];
	uint scale = idxCurrentScale[channel];

	// start measure
	if( scale == SCALE_FREQ )
	{
		DMM_StartFreqMeasure();
	}
	else if( scale >= SCALE_5_mF && scale <= SCALE_50_mF )	// mode II -> using AD1
	{
		DMM_StartCapMeasurement();
	}
}

static void DMM_StartCapMeasurement( void )
{
	/*
	63781624 HY3131: MOSI transfers: 54 00                       # Write R2a
	63784911 HY3131: MOSI transfers: 58 00                       # Write R2c

	delay

	64284104 74HC595: MOSI transfers: 15

	64290190 HY3131: MOSI transfers: 40 4C                       # Write R20

	64296409 HY3131: MOSI transfers: 40 48                       # Write R20

	64299713 HY3131: MOSI transfers: 54 00                       # Write R2a
	64303015 HY3131: MOSI transfers: 58 00                       # Write R2c

	64305930 HY3131: MOSI transfers: 3D FF                       # Read INTF
	64310919 HY3131: MOSI transfers: 01 FF FF FF                 # Read AD1_DATA
	*/
	uint8_t i;

	i = 0x00; DMM_SendCmdSPI( CS_DMM, 0x2A, 1, &i );	// write R2A=00h - P0=0000 P1=0000
	i = 0x00; DMM_SendCmdSPI( CS_DMM, 0x2C, 1, &i );	// write R2C=00h - P4=0000 P5=0000

	DelayAprox10Us( 1000 );

	DMM_GetCmdSPI( 0x20, 1, &i );						// read R20
	i = 0x4C; DMM_SendCmdSPI( CS_DMM, 0x20, 1, &i );	// write R20=4Ch - SCMPI=000, ENCMP=1, ENCNTI=1, ENPCMPO=1, ENCTR=0 (enable PCMP output -> discharge?)

	DMM_GetCmdSPI( 0x20, 1, &i );						// read R20
	i = 0x48; DMM_SendCmdSPI( CS_DMM, 0x20, 1, &i );	// write R20=48h - SCMPI=000, ENCMP=1, ENCNTI=1, ENPCMPO=0, ENCTR=0 (disable PCMP output)

#if 1
	i = 0x00; DMM_SendCmdSPI( CS_DMM, 0x2A, 1, &i );	// write R2A=00h again. TODO: do we need to do this?
	i = 0x00; DMM_SendCmdSPI( CS_DMM, 0x2C, 1, &i );	// write R2C=00h again. TODO: do we need to do this?
#endif
}

static void DMM_StartFreqMeasure( void )
{
	/*
	60004329 HY3131: MOSI transfers: 40 70                       # Write R20
	60010740 HY3131: MOSI transfers: 36 00 00 BF                 # Write CTA
	60014041 HY3131: MOSI transfers: 40 72                       # Write R20
	*/
	uint8_t i, cta[3];

	// this is different that what OWON does !!
	i = 0xC8;									// write R20=C8h : SCMPI=110, ENCMP=0, ENCNTI=1, ENPCMPO=0, ENCTR=0
	DMM_SendCmdSPI( CS_DMM, REG_20, 1, &i );	// (ENCTR=0: stop counters, clear CTB, CTC and CTA<7:0>, enable CTA writes)

	cta[0] =   CTA_Initial		   & 0xFF;		// reset CTA
	cta[1] = ( CTA_Initial >> 8 )  & 0xFF;
	cta[2] = ( CTA_Initial >> 16 ) & 0xFF;
	DMM_SendCmdSPI( CS_DMM, REG_CTA, 3, cta );

	i = 0xCA;
	DMM_SendCmdSPI( CS_DMM, REG_20, 1, &i );	// write R20=CAh : ENCTR=1 (start counting)
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
static double DMM_DGetStatus( uint8_t channel, uint8_t *pbErr )
{
	DMMREGISTERS dmmRegs;
	double v = NAN;
	uint8_t i;

	if( --channel >= NUM_CHANNELS )
	{
		if( pbErr ) *pbErr = ERRVAL_CMD_WRONGPARAMS;
		return NAN;
	}

	// Verify index
	uint8_t bResult = DMM_isScale( idxCurrentScale[channel] );
	if( bResult != ERRVAL_SUCCESS )
	{
		if( pbErr ) *pbErr = bResult;
		return NAN;
	}

	// Compute values, according to the specific scale
	uint8_t scale = idxCurrentScale[channel];
	if( DMM_isAC( scale ) )										// AC uses RMS
	{
		DMM_GetCmdSPI( REG_INTF, 1, &dmmRegs.INTF.reg );		// read INTF register
		if( ! dmmRegs.INTF.RMSF )								// RMSF=0 : conversion not yet ready
		{
			if( pbErr ) *pbErr = bResult;
			return NAN;
		}

		int64_t vrms = 0;
		DMM_GetCmdSPI( REG_RMS, 5, dmmRegs.RMS );				// read RMS register (RMS is 40-bits wide !)
		for( i = sizeof(dmmRegs.RMS)-1; i < sizeof(dmmRegs.RMS); --i )
			vrms = ( vrms << 8 ) | dmmRegs.RMS[i];

//		if( dmmRegs.RMS[4] == 0 )
			vrms &= ~0xFFllu;									// ignore noise on LSB

		if( fUseCalib[channel] )								// apply calibration coefficients
			v = sqrt( fabs( pow( curCfg->mul, 2 ) * (double)vrms - pow( curCal->Add, 2 ) ) ) * curCal->Mult;
		else
			v = curCfg->mul * sqrt( (double)vrms );
	}
	else if( scale >= SCALE_50_nF && scale <= SCALE_500_uF )	// Read Mode I
	{
		DMM_GetCmdSPI( REG_CTA, 10, dmmRegs.CTA );				// read counter registers & INTF
		if( ! dmmRegs.INTF.CTF )								// CTF=0 : conversion not yet ready
		{
			if( pbErr ) *pbErr = bResult;
			return NAN;
		}

		// TODO - NOT WORKING !!!
		// ---
		uint32_t cnt = 0;
		for( i = sizeof(dmmRegs.CTA)-1; i < sizeof(dmmRegs.CTA); --i )
			cnt = ( cnt << 8 ) | dmmRegs.CTA[i];
		dRawVal[0] = (double)cnt;

		v = cnt * curCfg->mul;
		if( fUseCalib[channel] )							// apply calibration coefficients
			v = v * curCal->Mult + curCal->Add;
		// ---

		// reset CTA
		i = 0x70; DMM_SendCmdSPI( CS_DMM, REG_20, 1, &i );	// write R20=70h - SCMPI=011, ENCMP=1, ENCNTI=0, ENPCMPO=0, ENCTR=0 (disable counters)
		dmmRegs.CTA[0] = 0x00;
		dmmRegs.CTA[1] = 0x00;
		dmmRegs.CTA[2] = 0xBF;
		DMM_SendCmdSPI( CS_DMM, REG_CTA, sizeof(REG_CTA), dmmRegs.CTA );	// write CTA=BF0000h (preset counter)
		i = 0x72; DMM_SendCmdSPI( CS_DMM, REG_20, 1, &i );					// write R20=72h - SCMPI=011, ENCMP=1, ENCNTI=0, ENPCMPO=0, ENCTR=1 (re-enable counters)

		// debug
		cnt = 0;
		for( i = sizeof(REG_CTB)-1; i < sizeof(REG_CTB); --i )
			cnt = ( cnt << 8 ) | dmmRegs.CTB[i];
		dRawVal[1] = (double)cnt;

		cnt = 0;
		for( i = sizeof(REG_CTC)-1; i < sizeof(REG_CTC); --i )
			cnt = ( cnt << 8 ) | dmmRegs.CTC[i];
		dRawVal[2] = (double)cnt;
	}
	else if( scale >= SCALE_5_mF && scale <= SCALE_50_mF )	// Read Mode II
	{
		DMM_GetCmdSPI( REG_AD1, 3, dmmRegs.AD1 );			// read AD1 register
		DMM_GetCmdSPI( REG_INTF, 1, &dmmRegs.INTF.reg );	// read INTF register
		if( ! dmmRegs.INTF.AD1F )							// conversion ready ?
		{
			if( pbErr ) *pbErr = bResult;
			return NAN;
		}

		// TODO - NOT WORKING !!!

		// Read and sign extend AD1
		int32_t vad1 = ( (int32_t)dmmRegs.AD1[ 2 ] << 24 )
					 | ( (int32_t)dmmRegs.AD1[ 1 ] << 16 )
					 | ( (int32_t)dmmRegs.AD1[ 0 ] << 8 );
		vad1 /= 256;
		dRawVal[0] = (double)vad1;

		if(		 vad1 >= 0x7FFFFE )	 v = +INFINITY;			// value outside convertor range
		else if( vad1 <= -0x7FFFFE ) v = -INFINITY;			// value outside convertor range
		else
		{
			v = curCfg->mul * vad1;

			if( fUseCalib[channel] )						// apply calibration coefficients
				v = v * curCal->Mult + curCal->Add;
		}
	}
	else if( scale == SCALE_FREQ )
	{
#if 1
		DMM_GetCmdSPI( REG_CTSTA, 11, &dmmRegs.CTSTA.reg );	// read CTSTA, CTC, CTB, CTA & INTF
#else
		DMM_GetCmdSPI( REG_INTF, 1, &dmmRegs.INTF.reg );	// read INTF
#endif
		if( ! dmmRegs.INTF.CTF )							// CTF=0 : conversion not yet ready
		{
			if( pbErr ) *pbErr = bResult;
			return NAN;
		}

#if 0
		DMM_GetCmdSPI( REG_CTSTA, 1, &dmmRegs.CTSTA.reg );	// read CTSTA
#endif
		if( dmmRegs.CTSTA.CTBOV )							// CTBOV=1 : overflow
		{
			if( pbErr ) *pbErr = bResult;
			return 0;
		}
#if 0
		DMM_GetCmdSPI( REG_CTA, 3, dmmRegs.CTA );			// read counter register CTA
		DMM_GetCmdSPI( REG_CTB, 3, dmmRegs.CTB );			// read counter register CTB
		DMM_GetCmdSPI( REG_CTC, 3, dmmRegs.CTC );			// read counter register CTC
#endif
		uint32_t val = 0;
		for( i = sizeof(dmmRegs.CTA)-1; i < sizeof(dmmRegs.CTA); --i )	// assemble CTA value
			val = ( val << 8 ) | dmmRegs.CTA[i];
		uint32_t T1 = 0x1000000U - CTA_Initial + val;
		if( T1 == 0 ) T1 = 1;

		val = 0;
		for( i = sizeof(dmmRegs.CTB)-1; i < sizeof(dmmRegs.CTB); --i )	// assemble CTB value
			val = ( val << 8 ) | dmmRegs.CTB[i];
		v = ( (uint64_t)val * (uint32_t)CRYSTAL ) / T1;		// frequency [Hz]

		val = 0;
		for( i = sizeof(dmmRegs.CTC)-1; i < sizeof(dmmRegs.CTC); --i )	// assemble CTC value
			val = ( val << 8 ) | dmmRegs.CTC[i];
		dMeasuredVal[1] = 100.0 * val / T1;								// duty cycle [0..1]
	}
	else													// DC, RES, DIODE, CONT, TEMP use AD1
	{
		DMM_GetCmdSPI( REG_AD1, 3, dmmRegs.AD1 );			// read AD1 register
		DMM_GetCmdSPI( REG_INTF, 1, &dmmRegs.INTF.reg );	// read INTF register
		if( ! dmmRegs.INTF.AD1F )							// conversion ready ?
		{
			if( pbErr ) *pbErr = bResult;
			return NAN;
		}

		// Read and sign extend AD1
		int32_t vad1 = ( (int32_t)dmmRegs.AD1[ 2 ] << 24 )
					 | ( (int32_t)dmmRegs.AD1[ 1 ] << 16 )
					 | ( (int32_t)dmmRegs.AD1[ 0 ] << 8 );
		vad1 /= 256;

		if(		 vad1 >= 0x7FFFFE )	 v = +INFINITY;		// value outside convertor range
		else if( vad1 <= -0x7FFFFE ) v = -INFINITY;		// value outside convertor range
		else
		{
			v = curCfg->mul * vad1;						// scaling factor, i.e. make 0x800000 = full-scale-range

			if( fUseCalib[channel] )					// apply calibration coefficients
			{
				v = v * curCal->Mult + curCal->Add;
			}
		}
	}

	if( pbErr ) *pbErr = ERRVAL_SUCCESS;
	return v;
}

/***	DMMCMD_CmdMeasure
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
 **		This function implements the repeated session functionality for DMMMeasureRep and DMMMeasureRaw text commands of DMMCMD module.
 **		The function calls the DMM_DGetValue, eventually without calibration parameters being applied for DMMMeasureRaw.
 */
uint8_t DMM_Measure( uint8_t channel )
{
	if( --channel >= NUM_CHANNELS )
		return ERRVAL_CMD_WRONGPARAMS;

	if( nAvgCount[channel] == 0 )
		return ERRVAL_CMD_NO_TRIGGER;

	int curScale = DMM_GetScale( channel+1 );
	if( DMM_isScale( curScale ) != ERRVAL_SUCCESS )
		return ERRVAL_CMD_WRONGPARAMS;

	uint8_t bErrCode = ERRVAL_SUCCESS;
	double dVal = DMM_DGetStatus( channel+1, &bErrCode );
	if( bErrCode != ERRVAL_SUCCESS )
		return ERRVAL_CMD_BUSY;

	if( DMM_isNAN( dVal ) || dVal == +INFINITY || dVal == -INFINITY )
		return ERRVAL_CALIB_NANDOUBLE;

	if( DMM_isAC( curScale ) )					// AC uses Quadratic Mean Average value
		dVal = pow( dVal, 2 );

	dValAvg[channel] += dVal;

	if( --nAvgCount[channel] )					// start next measurement
	{
		if(	curScale == SCALE_FREQ )
		{
			DMM_StartFreqMeasure();
		}
		else if( curScale >= SCALE_50_nF &&
				 curScale <= SCALE_500_uF )
		{
			DMM_StartCapMeasurement();
		}

		return ERRVAL_CMD_BUSY;
	}

	// measurement done
	dVal = dValAvg[channel] / (double)nAvgPasses[channel];
	if( DMM_isAC( curScale ) )
		dVal = sqrt( dVal );

	dMeasuredVal[channel] = dVal;
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

	memset( fUseCalib, 1, NUM_CHANNELS );					// controls if calibration coefficients should be applied in DMM_DGetStatus
	memset( nAvgPasses, 1, NUM_CHANNELS );					// total number of averaging passes to do
	memset( nAvgCount, 0, NUM_CHANNELS );					// 0: current measurement finished, else number of passes still to go

	CALIB_Init();
}

/* *****************************************************************************
 End of File
 */
