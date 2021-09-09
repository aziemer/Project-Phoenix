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
#define ERRVAL_CMD_NO_TRIGGER           0xF3    // Measurement not triggered
#define ERRVAL_CMD_VALFORMAT            0xF2    // The numeric value cannot be extracted from the provided string.
#define ERRVAL_CMD_BUSY                 0xF1    // Measurement in progress
#define ERRVAL_CALIB_MISSINGMEASUREMENT 0xF0    // A measurement must be performed before calling the finalize calibration.

#define ERRVAL_OVERFLOW					0xEF

#define DMM_DIODEOPENTHRESHOLD			3

enum DMM_MODE {
	DmmIllegal = -1,

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
	DmmDiode,

	DMM_CAL_ZERO,

	DMM_CNTMODES
};

enum DM_SCALE {
	SCALE_ILLEGAL = -1,

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
	SCALE_ALT,
	SCALE_NONE,
	SCALE_PERCENT,
	SCALE_SAVED
};

enum {
	TEMP_CELSIUS	= 1,
	TEMP_FAHRENHEIT = 2,
	TEMP_KELVIN		= 3
};

#define DMM_VALIDDATA_CNTTIMEOUT    0x100   // number of valid data retrieval re-tries

#define DMMVoltageDC50Scale				SCALE_DC_50V
#define DMM_Voltage50DCLinearCoeff_P3   -1.59128E-06
#define DMM_Voltage50DCLinearCoeff_P1   1.003918916
#define DMM_Voltage50DCLinearCoeff_P0   0.000196999

#define NUM_CHANNELS	3

typedef struct _DMMCFG{
    int mode;			// scale
    double range;		// full scale range
    char *fmt;			// display format, i.e. "%-2.3f"
    uint8_t sw;			// switch bits
    uint8_t cfg[21];	// configuration bits: 0x1F...0x33
    double mul;			// dmm measurement (ad1/rms) multiplication factor to get value in corresponding unit
} DMMCFG;

enum {
	REG_AD1		= 0x00,
	REG_AD2		= 0x03,
	REG_LPF		= 0x06,
	REG_RMS		= 0x09,
	REG_PKHMIN	= 0x0E,
	REG_PKHMAX	= 0x11,
	REG_CTSTA	= 0x14,
	REG_CTC		= 0x15,
	REG_CTB		= 0x18,
	REG_CTA		= 0x1B,
	REG_INTF	= 0x1E,
	REG_INTE	= 0x1F,
	REG_20		= 0x20,
	REG_21,
	REG_22,
	REG_23,
	REG_24,
	REG_25,
	REG_26,
	REG_27,
	REG_28,
	REG_29,
	REG_2A,
	REG_2B,
	REG_2C,
	REG_2D,
	REG_2E,
	REG_2F,
	REG_30,
	REG_31,
	REG_32,
	REG_33,
	REG_34,
	REG_35,
	REG_36,
	REG_37
} REG_ADDR;


typedef struct {
    uint8_t																																							AD1[3];		// 0x00
    uint8_t																																							AD2[3];		// 0x03
    uint8_t																																							LPF[3];		// 0x06
    uint8_t																																							RMS[5];		// 0x09
    uint8_t																																							PKHMIN[3];	// 0x0E
    uint8_t																																							PKHMAX[3];	// 0x11
    union { uint8_t reg; struct { uint8_t CTBOV:1; uint8_t _NO:3; uint8_t CMPLO:1; uint8_t CMPHO:1; uint8_t ACPO:1; uint8_t PCNTI:1; }; }							CTSTA;		// 0x14
    uint8_t																																							CTC[3];		// 0x15
    uint8_t																																							CTB[3];		// 0x18
   	uint8_t																																							CTA[3];		// 0x1B
   	union { uint8_t reg; struct { uint8_t CTF:1; uint8_t AD2F:1; uint8_t AD1F:1; uint8_t LPFF:1; uint8_t RMSF:1; uint8_t _NO:2; uint8_t BORF:1; }; }				INTF;		// 0x1E
   	union { uint8_t reg; struct { uint8_t CTIE:1; uint8_t AD2IE:1; uint8_t AD1IE:1; uint8_t LPFIE:1; uint8_t RMSIE:1; uint8_t _NO:3; }; }							INTE;		// 0x1F
    union { uint8_t reg; struct { uint8_t _NO:1; uint8_t ENCTR:1; uint8_t ENPCMPO:1; uint8_t ENCNTI:1; uint8_t ENCMP:1; uint8_t SCMPI:3; }; }						R20;		// 0x20
    union { uint8_t reg; struct { uint8_t SCMPRL:4; uint8_t SCMPRH:4; }; }																							R21;		// 0x21
    union { uint8_t reg; struct { uint8_t AD1OSR:3; uint8_t AD1CHOP:2; uint8_t AD1OS:3; }; }																		R22;		// 0x22
    union { uint8_t reg; struct { uint8_t AD1INBUF:1;uint8_t AD1IPBUF:1;uint8_t AD1RLBUF:1;uint8_t AD1RHBUF:1;uint8_t AD1RG:1;uint8_t _NO:2;uint8_t ENAD1:1; }; }	R23;		// 0x23
    union { uint8_t reg; struct { uint8_t SAD1FN:3; uint8_t SDIO:1; uint8_t SAD1FP:4; }; }																			R24;		// 0x24
    union { uint8_t reg; struct { uint8_t OPS1:1; uint8_t OPS2:1; uint8_t SACM:2; uint8_t AD1IG:2; uint8_t AD2IG:2; }; }											R25;		// 0x25
    union { uint8_t reg; struct { uint8_t AD2OSR:3; uint8_t SAD2CLK:1; uint8_t AD2RG:1; uint8_t ENCHOPAD1:1; uint8_t _NO:1; uint8_t ENAD2:1; }; }					R26;		// 0x26
    union { uint8_t reg; struct { uint8_t SAD2RL:2; uint8_t SAD2RH:2; uint8_t SAD2IN:2; uint8_t SAD2IP:2; }; }														R27;		// 0x27
    union { uint8_t reg; struct { uint8_t SAD1RL:3; uint8_t _NO1:1; uint8_t SAD1RH:3; uint8_t _NO2:1; }; }															R28;		// 0x28
    union { uint8_t reg; struct { uint8_t PKHSEL:2; uint8_t ENPKH:1; uint8_t LPFBW:3; uint8_t ENLPF:1; uint8_t ENRMS:1; }; }										R29;		// 0x29
    union { uint8_t reg; struct { uint8_t SS0:1; uint8_t FS0:1; uint8_t DS0:1; uint8_t PS0:1; uint8_t SS1:1; uint8_t FS1:1; uint8_t DS1:1; uint8_t PS1:1; }; }		R2A;		// 0x2A
    union { uint8_t reg; struct { uint8_t SS2:1; uint8_t FS2:1; uint8_t DS2:1; uint8_t PS2:1; uint8_t SS3:1; uint8_t FS3:1; uint8_t DS3:1; uint8_t PS3:1; }; }		R2B;		// 0x2B
    union { uint8_t reg; struct { uint8_t SS4:1; uint8_t FS4:1; uint8_t DS4:1; uint8_t PS4:1; uint8_t SS5:1; uint8_t FS5:1; uint8_t DS5:1; uint8_t PS5:1; }; }		R2C;		// 0x2C
    union { uint8_t reg; struct { uint8_t SS6:1; uint8_t FS6:1; uint8_t DS6:1; uint8_t PS6:1; uint8_t SS7:1; uint8_t FS7:1; uint8_t DS7:1; uint8_t PS7:1; }; }		R2D;		// 0x2D
    union { uint8_t reg; struct { uint8_t SS8:1; uint8_t FS8:1; uint8_t DS8:1; uint8_t PS8:1; uint8_t SS9:1; uint8_t FS9:1; uint8_t DS9:1; uint8_t PS9:1; }; }		R2E;		// 0x2E
    union { uint8_t reg; struct { uint8_t SMODE:7; uint8_t ENVS:1; }; }																								R2F;		// 0x2F
    union { uint8_t reg; struct { uint8_t ACC:7; uint8_t SREFO:1; }; }																								R30;		// 0x30
    union { uint8_t reg; struct { uint8_t SFUVR:4; uint8_t SAGND:2; uint8_t ENBIAS:1; uint8_t ENREFO:1; }; }														R31;		// 0x31
    union { uint8_t reg; struct { uint8_t SOP1P:3; uint8_t ENOP1:1; uint8_t SOP2P:3; uint8_t ENOP2:1; }; }															R32;		// 0x32
    union { uint8_t reg; struct { uint8_t SAD1I:2; uint8_t SFT1:2; uint8_t ENXI:1; uint8_t ENOSC:1; uint8_t OP1CHOP:2; }; }											R33;		// 0x33
    union { uint8_t reg; struct { uint8_t _NO:2; uint8_t SDO23:1; uint8_t SVXI:1; uint8_t AD3RG:1; uint8_t ENCHOPAD3:1; uint8_t _NO2:1; uint8_t ENAD3:1; }; }		R34;		// 0x34
    union { uint8_t reg; struct { uint8_t _NO:2; uint8_t AD3IG:2; uint8_t SAD3IN:2; uint8_t SAD3IP:2; }; }															R35;		// 0x35
    union { uint8_t reg; uint8_t SPHACAL; }																															R36;		// 0x36
    uint8_t																																							R37;		// 0x37
} __attribute__((__packed__)) DMMREGISTERS;

extern const DMMCFG dmmcfg[];
extern const char * const dmmranges[];

extern uint32_t currCTA, currCTB, currCTC;
extern double currAD1, currAD2, currAD3, currRMS;

extern double dMeasuredVal[NUM_CHANNELS];

uint8_t	DMM_SetScale( uint8_t ch, int idxScale );
int		DMM_GetScale( uint8_t ch );
int		DMM_FindScale( int mode, double range );

int		DMM_GetMode( int idxScale );
double	DMM_GetRange( int idxScale );
uint8_t	DMM_GetScaleUnit( uint8_t scale, double *pdScaleFact, char *szUnitPrefix, char *szUnit, char *szRange );
const char *DMM_GetFormat( int idxScale );

// value functions
uint8_t	DMM_Ready( uint8_t channel );
void	DMM_Trigger( uint8_t channel );
uint8_t	DMM_Measure( uint8_t channel );

void	DMM_SetUseCalib( uint8_t channel, uint8_t f );
char	DMM_GetUseCalib( uint8_t channel );

void	DMM_SetAveraging( uint8_t channel, uint8_t nAvg );
uint8_t	DMM_GetAveraging( uint8_t channel );

void	DMM_SetGateTime( uint16_t ms );
uint16_t DMM_GetGateTime( void );

void	DMM_SetTempUnits( uint8_t units );
uint8_t	DMM_GetTempUnits( void );

uint8_t	DMM_isNAN( double dVal );

uint8_t	DMM_isScale( int idxScale );
uint8_t	DMM_isDC( int idxScale );
uint8_t	DMM_isAC( int idxScale );
uint8_t	DMM_isVOLT( int idxScale );
uint8_t	DMM_isCURR( int idxScale );
uint8_t	DMM_isCAP( int idxScale );
uint8_t	DMM_isRES( int idxScale );
uint8_t DMM_isFRES( int idxScale );
uint8_t	DMM_isCAP( int idxScale );
uint8_t	DMM_isDIOD( int idxScale );
uint8_t	DMM_isCONT( int idxScale );

void	DMM_Init( void );

#endif /* __DMMCFG_H */

/* *****************************************************************************
 End of File
 */
