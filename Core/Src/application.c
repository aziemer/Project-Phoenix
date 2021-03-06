/*
 * application.c
 *
 *  Created on: 14.08.2021
 *      Author: aziemer
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

//#include "stm32f1xx_hal_conf.h"

#include "rtc.h"
#include "usart.h"
#include "main.h"
#include "gpio.h"
#include "tft.h"
#include "kbd.h"
#include "scpi.h"

#include "dmm.h"
#include "calib.h"
#include "application.h"

/*
 * RTC BKUP register (NVM) assignment:
 *
 * DR0:
 * DR1:		flags
 * DR2:		last state
 * DR3:
 * DR4:
 * DR5:
 * DR6:
 * DR7:
 * DR8:
 * DR9:
 * DR10:
 *
 */

static struct {
	uint16_t RestoreScale : 1;		// restore last scale before power off
} APP_flags;

static void SetTempUnits( uint8_t which, int state );
static void SetRelMode( uint8_t which, int state );

static const MENU Menus[] = {
	{
		"Resistance",
		{
			{ "RES 4W",			NULL,		SetScale,	1, SCALE_ALT },
			{ "CONT",			NULL,		SetScale,	1, SCALE_CONT },
			{ "DIODE",			NULL,		SetScale,	1, SCALE_DIODE },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ "REL",			NULL,		SetRelMode,	1, -1 }
		}
	},
	{
		"4W Resistance",
		{
			{ "RES 2W",			NULL,		SetScale,	1, SCALE_ALT },
			{ "CONT",			NULL,		SetScale,	1, SCALE_CONT },
			{ "DIODE",			NULL,		SetScale,	1, SCALE_DIODE },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ "REL",			NULL,		SetRelMode,	1, -1 }
		}
	},
	{
		"DC Voltage",
		{
			{ "AC",				NULL,		SetScale,	1, SCALE_ALT },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ "REL",			NULL,		SetRelMode,	1, -1 }
		}
	},
	{
		"AC Voltage",
		{
			{ "DC",				NULL,		SetScale,	1, SCALE_ALT },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ "REL",			NULL,		SetRelMode,	1, -1 }
		}
	},
	{
		"DC Current",
		{
			{ "AC",				NULL,		SetScale,	1, SCALE_ALT },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ "REL",			NULL,		SetRelMode,	1, -1 }
		}
	},
	{
		"AC Current",
		{
			{ "DC",				NULL,		SetScale,	1, SCALE_ALT },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ "REL",			NULL,		SetRelMode,	1, -1 }
		}
	},
	{
		"Capacitance",
		{
			{ NULL,				NULL,		NULL,		0, 0 },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ "REL",			NULL,		SetRelMode,	1, -1 }
		}
	},
	{
		"Frequency",
		{
			{ NULL,				NULL,		NULL,		0, 0 },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ "REL",			NULL,		SetRelMode,	1, -1 }
		}
	},
	{
		"Temperature",
		{
			{ "Celsius",		NULL,		SetTempUnits, TEMP_CELSIUS, 0 },
			{ "Fahrenheit",		NULL,		SetTempUnits, TEMP_FAHRENHEIT, 0 },
			{ "Kelvin",			NULL,		SetTempUnits, TEMP_KELVIN, 0 },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ "REL",			NULL,		SetRelMode,	1, -1 }
		}
	},
	{
		"Continuity",
		{
			{ "Beeper",			"On",		NULL,		0, 0 },
			{ "RES 2W",			NULL,		SetScale,	1, SCALE_50_kOhm },
			{ "RES 4W",			NULL,		SetScale,	1, SCALE_4W_50_kOhm },
			{ "DIODE",			NULL,		SetScale,	1, SCALE_DIODE },
			{ NULL,				NULL,		NULL,		0, 0 }
		}
	},
	{
		"Diode",
		{
			{ "Res 2W",			NULL,		SetScale,	1, SCALE_50_kOhm },
			{ "Res 4W",			NULL,		SetScale,	1, SCALE_4W_50_kOhm },
			{ "CONT",			NULL,		SetScale,	1, SCALE_CONT },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ NULL,				NULL,		NULL,		0, 0 }
		}
	},
	{
		"Utility",
		{
			{ "Date",			NULL,		NULL,		1, 0 },
			{ "Time",			NULL,		NULL,		1, 0 },
			{ "Zero",			NULL,		NULL,		1, 0 },
			{ NULL,				NULL,		NULL,		0, 0 },
			{ "Back",			NULL,		NULL,		0, 0 }
		}
	}
};

static const MENU *curMenu = NULL;

static uint8_t hold = 0;
static uint8_t autorange = 1;
static uint8_t dualmode = 0;
static uint8_t relmode = 0;
static double relVal = 0;

static void DrawTime( uint8_t force )
{
	const char *dayname[7] = { "Su", "Mo", "Tu", "We", "Th", "Fr", "Sa" };
	RTC_DateTypeDef sDate;
	RTC_TimeTypeDef sTime;
	char tmp[30];
	static uint8_t last_min = -1;

	HAL_RTC_GetTime( &hrtc, &sTime, RTC_FORMAT_BIN );
	if( last_min != sTime.Minutes || force )
	{
		HAL_RTC_GetDate( &hrtc, &sDate, RTC_FORMAT_BIN );
		sprintf( tmp, "  %s %02d.%02d.%04d %02d:%02d", dayname[sDate.WeekDay%7], sDate.Date, sDate.Month, sDate.Year + 2000, sTime.Hours, sTime.Minutes );

		TFT_setFont( TIME_FONT );
		TFT_setXPos( TFT_WIDTH - TFT_getStrWidth( tmp ) - 7 );
		TFT_setYPos( HEADER_HEIGHT - 9 );
		TFT_setForeGround( TIME_COLOR );
		TFT_setBackGround( HEADER_COLOR );
		TFT_printf( tmp );

		last_min = sTime.Minutes;
	}
}

static void DrawHeader( void )
{
	TFT_setForeGround( HEADER_COLOR );
	TFT_fillRect( 0, 0, TFT_WIDTH-1, HEADER_HEIGHT-1 );

	// Draw TITLE
	TFT_setFont( HEADER_FONT );
	TFT_setXPos( 10 );
	TFT_setYPos( HEADER_HEIGHT - 8 );
	TFT_setForeGround( 0 );
	TFT_setBackGround( HEADER_COLOR );
	TFT_printf( curMenu->legend );
}

void DrawFooter( char *msg, ... )
{
	char txt[50];
	uint32_t i;
	va_list ap;

	for(;;)
	{
		uint16_t ypos = BUTTON_YPOS( 4 );

		if( msg )
		{
			va_start( ap, msg );
			vsnprintf( txt, sizeof(txt), msg, ap );
			va_end( ap );

			uint8_t len = strlen( txt );
			while( ( len = TFT_getStrWidth( txt ) ) > BUTTON_XPOS - SPACING - 1 && i )			// truncate 1st line, if too long
				txt[ i-1 ] = 0;
		}
		else
		{
			int scale = DMM_GetScale( 1 );
			DMM_GetScaleUnit( scale, NULL, NULL, NULL, txt );
		}

		TFT_setForeGround( FOOTER_COLOR );
		TFT_setBackGround( BACKGROUND_COLOR );
		TFT_fillRoundRect( 2, ypos, BUTTON_XPOS - SPACING + 1, ypos + BUTTON_HEIGHT );

		TFT_setFont( FOOTER_FONT );
		TFT_setForeGround( FOOTER_TEXT_COLOR );
		TFT_setBackGround( FOOTER_COLOR );
		TFT_setXPos( 10 );
		TFT_setYPos( BUTTON_YPOS( 4 ) + BUTTON_HEIGHT - ( BUTTON_HEIGHT - TFT_getFontHeight() ) / 2 - 2 );	// base-line !!
		TFT_printf( txt );

		if( !msg )
			return;

		msg = NULL;

		for( i = 0; i < 0x500000; ++i )
			asm volatile ("nop");
	}
}

static uint8_t format_value( char *str, char spc, double Val, double fullscale, uint8_t scale )
{
	if( DMM_isNAN( Val ) || Val == +INFINITY || Val == -INFINITY )
	{
		strcpy( str, DMM_isCONT( scale ) ? "OPEN " : "OVER " );
		return ERRVAL_CMD_VALFORMAT;
	}
#if 1
	if( abs( Val ) > 1.1 * fullscale )
	{
		strcpy( str, "RANGE " );
		return ERRVAL_CMD_VALFORMAT;
	}
#endif
	else if( DMM_isDIOD( scale ) && Val > DMM_DIODEOPENTHRESHOLD )
	{
		strcpy( str, "OPEN " );
		return 1;
	}
	else
	{
		char tmp[20];
		uint8_t dot, l, i, pos;

		l = sprintf( tmp, DMM_GetFormat( scale ), Val );

		for( dot = 0; ( dot < l ) && ( tmp[dot] != '.' ); ++dot )
			;

		// group units
		pos = dot % 3;
		for( i = 0; i < dot; ++i )
		{
			if( pos == 0 && i )
				*str++ = spc;
			if( --pos > 2 ) pos = 2;

			*str++ = tmp[i];
		}

		// group decimals, if present
		if( dot < l )
		{
			*str++ = '.';

			pos = 0;
			for( i = 0; i < ( l - dot ); ++i )
			{
				if( ( pos == 0 ) && ( i > 0 ) )
					*str++ = spc;
				if( --pos > 2 ) pos = 2;

				*str++ = tmp[dot+1 + i];
			}
		}

		*str = 0;
	}

	return ERRVAL_SUCCESS;
}

static void DrawValue( void )
{
	uint8_t scale, pbErr;
	double dScaleFact, dFullScale, Val;
	char szValue[PREC+10] = "";
	char szUnitPrefix[10] = "";

	if( hold ) return;

	scale = DMM_GetScale( 1 );

	pbErr = DMM_isScale( scale );
	if( pbErr == ERRVAL_SUCCESS )
		pbErr = DMM_GetScaleUnit( scale, &dScaleFact, szUnitPrefix, NULL, NULL );

	if( pbErr == ERRVAL_SUCCESS )
	{
		Val = dMeasuredVal[0] * dScaleFact;
		if( relmode == 1 )
		{
			relVal = Val;
			relmode = 2;
		}
		else if( relmode == 2 )
			Val -= relVal;

		dFullScale = DMM_GetRange( scale ) * dScaleFact;

		TFT_setFont( VALUE1_FONT );
		TFT_setForeGround( VALUE1_COLOR );
		TFT_setBackGround( BACKGROUND_COLOR );
		TFT_setXPos( VALUE1_XPOS );
		TFT_setYPos( VALUE1_YPOS );

		pbErr = format_value( szValue, ',', Val, dFullScale, scale );
		if( pbErr != ERRVAL_SUCCESS )	// OVER / OPEN
		{
			TFT_setForeGround( BACKGROUND_COLOR );
			TFT_fillRect( VALUE1_XPOS, VALUE1_YPOS - 50, UNIT1_XPOS - 2, VALUE1_YPOS  );

			TFT_setForeGround( VGA_RED );
			TFT_setFont( ERROR_FONT );
			TFT_setFontSize( ERROR_FONT_SIZE );
			TFT_setXPos( VALUE1_XPOS + 75 );
			TFT_setYPos( VALUE1_YPOS - 5 );
		}
		TFT_printf( szValue );
		TFT_setFontSize( 1 );
	}

//	if( pbErr == ERRVAL_SUCCESS )
	{
//#define DEBUG
#ifdef DEBUG
		TFT_setFont( VALUE2_FONT );
		TFT_setForeGround( VALUE2_COLOR );
		TFT_setBackGround( BACKGROUND_COLOR );

		TFT_setXPos( VALUE2_XPOS );
		TFT_setYPos( VALUE2_YPOS );
		TFT_printf( "%08lu  ", currCTA );

		TFT_setXPos( VALUE2_XPOS );
		TFT_setYPos( VALUE2_YPOS + 30 );
		TFT_printf( "%08lu  ", currCTB );

		TFT_setXPos( VALUE2_XPOS );
		TFT_setYPos( VALUE2_YPOS + 60 );
		TFT_printf( "%08lu  ", currCTC );

		TFT_setXPos( VALUE2_XPOS + 150 );
		TFT_setYPos( VALUE2_YPOS );
		TFT_printf( "%08lu  ", currAD1 );

		TFT_setXPos( VALUE2_XPOS + 150 );
		TFT_setYPos( VALUE2_YPOS + 30 );
		TFT_printf( "%010lu  ", currRMS );

		TFT_setXPos( VALUE2_XPOS + 150 );
		TFT_setYPos( VALUE2_YPOS + 60 );
		TFT_printf( "%11.2f  ", (double)currCTC / currCTB );
#else
		if( dualmode )
		{
			TFT_setFont( VALUE2_FONT );
			TFT_setForeGround( VALUE2_COLOR );
			TFT_setBackGround( BACKGROUND_COLOR );

			if( scale == SCALE_FREQ )
				snprintf( szValue, 9, "%5.1f  ", dMeasuredVal[1] );		// duty cycle
			else if( DMM_isAC( scale ) )
				snprintf( szValue, 9, "%5.0f  ", dMeasuredVal[1] );		// frequency
			else
				sprintf( szValue, "-" );

			TFT_setXPos( VALUE2_XPOS );
			TFT_setYPos( VALUE2_YPOS );
			TFT_printf( szValue );
		}
#if 0
		if( dualmode == 2 )
		{
			TFT_setFont( VALUE3_FONT );
			TFT_setForeGround( VALUE3_COLOR );
			TFT_setBackGround( BACKGROUND_COLOR );

			pbErr = DMM_GetScaleUnit( 1, &dScaleFact, szUnitPrefix, NULL, NULL );
			if( pbErr == ERRVAL_SUCCESS )
			{
				dFullScale = DMM_GetRange( scale ) * dScaleFact;
				Val = dMeasuredVal[2] * dScaleFact;
				pbErr = format_value( szValue, ' ', Val, dFullScale, scale );
			}
			else
				sprintf( szValue, " ..... " );

			TFT_setXPos( VALUE3_XPOS );
			TFT_setYPos( VALUE3_YPOS );
			TFT_printf( szValue );
		}
# endif

#endif
	}
}

void SetHold( int mode )
{
	while( KBD_Read() )					// wait until released
		;

	switch( mode )
	{
	case 1:		hold = 1; break;
	case 0:		hold = 0; break;
	default:	hold = !hold; break;
	}

	TFT_setFont( INDICATOR_FONT );
	TFT_setForeGround( hold ? INDICATOR_ON_COLOR : INDICATOR_OFF_COLOR );
	TFT_setBackGround( BACKGROUND_COLOR );
	TFT_setXPos( HOLD_XPOS );
	TFT_setYPos( HOLD_YPOS );
	TFT_printf( "HOLD" );
}

void SetAuto( int mode )
{
	while( KBD_Read() )					// wait until released
		;

	switch( mode )
	{
	case 1:		autorange = 1; break;
	case 0:		autorange = 0; break;
	default:	autorange = !autorange; break;
	}

	TFT_setFont( INDICATOR_FONT );
	TFT_setForeGround( autorange ? INDICATOR_ON_COLOR : BACKGROUND_COLOR );
	TFT_setBackGround( BACKGROUND_COLOR );
	TFT_setXPos( BUTTON_XPOS - TFT_getStrWidth( "HOLD" ) - 10 );
	TFT_setYPos( AUTO_YPOS );
	TFT_printf( "AUTO" );
}

static void SetRelMode( uint8_t which, int state )
{
	while( KBD_Read() )					// wait until released
		;

	switch( state )
	{
	case 1:		relmode = 1; break;
	case 0:		relmode = 0; break;
	default:	relmode = !relmode; break;
	}

	TFT_setFont( INDICATOR_FONT );
	TFT_setForeGround( relmode ? INDICATOR_ON_COLOR : BACKGROUND_COLOR );
	TFT_setBackGround( BACKGROUND_COLOR );
	TFT_setXPos( HOLD_XPOS );
	TFT_setYPos( HOLD_YPOS + 20 );
	TFT_printf( "REL" );
}

static void SetTempUnits( uint8_t which, int state )
{
	if( which == 0 || which > 3 )
	{
		switch( DMM_GetTempUnits() )
		{
		case TEMP_CELSIUS:		which = TEMP_FAHRENHEIT; break;
		case TEMP_FAHRENHEIT:	which = TEMP_KELVIN; break;
		default:				which = TEMP_CELSIUS; break;
		}
	}
	DMM_SetTempUnits( which );
}

static void DoMenu( int mode )	// no = -1: reload current menu
{
	char tmp1[10], tmp2[10];
	uint8_t no;

	if( mode < 0 )
		mode = DMM_GetMode( DMM_GetScale( 1 ) );
	else if( mode >= DMM_CNTMODES )
		return;

	while( KBD_Read() )					// wait until released
		;

	curMenu = &Menus[mode];

	// Draw BUTTONS
	for( no = 0; no < 5; ++no )
	{
		uint16_t ypos = BUTTON_YPOS( no );
		size_t len;

		TFT_setForeGround( BUTTON_COLOR );
		TFT_setBackGround( BACKGROUND_COLOR );
		TFT_fillRoundRect( BUTTON_XPOS, ypos, TFT_WIDTH-1, ypos + BUTTON_HEIGHT );

		TFT_setForeGround( BUTTON_TEXT_COLOR );
		TFT_setBackGround( BUTTON_COLOR );

		strcpy( tmp1, curMenu->button[no].legend1 );
		strcpy( tmp2, curMenu->button[no].legend2 );

		if( tmp2[0] == 0 )						// single line
		{
			TFT_setFont( BUTTON_FONT1 );
			len = TFT_getStrWidth( tmp1 );
			if( len > BUTTON_WIDTH - 2 )		// shrink font size, if too long
			{
				TFT_setFont( BUTTON_FONT2 );
				len = TFT_getStrWidth( tmp1 );
				if( len > BUTTON_WIDTH - 2 )	// still too long, try wrapping
				{
					uint8_t c, l = strlen( tmp1 );
					for( c = 0; c < l; ++c )
					{
						if( tmp1[c] < '0' )
						{
							strcpy( tmp2, tmp1 + c + 1 );
							if( tmp1[c] == ' ' )
								tmp1[c] = 0;
							else
								tmp1[c+1] = 0;
							break;
						}
					}
					goto DUAL_LINE;
				}
			}

			TFT_setXPos( BUTTON_XPOS + ( BUTTON_WIDTH - len ) / 2 );
			TFT_setYPos( ypos + BUTTON_HEIGHT - ( BUTTON_HEIGHT - TFT_getFontHeight() ) / 2 - 4 );
			TFT_printf( tmp1 );
			continue;
		}

DUAL_LINE:

		TFT_setFont( BUTTON_FONT2 );

		uint8_t i = strlen( tmp1 );
		while( ( len = TFT_getStrWidth( tmp1 ) ) > BUTTON_WIDTH-1 && i )			// truncate 1st line, if too long
			tmp1[ i-1 ] = 0;

		TFT_setXPos( BUTTON_XPOS + ( BUTTON_WIDTH - len ) / 2 );
		TFT_setYPos( ypos + BUTTON_HEIGHT / 2 - ( BUTTON_HEIGHT - TFT_getFontHeight() ) / 2 + 9 );
		TFT_printf( tmp1 );

		i = strlen( tmp2 );
		while( ( len = TFT_getStrWidth( tmp2 ) ) > BUTTON_WIDTH-1 && i )			// truncate 2nd line, if too long
			tmp2[ i-1 ] = 0;

		if( curMenu->button[no].legend2[0] )	// if dual line, overlay 2nd background
		{
			TFT_setForeGround( BUTTON_VALUE_COLOR );
			TFT_fillRoundRect( BUTTON_XPOS, ypos + BUTTON_HEIGHT / 2, TFT_WIDTH-1, ypos + BUTTON_HEIGHT );

			TFT_setForeGround( BUTTON_TEXT_COLOR );
			TFT_setBackGround( BUTTON_VALUE_COLOR );
		}

		TFT_setXPos( BUTTON_XPOS + ( BUTTON_WIDTH - len ) / 2 );
		TFT_setYPos( ypos + BUTTON_HEIGHT - ( BUTTON_HEIGHT - TFT_getFontHeight() ) / 2 + 9 );
		TFT_printf( tmp2 );
	}

	DrawHeader();
	DrawFooter( NULL );
	DrawTime( 1 );

	SetAuto( autorange );					// redraw AUTO message
}

static void MenuFunction( uint8_t no )		// soft buttons below the display
{
	// call linked function
	if( --no < 5 && curMenu->button[no].callback )
		curMenu->button[no].callback( curMenu->button[no].cbChannel, curMenu->button[no].cbValue );

	DoMenu( -1 );							// redraw current menu with possibly changed parameters
}

void SetScale( uint8_t channel, int scale )
{
	uint8_t i, err = ERRVAL_SUCCESS;
	int curScale = DMM_GetScale( channel );
	int curMode = DMM_GetMode( curScale );
	double fsr = DMM_GetRange( curScale );

	SetHold( 0 );

	switch( scale )
	{
	case SCALE_AUTO:
		SetAuto( 1 );
		break;

	case SCALE_SAVED:
		if( APP_flags.RestoreScale )
			scale = HAL_RTCEx_BKUPRead( &hrtc, RTC_BKP_DR2 );
		else
			scale = SCALE_DC_50V;

		if( DMM_isScale( scale ) != ERRVAL_SUCCESS )
			scale = SCALE_DC_50V;
		break;

	case SCALE_UP:
		SetAuto( 0 );
		if( DMM_isScale( curScale + 1 ) != ERRVAL_SUCCESS ||
			DMM_GetMode( curScale + 1 ) != curMode )
			return;
		scale = curScale + 1;
		break;

	case SCALE_DOWN:
		SetAuto( 0 );
		if( DMM_isScale( curScale - 1 ) != ERRVAL_SUCCESS ||
			DMM_GetMode( curScale - 1 ) != curMode )
			return;
		scale = curScale - 1;
		break;

	case SCALE_ALT:		// AC<->DC, Ohm->Continuity->Diode->Ohm
		SetAuto( 1 );
		switch( curMode )
		{
		case DmmResistance:		scale = SCALE_4W_500_Ohm; break;
		case DmmResistance4W:	scale = SCALE_CONT; break;
		case DmmContinuity:		scale = SCALE_DIODE; break;
		case DmmDiode:			scale = SCALE_500_Ohm; break;

		case DmmDCVoltage:
			scale = DMM_FindScale( DmmACVoltage, fsr );
			break;
		case DmmACVoltage:
			scale = DMM_FindScale( DmmDCVoltage, fsr );
			break;

		case DmmDCCurrent:
			scale = DMM_FindScale( DmmACCurrent, fsr );
			break;
		case DmmACCurrent:
			scale = DMM_FindScale( DmmDCCurrent, fsr );
			break;

		case DmmCapacitance:
		case DmmFrequency:
			break;

		case DmmTemperature:
			switch( curScale == DmmTemperature ? DMM_GetTempUnits() : 0 )
			{
			case TEMP_CELSIUS:		i = TEMP_FAHRENHEIT; break;
			case TEMP_FAHRENHEIT:	i = TEMP_KELVIN; break;
			default:				i = TEMP_CELSIUS; break;
			}
			DMM_SetTempUnits( i );
			break;
		}
		break;

	default:	// direct select
//		SetAuto( 0 );
		break;
	}

	if( DMM_isScale( scale ) != ERRVAL_SUCCESS )
		err = ERRVAL_CMD_WRONGPARAMS;

	if( err == ERRVAL_SUCCESS && scale != curScale )
		err = DMM_SetScale( channel, scale );

	if( err == ERRVAL_SUCCESS )
	{
		double dScaleFact;
		char szUnitPrefix[50] = "", szUnit[50] = "";

		if( scale > -1 )
		{
			uint16_t xpos = UNIT1_XPOS;
			curMode = DMM_GetMode( scale );
			if( curMode == DmmFrequency )
			{
				dualmode = 1;
				xpos += 40;
			}
			else if( DMM_isAC( scale ) )
				dualmode = 1;
			else
				dualmode = 0;

			TFT_setForeGround( BACKGROUND_COLOR );
			TFT_fillRect( 0, AUTO_YPOS + 6, BUTTON_XPOS - 5, BUTTON_YPOS(4) - 5 );

			err = DMM_GetScaleUnit( scale, &dScaleFact, szUnitPrefix, szUnit, NULL );
			if( err == ERRVAL_SUCCESS )
				strcat( szUnitPrefix, szUnit );

			TFT_setFont( UNIT1_FONT );
			TFT_setForeGround( UNIT1_COLOR );
			TFT_setBackGround( BACKGROUND_COLOR );
			TFT_setXPos( xpos );
			TFT_setYPos( UNIT1_YPOS );
			TFT_printf( szUnitPrefix );

			TFT_setForeGround( BACKGROUND_COLOR );
			TFT_fillRect( TFT_getXPos(), TFT_getYPos() - TFT_getFontHeight(), BUTTON_XPOS - 10, TFT_getYPos() + 10 );	// must be TOP/LEFT -> BOTTOM/RIGHT

			if( dualmode )
			{
				if( scale == SCALE_FREQ )
				{
					*szUnitPrefix = 0;
					*szUnit = '%'; szUnit[1] = 0;
					err = ERRVAL_SUCCESS;
				}
				else if( DMM_isAC( scale ) )
					err = DMM_GetScaleUnit( SCALE_FREQ, &dScaleFact, szUnitPrefix, szUnit, NULL );
				else
				{
					*szUnitPrefix = 0;
					*szUnit = 0;
					err = ERRVAL_SUCCESS;
				}

				if( err == ERRVAL_SUCCESS )
					strcat( szUnitPrefix, szUnit );

				TFT_setForeGround( UNIT2_COLOR );
				TFT_setBackGround( BACKGROUND_COLOR );
				TFT_setXPos( UNIT2_XPOS );
				TFT_setYPos( UNIT2_YPOS );
				TFT_printf( "%s", szUnitPrefix );

				TFT_setForeGround( BACKGROUND_COLOR );
				TFT_fillRect( TFT_getXPos(), TFT_getYPos() - TFT_getFontHeight(), BUTTON_XPOS - 10, TFT_getYPos() + 10 );	// must be TOP/LEFT -> BOTTOM/RIGHT
			}
#if 0
			if( dualmode == 2 )
			{
				err = DMM_GetScaleUnit( scale, &dScaleFact, szUnitPrefix, szUnit, NULL );
				if( err == ERRVAL_SUCCESS )
					strcat( szUnitPrefix, szUnit );

				TFT_setForeGround( UNIT3_COLOR );
				TFT_setBackGround( BACKGROUND_COLOR );
				TFT_setXPos( UNIT3_XPOS );
				TFT_setYPos( UNIT3_YPOS );
				TFT_printf( szUnitPrefix );

				TFT_setForeGround( BACKGROUND_COLOR );
				TFT_fillRect( TFT_getXPos(), TFT_getYPos() - TFT_getFontHeight(), BUTTON_XPOS - 10, TFT_getYPos() + 10 );	// must be TOP/LEFT -> BOTTOM/RIGHT
			}
#endif
			DMM_Trigger( channel );
		}
	}

	if( err == ERRVAL_SUCCESS )
	{
		curMode = DMM_GetMode( scale );
		DoMenu( curMode );
	}
}
#if 0
static void RangeCalib( uint8_t which, int state )
{
	uint8_t key;
	int idxScale = DMM_GetScale( 1 );
	if( DMM_isScale( idxScale ) != ERRVAL_SUCCESS )
		return;

	for( state = 0; state < 3; ++state )
	{
		while( KBD_Read() )					// wait until released
			;

		TFT_setForeGround( VGA_BLACK );
		TFT_setBackGround( VGA_BLACK );
		TFT_fillRect( 0, HEADER_HEIGHT + 1, BUTTON_XPOS - 2, BUTTON_YPOS(4) - 1);

		TFT_setFont( FONT_10X16 );
		TFT_setForeGround( VGA_WHITE );
		TFT_setXPos( 5 );
		TFT_setYPos( HEADER_HEIGHT + 20 );
		switch( state )
		{
		case 0:
			if(		 DMM_isFRES(idxScale) )	TFT_printf( "4W short inputs" );
			else if( DMM_isCURR(idxScale) )	TFT_printf( "Remove all input signals" );
			else							TFT_printf( "Short HI & LO inputs" );
			break;

		case 1:
			if(		 DMM_isFRES(idxScale) )	TFT_printf( "Apply resistance %s", dmmranges[idxScale] );
			else if( DMM_isCURR(idxScale) )	TFT_printf( "Apply %s to I & LO inputs", dmmranges[idxScale] );
			else if( DMM_isDIOD(idxScale) )	TFT_printf( "Apply 3.0V to HI & LO inputs" );
			else							TFT_printf( "Apply %s to HI & LO inputs", dmmranges[idxScale] );
			break;

		case 2:
			if(		 DMM_isCURR(idxScale) )	TFT_printf( "Apply -%s to I & LO inputs", dmmranges[idxScale] );
			else							TFT_printf( "Apply -%s to HI & LO inputs", dmmranges[idxScale] );
			break;
		}

		TFT_setXPos( 5 );
		TFT_setYPos( HEADER_HEIGHT + 40 );
		TFT_printf( "Press RANGE to start, DUAL EXIT to abort" );

		while( ( key = KBD_Read() ) == 0 )
			;
		if( key == KEY_EXIT )
			break;

		uint8_t bResult = 0;
		switch( state )
		{
		case 0:	bResult = CALIB_CalibOnZero( NULL ); break;
		case 1: bResult = CALIB_CalibOnPositive( dmmcfg[idxScale].range, NULL ); break;
		case 2: bResult = CALIB_CalibOnNegative( 0.0 - dmmcfg[idxScale].range, NULL ); break;
		}

		if( bResult != ERRVAL_SUCCESS )
		{
			TFT_setForeGround( VGA_BLACK );
			TFT_setBackGround( VGA_BLACK );
			TFT_fillRect( 0, HEADER_HEIGHT + 1, BUTTON_XPOS - 2, BUTTON_YPOS(4) - 1);

			TFT_setForeGround( VGA_RED );
			TFT_setXPos( 5 );
			TFT_setYPos( HEADER_HEIGHT + 20 );
			TFT_printf( "FAILED (%02X)!", bResult );

			TFT_setXPos( 5 );
			TFT_setYPos( HEADER_HEIGHT + 80 );
			TFT_printf( "DONE - press any key" );

			while( !KBD_Read() )		// wait until pressed
				;
			while( KBD_Read() )			// wait until released
				;

			break;
		}

		if( state == 2 || ( state == 1 && ( DMM_isFRES(idxScale) || DMM_isDIOD(idxScale) ) ) )
		{
			TFT_setForeGround( VGA_BLACK );
			TFT_setBackGround( VGA_BLACK );
			TFT_fillRect( 0, HEADER_HEIGHT + 1, BUTTON_XPOS - 2, BUTTON_YPOS(4) - 1);

			TFT_setForeGround( VGA_WHITE );
			TFT_setXPos( 5 );
			TFT_setYPos( HEADER_HEIGHT + 20 );
			TFT_printf( "MULT = %1.5f", calib[idxScale].Mult );

			TFT_setXPos( 5 );
			TFT_setYPos( HEADER_HEIGHT + 40 );
			TFT_printf( "ADD  = %1.5f", calib[idxScale].Add );

			TFT_setXPos( 5 );
			TFT_setYPos( HEADER_HEIGHT + 80 );
			TFT_printf( "DONE - press any key" );

			while( !KBD_Read() )		// wait until pressed
				;
			while( KBD_Read() )			// wait until released
				;

			break;
		}
	}

	TFT_setForeGround( VGA_BLACK );
	TFT_setBackGround( VGA_BLACK );
	TFT_fillRect( 0, HEADER_HEIGHT + 1, BUTTON_XPOS - 2, BUTTON_YPOS(4) - 1);
	DoMenu( -1 );
}
#endif

void Application( void )
{
	uint8_t key, last_key = 0;
	uint16_t repeat_timeout = 0;

	KBD_Init();
	DMM_Init();

	TFT_setForeGround( BACKGROUND_COLOR );
	TFT_fillRect( 0, 0, TFT_WIDTH-1, TFT_HEIGHT-1 );		// Clear screen

	*(uint16_t*)&APP_flags = HAL_RTCEx_BKUPRead( &hrtc, RTC_BKP_DR1 );	// read NVM flags

	SetScale( 1, SCALE_SAVED );
	SetAuto( 1 );
	SetHold( 0 );
	SetRelMode( 0, 0 );

	DMM_SetAveraging( 1, 1 );
	DMM_SetUseCalib( 1, 1 );

	for(;;)
	{
		Do_SCPI();

		DrawTime( 0 );

		switch( DMM_Measure( 1 ) )
		{
		case ERRVAL_SUCCESS:
			DrawValue();
			// no break
		case ERRVAL_CMD_NO_TRIGGER:
//		case ERRVAL_CALIB_NANDOUBLE:
			DMM_Trigger( 1 );
			break;
		}

		key = KBD_Read();

		if( key == last_key && key && !hold )
		{
			if( ++repeat_timeout == START_REPEAT )
				last_key = 0;
		}

		if( key != last_key )
		{
			int curMode = DMM_GetMode( DMM_GetScale( 1 ) );

			repeat_timeout = 0;

			switch( key )
			{
			// below display
			case KEY_VOLT:	SetScale( 1, ( curMode == DmmDCVoltage || curMode == DmmACVoltage ) ? SCALE_ALT : SCALE_DC_50V ); break;
			case KEY_AMP:	SetScale( 1, ( curMode == DmmDCCurrent || curMode == DmmACCurrent ) ? SCALE_ALT : SCALE_DC_500mA ); break;
			case KEY_OHM:	SetScale( 1, ( curMode == DmmResistance || curMode == DmmResistance4W || curMode == DmmDiode || curMode == DmmContinuity ) ? SCALE_ALT : SCALE_500_Ohm ); break;
			case KEY_FREQ:	SetScale( 1, SCALE_FREQ ); break;
			case KEY_CAP:	SetScale( 1, SCALE_50_nF ); break;
			case KEY_TEMP:	SetScale( 1, ( curMode == DmmTemperature ) ? SCALE_ALT : SCALE_TEMP ); break;

			// soft keys right
			case KEY_F1:	MenuFunction( 1 ); break;
			case KEY_F2:	MenuFunction( 2 ); break;
			case KEY_F3:	MenuFunction( 3 ); break;
			case KEY_F4:	MenuFunction( 4 ); break;
			case KEY_F5:	MenuFunction( 5 ); break;

			// Run/Stop
			case KEY_RUN:	SetHold( -1 ); break;

			case KEY_RANGE:	SetScale( 1, SCALE_AUTO ); break;
			case KEY_UP:	SetScale( 1, SCALE_UP ); break;
			case KEY_DOWN:	SetScale( 1, SCALE_DOWN ); break;

			case KEY_UTIL:	DoMenu( DMM_CAL_ZERO ); break;

			default:		break;
#if 0
			case KEY_LEFT:	break;
			case KEY_RIGHT:	break;
			case KEY_DUAL:	break;
			case KEY_EXIT:	break;
			case KEY_MATH:	break;
			case KEY_SAVE:	break;
			case KEY_REC:	break;
			case KEY_PORT:	break;
#endif
			}
		}
		last_key = key;
	}
}

