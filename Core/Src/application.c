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

typedef void (*CALLBACK)(uint8_t,int);

typedef struct {
	char *legend1;			// 1st or middle line
	char *legend2;			// 2nd line
	uint8_t state;
	CALLBACK callback;
	uint8_t cbChannel;
	int cbValue;
} BUTTON;

typedef struct {
	char *legend;
	uint8_t state;
	uint8_t id;
	BUTTON button[5];
} MENU;

static MENU Menu[] = {
	{
		"Voltage",		1, 1,
		{
			{ "AC/DC",			NULL,		1,	SetScale,	1, SCALE_ALT },
			{ "Long Text",		NULL,		1,	NULL,		0, 0 },
			{ "Longer Text",	NULL,		1,	NULL,		0 },
			{ "Variable",		"Value", 	1,	NULL,		0 },
			{ NULL,				NULL,		0,	NULL,		0 }
		}
	},
	{
		"Current",		2, 1,
		{
			{ "AC/DC",			NULL,		1,	SetScale,	1, SCALE_ALT },
			{ NULL,				NULL,		0,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 }
		}
	},
	{
		"Resistance",	3, 1,
		{
			{ "2W/4W",			NULL,		1,	SetScale,	1, SCALE_ALT },
			{ NULL,				NULL,		0,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 }
		}
	},
	{
		"Continuity",	4, 1,
		{
			{ "Beep",			"On",		1,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 }
		}
	},
	{
		"Diode",		5, 1,
		{
			{ "Hi/Lo",			NULL,		1,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 }
		}
	},
	{
		"Capacitance",	6, 1,
		{
			{ NULL,				NULL,		0,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 }
		}
	},
	{
		"Frequency",	7, 1,
		{
			{ NULL,				NULL,		0,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 },
			{ NULL,				NULL,		0,	NULL,		0, 0 }
		}
	}
};

static MENU *curMenu = NULL;

static uint8_t hold = 0;
static uint8_t autorange = 1;
static uint8_t dualmode = 1;

#if 0
static void DrawDot( uint16_t color )
{
	setColor( color );
	fillCircle( TFT_WIDTH - HEADER_HEIGHT / 2, HEADER_HEIGHT / 2, HEADER_HEIGHT / 2 - 2 );
}
#endif

static RTC_TimeTypeDef sTime;
static RTC_DateTypeDef sDate;

static void DrawTime( void )
{
	static uint8_t last_min = 0xFF;

	HAL_RTC_GetTime( &hrtc, &sTime, RTC_FORMAT_BIN );	// MUST read both time and date, and in this order !
	HAL_RTC_GetDate( &hrtc, &sDate, RTC_FORMAT_BIN );

	if( last_min != sTime.Minutes )
	{
		char tmp[20];

		sprintf( tmp, "%02d.%02d. %02d:%02d", sDate.Date, sDate.Month, sTime.Hours, sTime.Minutes );

		TFT_setFont( TIME_FONT );
		TFT_setXPos( TFT_WIDTH - TFT_getStrWidth( tmp ) - 7 );
		TFT_setYPos( HEADER_HEIGHT - 9 );
		TFT_setForeGround( TIME_COLOR );
		TFT_setBackGround( HEADER_COLOR );
		TFT_printf( tmp );
	}
}

/*
void InitTime()
{
	RTC_TimeTypeDef sTime1;
	RTC_DateTypeDef sDate1;

	sDate1.Date = 17;
	sDate1.Month = 8;
	sDate1.Year = 21;
	sTime1.Hours = 23;
	sTime1.Minutes = 40;
	HAL_RTC_SetTime( &hrtc, &sTime1, RTC_FORMAT_BIN );
	HAL_RTC_SetDate( &hrtc, &sDate1, RTC_FORMAT_BIN );
}
*/

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

static void DrawFooter( char *msg, ... )
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
			DMM_GetScaleUnit( 1, NULL, NULL, NULL, txt );
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

static void DrawValue( void )
{
	uint8_t scale, mode, pbErr;
	double dScaleFact, dFullScale, Val;
	char szValue[PREC+10] = "";
	char szUnitPrefix[10] = "";

//	DrawDot( VGA_FUCHSIA );

	if( hold ) return;

	scale = DMM_GetScale( 1 );
	mode = DMM_GetMode( scale );
	pbErr = DMM_GetScaleUnit( 1, &dScaleFact, szUnitPrefix, NULL, NULL );
	if( pbErr == ERRVAL_SUCCESS )
	{
		dFullScale = DMM_GetRange( scale );
		Val = dMeasuredVal[0] * dScaleFact;

		if( Val > 1.1 * dFullScale )
			strcpy( szValue, "+++++++" );
		if( Val < -1.1 * dFullScale )
			strcpy( szValue, "-------" );
		else if( mode == DmmFrequency )
			snprintf( szValue, 10, "%8.0f", dMeasuredVal[0] );
		else if( mode == DmmCapacitance )
			snprintf( szValue, PREC, "%1.5f", Val );
		else
			snprintf( szValue, PREC, "%+1.4f", Val );
	}

	if( pbErr != ERRVAL_SUCCESS )
		sprintf( szValue, " ..... " );

	szValue[PREC] = 0;		// just in case...

	TFT_setFont( VALUE1_FONT );
	TFT_setForeGround( VALUE1_COLOR );
	TFT_setBackGround( BACKGROUND_COLOR );
	TFT_setXPos( VALUE1_XPOS );
	TFT_setYPos( VALUE1_YPOS );
	TFT_printf( szValue );

	if( dualmode )
	{
//		scale = DMM_GetScale( 0 );
		pbErr = DMM_GetScaleUnit( 1, &dScaleFact, szUnitPrefix, NULL, NULL );
		if( pbErr == ERRVAL_SUCCESS )
		{
			dFullScale = DMM_GetRange( scale );
			Val = dMeasuredVal[1] * dScaleFact;

			if( Val > 1.1 * dFullScale )
				strcpy( szValue, "+++++++" );
			if( Val < -1.1 * dFullScale )
				strcpy( szValue, "-------" );
			else if( mode == DmmCapacitance )
				snprintf( szValue, PREC, " %1.4f", Val );
			else if( mode == DmmFrequency )
				snprintf( szValue, PREC, "%2.1f", dMeasuredVal[1] );
			else
				snprintf( szValue, PREC, "%+1.4f", Val );
		}

		if( pbErr != ERRVAL_SUCCESS )
			sprintf( szValue, " ..... " );
		szValue[PREC] = 0;		// just in case...

		TFT_setFont( VALUE2_FONT );
		TFT_setForeGround( VALUE2_COLOR );
		TFT_setBackGround( BACKGROUND_COLOR );
		TFT_setXPos( VALUE2_XPOS );
		TFT_setYPos( VALUE2_YPOS );
		TFT_printf( szValue );
	}

	if( dualmode == 2 )
	{
//		scale = DMM_GetScale( 3 );
		pbErr = DMM_GetScaleUnit( 1, &dScaleFact, szUnitPrefix, NULL, NULL );
		if( pbErr == ERRVAL_SUCCESS )
		{
			dFullScale = DMM_GetRange( scale );
			Val = dMeasuredVal[1] * dScaleFact;

			if( Val > 1.1 * dFullScale )
				strcpy( szValue, "+++++++" );
			if( Val < -1.1 * dFullScale )
				strcpy( szValue, "-------" );
			else if( mode == DmmCapacitance || mode == DmmFrequency )
				snprintf( szValue, PREC, "%1.5f", Val );
			else
				snprintf( szValue, PREC, "%+1.4f", Val );
		}

		if( pbErr != ERRVAL_SUCCESS )
			sprintf( szValue, " ..... " );
		szValue[PREC] = 0;		// just in case...

		TFT_setFont( VALUE3_FONT );
		TFT_setForeGround( VALUE3_COLOR );
		TFT_setBackGround( BACKGROUND_COLOR );
		TFT_setXPos( VALUE3_XPOS );
		TFT_setYPos( VALUE3_YPOS );
		TFT_printf( szValue );
	}

//	DrawDot( VGA_LIME );
}

void SetHold( int mode )
{
	switch( mode )
	{
	case 0:		hold = 0; break;
	case 1:		hold = 1; break;
	default:	hold = !hold; break;
	}

	TFT_setFont( HOLD_FONT );
	TFT_setForeGround( hold ? INDICATOR_ON_COLOR : INDICATOR_OFF_COLOR );
	TFT_setBackGround( BACKGROUND_COLOR );
	TFT_setXPos( HOLD_XPOS );
	TFT_setYPos( HOLD_YPOS );
	TFT_printf( "HOLD" );
}

void SetAuto( int mode )
{
	switch( mode )
	{
	case 0:		autorange = 0; break;
	case 1:		autorange = 1; break;
	default:	autorange = !autorange; break;
	}

	TFT_setFont( AUTO_FONT );
	TFT_setForeGround( autorange ? INDICATOR_ON_COLOR : INDICATOR_OFF_COLOR );
	TFT_setBackGround( BACKGROUND_COLOR );
	TFT_setXPos( BUTTON_XPOS - TFT_getStrWidth( "HOLD" ) - 10 );
	TFT_setYPos( AUTO_YPOS );
	TFT_printf( "AUTO" );
}

static void DoMenu( int no )	// no=0: (re)load current menu
{
	char tmp1[10], tmp2[10];

	if( no == 0 )
		no = curMenu->id - 1;
	else if( --no >= sizeof(Menu)/sizeof(MENU) )
		return;

	if( Menu[no].state == 0 )		// not allowed
		return;

	curMenu = &Menu[no];

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
	SetAuto( -1 );
}

static void MenuFunction( uint8_t no )		// soft buttons to the right of the display
{
	// call linked function
	if( --no < 5 && curMenu->button[no].callback )
		curMenu->button[no].callback( curMenu->button[no].cbChannel, curMenu->button[no].cbValue );

	DoMenu( 0 );		// redraw current menu with possibly changed parameters
}

static int findScale( uint8_t mode, double fsr )	// AC<->DC, 2W<->4W
{
	uint8_t idxScale;

	if( mode < 0 )		// oops
		return -1;

	for( idxScale = 0; idxScale < DMM_CNTSCALES; ++idxScale )
	{
		if( DMM_GetMode( idxScale ) == mode &&
			DMM_GetRange( idxScale ) == fsr )
		{
			return idxScale;
		}
	}
	return -1;
}

void SetScale( uint8_t channel, int scale )
{
	int curScale = DMM_GetScale( channel );
	int curMode = DMM_GetMode( curScale );
	double fsr = DMM_GetRange( curScale );
	uint8_t err = ERRVAL_SUCCESS;
	uint8_t menu = 0;

	SetHold( 0 );

	switch( scale )
	{
	case SCALE_AUTO:
		SetAuto( 1 );
		break;

	case SCALE_UP:
		SetAuto( 0 );
		if( DMM_GetMode( curScale + 1 ) == curMode )
			scale = curScale + 1;
		else
			scale = curScale;
		break;

	case SCALE_DOWN:
		SetAuto( 0 );
		if( DMM_GetMode( curScale - 1 ) == curMode )
			scale = curScale - 1;
		else
			scale = curScale;
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
			scale = findScale( DmmACVoltage, fsr );
			break;
		case DmmACVoltage:
			scale = findScale( DmmDCVoltage, fsr );
			break;

		case DmmDCCurrent:
			scale = findScale( DmmACCurrent, fsr );
			break;
		case DmmACCurrent:
			scale = findScale( DmmDCCurrent, fsr );
			break;

		case DmmCapacitance:
		case DmmFrequency:
		case DmmTemperature:
			break;
		}
		break;

	default:	// direct select
//		SetAuto( 0 );
		break;
	}

	if( DMM_ERR_CheckIdxCalib( scale ) != ERRVAL_SUCCESS )
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
			if( curMode == DmmCapacitance )		// debug counter values
				dualmode = 2;
			else if( curMode == DmmFrequency )	// debug counter values
			{
				dualmode = 1;
				xpos += 40;
			}
			else
				dualmode = 0;

			TFT_setForeGround( BACKGROUND_COLOR );
			TFT_fillRect( 0, AUTO_YPOS + 6, BUTTON_XPOS - 5, BUTTON_YPOS(4) - 5 );

			err = DMM_GetScaleUnit( 1, &dScaleFact, szUnitPrefix, szUnit, NULL );
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
				err = DMM_GetScaleUnit( 2, &dScaleFact, szUnitPrefix, szUnit, NULL );
				if( err == ERRVAL_SUCCESS )
					strcat( szUnitPrefix, szUnit );

				TFT_setForeGround( UNIT2_COLOR );
				TFT_setBackGround( BACKGROUND_COLOR );
				TFT_setXPos( UNIT2_XPOS );
				TFT_setYPos( UNIT2_YPOS );
				TFT_printf( szUnitPrefix );

				TFT_setForeGround( BACKGROUND_COLOR );
				TFT_fillRect( TFT_getXPos(), TFT_getYPos() - TFT_getFontHeight(), BUTTON_XPOS - 10, TFT_getYPos() + 10 );	// must be TOP/LEFT -> BOTTOM/RIGHT
			}

			if( dualmode == 2 )
			{
				err = DMM_GetScaleUnit( 3, &dScaleFact, szUnitPrefix, szUnit, NULL );
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
		}
	}

	if( err == ERRVAL_SUCCESS )
	{
		curMode = DMM_GetMode( scale );
		switch( curMode )
		{
		case DmmDCVoltage:
		case DmmACVoltage:		menu = 1; break;
		case DmmDCCurrent:
		case DmmACCurrent:		menu = 2; break;
		case DmmResistance:
		case DmmResistance4W:	menu = 3; break;
		case DmmContinuity:		menu = 4; break;
		case DmmDiode:			menu = 5; break;
		case DmmCapacitance:	menu = 6; break;
		case DmmFrequency:		menu = 7; break;

		case DmmTemperature:
		default:				break;
		}
	}

	DoMenu( menu );
}

void Application( void )
{
	uint8_t key, last_key = 0;
	uint16_t repeat_timeout = 0;

//	InitTime();

	KBD_Init();

	DMM_Init();

	TFT_setForeGround( BACKGROUND_COLOR );
	TFT_fillRect( 0, 0, TFT_WIDTH-1, TFT_HEIGHT-1 );		// Clear screen

	SetScale( 1, SCALE_DC_1kV );
	SetAuto( 1 );

	for(;;)
	{
		Do_SCPI();

		DrawTime();

		if( DMM_Measure( 1, 0, 1 ) == ERRVAL_SUCCESS )		// channel1, scaled value, averaging = 1 (no)
			DrawValue();

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

			default:		break;
#if 0
			case KEY_CAP:	SetScale( 1, SCALE_50_nF ); break;
			case KEY_TEMP:	SetScale( 1, SCALE_TEMP ); break;
			case KEY_LEFT:	break;
			case KEY_RIGHT:	break;
			case KEY_DUAL:	break;
			case KEY_EXIT:	break;
			case KEY_MATH:	break;
			case KEY_SAVE:	break;
			case KEY_REC:	break;
			case KEY_PORT:	break;
			case KEY_UTIL:	break;
#endif
			}
		}
		last_key = key;
	}
}

