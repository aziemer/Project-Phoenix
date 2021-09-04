/*
 * scpi.c
 *
 *  Created on: 20.08.2021
 *      Author: aziemer
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "version.h"

#include "rtc.h"
#include "usart.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include "gpio.h"
#include "tft.h"
#include "kbd.h"

#include "dmm.h"
#include "calib.h"
#include "application.h"

typedef enum {		// must be same order as above !
	SCPI_IDN,
	SCPI_RST,
	SCPI_OPC,		// missing in original software !!
	SCPI_SENS,
	SCPI_CONF,
	SCPI_CALC,
	SCPI_SYST,
	SCPI_AUTO,
	SCPI_RANG,
	SCPI_RATE,
	SCPI_MEAS,
	SCPI_FUNC,
	SCPI_VOLT,
	SCPI_CURR,
	SCPI_FREQ,
	SCPI_PER,
	SCPI_CAP,
	SCPI_CONT,
	SCPI_DIOD,
	SCPI_FRES,
	SCPI_RES,
	SCPI_TEMP,
	SCPI_RTD,
	SCPI_TYP,
	SCPI_UNIT,
	SCPI_SHOW,
	SCPI_SCAL,
	SCPI_THRE,
	SCPI_AVER,
	SCPI_ALL,
	SCPI_MAX,
	SCPI_MIN,
	SCPI_DBM,
	SCPI_DB,
	SCPI_REF,
	SCPI_NULL,
	SCPI_OFFS,
	SCPI_STAT,
	SCPI_BEEP,
	SCPI_DATE,
	SCPI_TIME,
	SCPI_LOC,
	SCPI_REM,
	SCPI_DEF,
	SCPI_CAL,
	SCPI_SEC,
	SCPI_VAL,
	SCPI_AC,
	SCPI_DC,
	SCPI_NONE,

	SCPI_NUM_STRINGS
} SCPI_STRINGS;

const char * const scpi_keywords[] = {
	"*IDN",
	"*RST",
	"*OPC",
	"SENSe",
	"CONFigure",
	"CALCulate",
	"SYSTem",
	"AUTO",
	"RANGe",
	"RATE",
	"MEAS",				// with ? only
	"FUNCtion",
	"VOLTage",
	"CURRent",
	"FREQuency",
	"PERiod",
	"CAPacitance",
	"CONTinuity",
	"DIODe",
	"FourwireRESistance",
	"RESistance",
	"TEMPerature",
	"RTD",
	"TYPe",
	"UNIT",
	"SHOW",
	"SCALe",
	"THREshold",
	"AVERage",
	"ALL",
	"MAXimum",
	"MINimum",
	"DBM",
	"DB",
	"REFerence",
	"NULL",
	"OFFSet",
	"STATe",
	"BEEPer",
	"DATE",
	"TIME",
	"LOCal",
	"REMote",
	"DEFault",
	"CALibrate",
	"SECure",
	"VALue",
	"AC",
	"DC",
	"NONe"
};

typedef struct _TRANSLATE {
	int in;
	int out;
} TRANSLATE;

TRANSLATE FUNC1_table[] = {
	{ SCPI_RES,			DmmResistance },
	{ SCPI_FRES,		DmmResistance4W },
	{ SCPI_VOLT,		DmmDCVoltage },
	{ SCPI_CURR,		DmmDCCurrent },
	{ SCPI_CAP,			DmmCapacitance },
	{ SCPI_FREQ,		DmmFrequency },
	{ SCPI_TEMP,		DmmTemperature },
	{ SCPI_CONT,		DmmContinuity },
	{ SCPI_DIOD,		DmmDiode },
	{ -1,				DmmIllegal }
};

TRANSLATE FUNC2_table[] = {
	{ SCPI_FREQ,		DmmFrequency },
	{ SCPI_NONE,		DmmIllegal },
	{ -1,				DmmIllegal }
};

#if 0
static struct {
	uint8_t Ready		: 1;	// all previous commands and *OPC have been executed
	uint8_t Unused1		: 1;	// 0
	uint8_t QueryError	: 1;	// trying to read the response buffer while it was empty
	uint8_t DeviceError	: 1;	// selftest or calibration error (error range -300, see SCPI error list)
	uint8_t ExecError	: 1;	// excution error (error range -200)
	uint8_t CmdError	: 1;	// command syntax error (error range -100)
	uint8_t Unused2		: 1;	// 0
	uint8_t PowerCycle	: 1;	// device was just turned on, gets reset after reading the event register
} SCPI_StandardEvent = { .PowerCycle = 1, };

static struct {
	uint8_t Unused1		: 2;	// for future use, return 0
	uint8_t ErrorQueue	: 1;	// one or more errors are in the error queue, use SYST:ERR? to reset
	uint8_t QuestData	: 1;	// one or more enabled bits (via STAT:QUES:ENAB) in the questionable data register are set
	uint8_t MsgAvail	: 1;	// the response buffer is filled
	uint8_t StdEvt		: 1;	// one or more enabled bits (via *ESE) in the standard event register are set
	uint8_t MasterSum	: 1;	// one or more enabled bits (via *SRE) in the status register are set and can trigger a RQS
	uint8_t StdOp		: 1;	// one or more enabled bits (via STAT:OPER:ENAB) in the standard operations register are set
} SCPI_StatusByte = { 0, };
#endif

static char cmd_buffer[100];
static uint8_t buf_pos = 0;
static uint8_t buf_ready = 1;
static uint8_t buf_busy = 0;

int translate( TRANSLATE *table, int val, uint8_t reverse )
{
	int x;

	for( x = 0; table[x].in >= 0; ++x )
	{
		if( reverse )
		{
			if( table[x].out == val ) return table[x].in;
		}
		else
		{
			if( table[x].in == val ) return table[x].out;
		}
	}
	return table[x].out;
}

int scpi_printf( const char *fmt, ... )
{
	va_list ap;
	char txt[100];
	va_start( ap, fmt );

	int l = vsnprintf( txt, sizeof(txt)-1, fmt, ap );
	HAL_UART_Transmit( &huart1, (uint8_t*)txt, l, 1000 );

	va_end( ap );
	return l;
}

static char *SCPI_Short( int kw )
{
	static char response[10];

	if( kw < 0 || kw >= SCPI_NUM_STRINGS )
		return "ERR";

	const char *s = scpi_keywords[ kw ];
	int x = 0;
	while( *s && x < sizeof(response) - 1 )
	{
		if( isupper( (uint8_t)*s ) || *s == ' ' || *s == '*' )
			response[x++] = *s;
		++s;
	}
	response[x] = 0;
	return response;
}

int SCPI_Match( char *keyword )
{
	uint8_t i;

	for( i = 0; i < SCPI_NUM_STRINGS; ++i )
	{
		const uint8_t *pattern;
		uint8_t *cmd = (uint8_t*)keyword;

		for( pattern = (const uint8_t*)scpi_keywords[i]; *pattern; ++pattern )
		{
			uint8_t ch = toupper( *cmd );

			if( *pattern == ' ' )					// space in SCPI string must match one or more whitespace(s) in command string
			{
				if( !*cmd || !isspace( *cmd ) )		// no, command has no space there
					break;

				while( *cmd && isspace( *cmd ) )	// yes, command had space(s), skip them
					++cmd;
			}
			else if( isupper( *pattern ) ||			// upper case SCPI character or star *MUST* match command character, ignoring it's case
					*pattern == '*' )
			{
				if( ch != *pattern )				// no, command character does not match
					break;
				++cmd;								// yes, command character matches, advance
			}
			else if( toupper( *pattern ) == ch )	// lower case SCPI character *MAY* match command character, ignoring it's case
			{
				++cmd;								// yes, that matches, advance
			}
			else if( isalpha( ch ) )				// if command character is not a delimiter or number, it must match the NEXT upper case character of the SCPI string
			{
				const uint8_t *p = pattern;
				while( *p && islower( *p ) )		// advance SCPI string to next upper case character - if any
					++p;

				if( ch != *p )						// no match either
					break;
				++cmd;								// yes, that matches, advance
			}
		}

		if( !*pattern )								// bingo!
			return i;
	}

	return -1;
}

#define MAX_SCPI_PATH 5
#define MAX_SCPI_PARM 5

static void parse_value( char *str, double *val, char *unit, uint8_t unit_size )
{
	char *p = str;

	if( *p == '-' || *p == '+' || *p == '.' || isdigit( (uint8_t)*p ) )
	{
		while( *p == '-' || *p == '+' || *p == '.' || toupper( (uint8_t)*p ) == 'E' || isdigit( (uint8_t)*p ) )
			++p;

		char x = *p;
		*p = 0;
		*val = atof( str );
		*p = x;
	}
	else
		*val = 0;

	if( *p )
	{
		strncpy( unit, p, unit_size-1 );
		unit[unit_size-1] = 0;
	}
	else
		*unit = 0;
}

char *scpi_show( int ch_index )
{
	int scale = DMM_GetScale( ch_index );
	if( scale < 0 ) return "NONE";

	char *s;
	int mode = DMM_GetMode( scale );
	switch( mode )
	{
	case DmmACVoltage:	s = "VOLT:AC"; break;
	case DmmACCurrent:	s = "CURR:AC"; break;
	default: s = SCPI_Short( translate( FUNC1_table, mode, 1 ) ); break;		// DmmXXX -> SCPI_XXX scale
	}

	sprintf( cmd_buffer, "%s %+1.4e\n", s, dmmcfg[scale].range );
	return cmd_buffer;
}

char *SCPI_Execute( char *command_string )
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	int keyword[MAX_SCPI_PATH];
	int ch_index;
	char *parameter[MAX_SCPI_PARM];
	uint8_t num_kw, idx, num_parm;
	char *p, *kw, delimiter = 0;
	char *cmd, *sep1 = command_string;
	char *parm, *sep2;
	double val;
	char unit[10];
	int s, scale, mode;

	while( ( cmd = strsep( &sep1, ";" ) ) != NULL )		// multiple commands can be chained
	{
		ch_index = -1;

		// traverse SCPI path
		for( num_kw = 0; num_kw < MAX_SCPI_PATH && *cmd; )
		{
			for( kw = p = cmd; isalpha( (uint8_t)*p ) || *p == '*'; ++p )
				;

			if( isdigit( (uint8_t)*p ) )
			{
				if( ch_index != -1 ) return NULL;		// channel index already on a previous keyword
				ch_index = atoi( p );					// TODO: check index
				for( *p = 0; isdigit( (uint8_t)*++p ); )
					;
			}

			if( ( delimiter = *p ) != 0 )
				*p++ = 0;
			cmd = p;

			keyword[num_kw++] = SCPI_Match( kw );

			if( delimiter != ':' )						// more to come ?
				break;
		}

		if( ch_index == -1 ) ch_index = 1;

		num_parm = 0;
		if( isspace( (uint8_t)delimiter ) )				// command with parameters
		{
			while( *cmd && isspace( (uint8_t)*cmd ) )
				++cmd;

			sep2 = cmd;
			while( num_parm < MAX_SCPI_PARM && ( parm = strsep( &sep2, "," ) ) != NULL )
				parameter[ num_parm++ ] = parm;
		}
		else if( delimiter != '?' )						// not a query -> command without parameters
			*cmd = 0;

		if( num_kw == 0 ) return NULL;					// nothing to do

#if 1
		for( s = 0; s < num_kw; ++s )
		{
			if( s ) scpi_printf( ":" );
			scpi_printf( SCPI_Short( keyword[s] ) );
		}

		if( delimiter ) scpi_printf( "%c", delimiter );
		scpi_printf( " " );

		for( s = 0; s < num_parm; ++s )
		{
			if( s ) scpi_printf( "," );
			scpi_printf( parameter[s] );
		}
		scpi_printf( "\n" );
#endif

		idx = 0;
		if( keyword[idx] == SCPI_SENS )				// a leading SENS is optional
		{
			if( ++idx == num_kw )						// but nothing else follows?
				return NULL;
		}

		switch( keyword[idx++] )
		{
		case SCPI_IDN:			// IEEE mandatory command
			if( delimiter == '?' )
			{
				if( *MANUFACTURER == 0xFF || *MODEL == 0xFF || *SERIALNO == 0xFF )	// missing calibration data (empty FLASH area @ 0x0801F800)!
					snprintf( cmd_buffer, sizeof(cmd_buffer)-1, "A-Z-E,Phoenix DMM,00001,%s", VER_SHORT );
				else
					snprintf( cmd_buffer, sizeof(cmd_buffer)-1, "%s,%s,%s,%s", MANUFACTURER, MODEL, SERIALNO, VER_SHORT );
				return cmd_buffer;
			}
			break;

		case SCPI_RST:			// IEEE mandatory command
			if( delimiter == '?' )
			{
				DMM_Init();
				TFT_setForeGround( BACKGROUND_COLOR );
				TFT_fillRect( 0, 0, TFT_WIDTH-1, TFT_HEIGHT-1 );		// Clear screen
				SetScale( 1, SCALE_DC_1kV );
				SetAuto( 1 );
			}
			break;

		case SCPI_OPC:			// IEEE mandatory command
			if( delimiter == '?' )
			{
				int state = DMM_Ready( ch_index );
				if( state >= 0 ) return state ? "1" : "0";
			}
			break;

		case SCPI_CONF:			// CONF[:SCAL][:<scale>][:AC|DC]{?| <range>}		// SCAL isoptional, if <scale> and/or <AC|DC> are missing, default to DCV
			if( delimiter == '?' ) return scpi_show( ch_index );					// query? -> show current scale/range
			if( num_parm == 0 ) return NULL;										// else a range parameter must be given

			if( keyword[idx] == SCPI_SCAL && ++idx == num_kw ) return NULL;			// skip SCAL keyword, if present

			mode = translate( FUNC1_table, keyword[idx], 0 );						// <scale> -> DmmXXX
			if( mode < 0 ) mode = DmmDCVoltage;										// default to DmmDCVoltage, happens i.e. if no <scale> is present
			else ++idx;

			if( idx < num_kw && ( mode == DmmDCVoltage || mode == DmmDCCurrent ) )
			{
				if(		 keyword[idx] == SCPI_AC )
				{
					if(		 mode == DmmDCVoltage )	mode = DmmACVoltage;
					else if( mode == DmmDCCurrent )	mode = DmmACCurrent;
					else							return NULL;					// bad AC parameter
				}
				else if( keyword[idx] != SCPI_DC )	return NULL;					// bad keyword
			}

			parse_value( parameter[0], &val, unit, sizeof(unit) );

			switch( mode )
			{
			case DmmTemperature:	SetScale( 1, SCALE_TEMP ); break;
			case DmmFrequency:		SetScale( 1, SCALE_FREQ ); break;
			case DmmDiode:			SetScale( 1, SCALE_DIODE ); break;

			default:
				scale = DMM_FindScale( mode, val );
				if( scale < 0 ) return NULL;										// bad range
				SetScale( 1, scale );
				break;
			}
			break;

		case SCPI_FUNC:		// [SENS:]FUNC[1|2] [<function>]
			if( ch_index > 2 ) return NULL;											// bad index
			if( delimiter == '?' ) return scpi_show( ch_index );
			if( num_parm )
			{
				s = SCPI_Match( parameter[0] );
				s = translate( ch_index == 2 ? FUNC2_table : FUNC1_table, s, 0 );
				if( s < 0 ) return "bad scale";
				SetScale( ch_index, s );
			}
			break;

		case SCPI_TEMP:
			break;

		case SCPI_SYST:
			switch( keyword[idx] )
			{
			case SCPI_DATE:
			case SCPI_TIME:
				{
					HAL_RTC_GetTime( &hrtc, &sTime, RTC_FORMAT_BIN );	// MUST read both time and date, and in this order !
					HAL_RTC_GetDate( &hrtc, &sDate, RTC_FORMAT_BIN );

					if( delimiter == '?' )
					{
						if( keyword[idx] == SCPI_DATE )
							sprintf( cmd_buffer, "%d,%d,%d\n", sDate.Year + 1980, sDate.Month, sDate.Date );
						else
							sprintf( cmd_buffer, "%d,%d,%d\n", sTime.Hours, sTime.Minutes, sTime.Seconds );
						return cmd_buffer;
					}

					if( num_parm == 3 )
					{
						uint16_t i, val[3];

						for( i = 0; i < 3; ++i )
							val[i] = atoi( parameter[i] );

						if( keyword[idx] == SCPI_DATE )		// year,month,day
						{
							sDate.Year = val[0] - ( ( val[0] >= 2000 ) ? 2000 : 0 );
							sDate.Month = val[1];
							sDate.Date = val[2];
							sDate.WeekDay = RTC_WEEKDAY_MONDAY;
						}
						else								// hour, minute, second
						{
							sTime.Hours = val[0];
							sTime.Minutes = val[1];
							sTime.Seconds = val[2];
						}

						HAL_RTC_SetTime( &hrtc, &sTime, RTC_FORMAT_BIN );
						HAL_RTC_SetDate( &hrtc, &sDate, RTC_FORMAT_BIN );
					}
				}
				break;
			}
			break;

		case SCPI_CAL:
			switch( keyword[idx] )
			{
			case SCPI_SEC:	break;		// no password needed

			case SCPI_VAL:
				if( num_parm == 0 )
					break;

				parse_value( parameter[0], &val, unit, sizeof(unit) );
				s = val;

				if( s > 1 )
				{
					if( num_parm < 2 ) break;
					parse_value( parameter[1], &val, unit, sizeof(unit) );
				}

				switch( s )
				{
				case 0:	CALIB_CalibOnZero( NULL ); break;				// zero
				case 1:	CALIB_CalibOnPositive( val, NULL ); break;		// positive
				case 2: CALIB_CalibOnNegative( val, NULL ); break;		// negative (DC and TEMP only)
				}
				break;
			}
			break;

		}
	}

	return NULL;
}

void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart )
{
	if( huart != &huart1 )
		return;

	// a character has been received into cmd_buffer[buf_pos]

	if( buf_busy == 2 || buf_ready ) return;	// USB CDC is already receiving, or last command not yet processed

	buf_busy = 1;
	if( cmd_buffer[buf_pos] == '\n' )		// message complete
	{
		cmd_buffer[buf_pos] = 0;
		buf_ready = 1;
	}
	else
	{
		if( cmd_buffer[buf_pos] != '\r' )	// overwrite CR
			++buf_pos;

		// start receiving next character
		HAL_UART_Receive_IT( &huart1, (uint8_t*)&cmd_buffer[buf_pos], 1 );
	}
}

void SCPI_CDC_RxCallback( uint8_t *Buf, uint16_t Len )
{
	if( buf_busy == 2 || buf_ready ) return;	// RS232 is already receiving, or last command not yet processed

	buf_busy = 2;
	if( Len >= sizeof(cmd_buffer) - 1 )
		Len = sizeof(cmd_buffer) - 2;
	strncpy( cmd_buffer, (char*)Buf, Len );
	cmd_buffer[Len] = 0;
	buf_ready = 2;
}

void Do_SCPI( void )
{
	if( !buf_ready ) return;

	if( buf_pos > 0 )
	{
		char *response = SCPI_Execute( cmd_buffer );
		if( response )
		{
			if( buf_ready == 1 )	// command came from from RS232
			{
				scpi_printf( "%s\n", response );
			}
			else					// command came from USB
			{
				int l = snprintf( cmd_buffer, sizeof(cmd_buffer), "%s\n", response );
				CDC_Transmit_FS( (uint8_t*)response, l );
			}
		}
	}

	buf_ready = buf_busy = buf_pos = 0;							// invalidate buffer
	HAL_UART_Receive_IT( &huart1, (uint8_t*)cmd_buffer, 1 );	// trigger reception of first character
}
