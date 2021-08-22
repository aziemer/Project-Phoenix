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

#include "rtc.h"
#include "usart.h"
#include "main.h"
#include "gpio.h"
#include "tft.h"
#include "kbd.h"

#include "dmm.h"
#include "calib.h"
#include "application.h"

const char * const scpi_keywords[] = {
	"*IDN",
	"*RST",
	"SENSe",
	"CONFigure",
	"CALCulate",
	"SYSTem",
	"AUTO",
	"RANGe",
	"RATE",
	"MEAS",				// with ? only
	"FUNCtion",
	"VOLTage DC",
	"CURRent DC",
	"VOLTage AC",
	"CURRent AC",
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
	"SHOW"
	"THREshold",
	"SCALar"
	"AVERage",
	"ALL",
	"MAXimum",
	"MINimum",
	"DB",
	"DBM",
	"REFerence",
	"NULL",
	"OFFSet",
	"STATe",
	"BEEPer",
	"DATE",
	"TIME",
	"LOCal",
	"REMote",
	"DEFault"
};

typedef enum {		// must be same order as above !
	SCPI_DOT_IDN,
	SCPI_DOT_RST,
	SCPI_SENS,
	SCPI_CONF,
	SCPI_CALC,
	SCPI_SYST,
	SCPI_AUTO,
	SCPI_RANG,
	SCPI_RATE,
	SCPI_MEAS,
	SCPI_FUNC,
	SCPI_VOLT_DC,
	SCPI_CURR_DC,
	SCPI_VOLT_AC,
	SCPI_CURR_AC,
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
	SCPI_THRE,
	SCPI_SCAL,
	SCPI_AVER,
	SCPI_ALL,
	SCPI_MAX,
	SCPI_MIN,
	SCPI_DB,
	SCPI_DBM,
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

	SCPI_NUM_STRINGS
} SCPI_STRINGS;

static char cmd_buffer[100];
static uint8_t buf_pos = 0;
static uint8_t buf_ready = 1;

static int print( const char *fmt, ... )
{
	va_list ap;
	char txt[100];
	va_start( ap, fmt );

	int l = vsnprintf( txt, sizeof(txt)-1, fmt, ap );
	HAL_UART_Transmit( &huart1, (uint8_t*)txt, l, 1000 );

	va_end( ap );
	return l;
}

static char *SCPI_Short( uint8_t kw )
{
	static char response[10];

	if( kw >= SCPI_NUM_STRINGS )
		return "";

	const char *s = scpi_keywords[ kw ];
	int x = 0;
	while( *s && x < sizeof(response) - 1 )
	{
		if( isupper( (uint8_t)*s ) || *s == ' ' )
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
		const uint8_t *token;
		uint8_t *cmd = (uint8_t*)keyword;

		for( token = (const uint8_t*)scpi_keywords[i]; *token; ++token )
		{
			uint8_t ch = toupper( *cmd );

			if( *token == ' ' )						// space in SCPI string must match one or more whitespace(s) in command string
			{
				if( !*cmd || !isspace( *cmd ) )		// no, command has no space there
					break;

				while( *cmd && isspace( *cmd ) )	// yes, command had space(s), skip them
					++cmd;
			}
			else if( isupper( *token ) ||
					*token == '*' )					// upper case SCPI character or star *MUST* match command character, ignoring it's case
			{
				if( ch != *token )					// no, command does not have that character there
					break;
				++cmd;								// yes, command has the same character there, advance
			}
			else if( toupper( *token ) == ch )		// lower case SCPI character *MAY* match command character, ignoring it's case
			{
				++cmd;								// yes, that matches, advance
			}
			else if( !isalpha( ch ) )				// if it does not, it must either be a delimiter or a number
			{

			}
			else									// if it does not, it must match the next upper case character of the SCPI string
			{
				const uint8_t *p = token;
				while( *p && islower( *p ) )		// advance SCPI string to next upper character - if any
					++p;

				if( ch != *p )						// no match either
					break;
			}
		}

		if( !*token )								// bingo!
			return i;
	}

	return -1;
}

#define MAX_SCPI_PATH 5

char *SCPI_Execute( char *command_string )
{
	char *cmd, *p, *keyword, delimiter = 0;
	int kw_index[MAX_SCPI_PATH];
	int ch_index[MAX_SCPI_PATH];
	uint8_t num_kw;

	for( cmd = strtok( command_string, ";" ); cmd; cmd = strtok( NULL, ";" ) )
	{
		// traverse SCPI path
		for( num_kw = 0; num_kw < MAX_SCPI_PATH && *cmd; )
		{
			for( keyword = p = cmd; isalpha( (uint8_t)*p ) || *p == '*'; ++p )
				;

			if( isdigit( (uint8_t)*p ) )
			{
				ch_index[num_kw] = atoi( p );
				*p = 0;
				while( isdigit( (uint8_t)*++p ) )
					;
			}
			else
				ch_index[num_kw] = -1;

			delimiter = *p;
			if( *p ) *p++ = 0;
			cmd = p;

			kw_index[num_kw++] = SCPI_Match( keyword );

			if( delimiter != ':' )			// more to come ?
				break;
		}

		if( isspace( (uint8_t)delimiter ) )	// command with parameters
		{
			while( *cmd && isspace( (uint8_t)*cmd ) )
				++cmd;
		}
		else if( delimiter != '?' )			// not a query -> command without parameters
			*cmd = 0;

		switch( kw_index[0] )
		{
		// IEEE mandatory commands
		case SCPI_DOT_IDN:
			return "VoltCraft,VC-7055BT,2038274,V1.9.0";

		case SCPI_DOT_RST:
			DMM_Init();

			TFT_setForeGround( BACKGROUND_COLOR );
			TFT_fillRect( 0, 0, TFT_WIDTH-1, TFT_HEIGHT-1 );		// Clear screen

			SetScale( 1, SCALE_DC_1kV );
			SetAuto( 1 );
			return NULL;

		case SCPI_SENS:
			if( kw_index[1] == SCPI_FUNC )
			{
				if( ch_index[1] < 1 || ch_index[1] > 3 )
					ch_index[1] = 1;

				if( *cmd )		// parameters ?
				{
					int s = SCPI_Match( cmd );
					switch( s )
					{
					case SCPI_RES:		SetScale( ch_index[1], SCALE_500_Ohm ); break;
					case SCPI_FRES:		SetScale( ch_index[1], SCALE_4W_500_Ohm ); break;
					case SCPI_VOLT_DC:	SetScale( ch_index[1], SCALE_DC_50V ); break;
					case SCPI_VOLT_AC:	SetScale( ch_index[1], SCALE_AC_50V ); break;
					case SCPI_CURR_DC:	SetScale( ch_index[1], SCALE_DC_50mA ); break;
					case SCPI_CURR_AC:	SetScale( ch_index[1], SCALE_AC_50mA ); break;
					case SCPI_CAP:		SetScale( ch_index[1], SCALE_5_uF ); break;
					case SCPI_FREQ:		SetScale( ch_index[1], SCALE_FREQ ); break;
					case SCPI_TEMP:		SetScale( ch_index[1], SCALE_TEMP ); break;
					case SCPI_CONT:		SetScale( ch_index[1], SCALE_CONT ); break;
					case SCPI_DIOD:		SetScale( ch_index[1], SCALE_DIODE ); break;
					default:			print( "Bad FUNC\n" );
					}
				}

				int scale = DMM_GetScale( ch_index[1] );
				int mode = DMM_GetMode( scale );
				switch( mode )
				{
				case DmmResistance:		return SCPI_Short( SCPI_RES );
				case DmmResistance4W:	return SCPI_Short( SCPI_FRES );
				case DmmDCVoltage:		return SCPI_Short( SCPI_VOLT_DC );
				case DmmACVoltage:		return SCPI_Short( SCPI_VOLT_AC );
				case DmmDCCurrent:		return SCPI_Short( SCPI_CURR_DC );
				case DmmACCurrent:		return SCPI_Short( SCPI_CURR_AC );
				case DmmCapacitance:	return SCPI_Short( SCPI_CAP );
				case DmmFrequency:		return SCPI_Short( SCPI_FREQ );
				case DmmTemperature:	return SCPI_Short( SCPI_TEMP );
				case DmmContinuity:		return SCPI_Short( SCPI_CONT );
				case DmmDiode:			return SCPI_Short( SCPI_DIOD );
				default:				return "Error";
				}
			}
			break;
		}
	}
	return NULL;
}

// UART interrupt callback
void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart )
{
	if( huart != &huart1 )
		return;

	if( cmd_buffer[buf_pos] == '\n' )		// message complete
	{
		cmd_buffer[buf_pos] = 0;
		buf_ready = 1;
	}
	else
	{
		if( cmd_buffer[buf_pos] != '\r' )	// overwrite CR with next character
			++buf_pos;

		// start receiving next character
		HAL_UART_Receive_IT( &huart1, (uint8_t*)&cmd_buffer[buf_pos], 1 );
	}
}

void Do_SCPI( void )
{
	if( !buf_ready ) return;

	if( buf_pos > 0 )
	{
		char *response = SCPI_Execute( cmd_buffer );
		if( response ) print( "%s\n", response );
		cmd_buffer[ buf_pos = 0 ] = 0;
	}

	buf_pos = buf_ready = 0;
	HAL_UART_Receive_IT( &huart1, (uint8_t*)cmd_buffer, 1 );
}
