/*
 * application.h
 *
 *  Created on: 14.08.2021
 *      Author: aziemer
 */

#ifndef CORE_INC_APPLICATION_H_
#define CORE_INC_APPLICATION_H_

#define	SPACING					5		// hor. and vert.

#define PREC					8

#define BACKGROUND_COLOR		RGB(0,0,0)
#define WARNING_COLOR			RGB(99,33,33)

#define ERROR_FONT				FONT_16X24
#define ERROR_FONT_SIZE			2

#define INDICATOR_ON_COLOR		RGB(0,99,0)
#define INDICATOR_OFF_COLOR		RGB(33,33,33)
#define INDICATOR_FONT			FONT_10X16

#define HEADER_WIDTH			( TFT_WIDTH - 1 )
#define HEADER_HEIGHT			29
#define HEADER_FONT				FONT_16X24
#define HEADER_COLOR			RGB(66,66,66)
#define HEADER_TEXT_COLOR		RGB(0,0,0)

#define BUTTON_WIDTH			100
#define	BUTTON_HEIGHT			( ( TFT_HEIGHT - HEADER_HEIGHT - 5 * SPACING ) / 5 - 1 )
#define	BUTTON_XPOS				( TFT_WIDTH - BUTTON_WIDTH - 1 )
#define	BUTTON_YPOS(n)			( HEADER_HEIGHT + SPACING + (n) * ( BUTTON_HEIGHT + SPACING ) )
#define BUTTON_FONT1			FONT_16X24
#define BUTTON_FONT2			FONT_10X16
#define BUTTON_COLOR			RGB(66,66,66)
#define BUTTON_VALUE_COLOR		RGB(90,90,90)
#define BUTTON_TEXT_COLOR		RGB(0,0,0)

#define FOOTER_FONT				FONT_16X24
#define FOOTER_COLOR			RGB(66,66,66)
#define FOOTER_TEXT_COLOR		RGB(0,0,0)

// 1st measurement (main maesurement)
#define VALUE1_FONT				FONT_32X50
#define VALUE1_XPOS				10
#define VALUE1_YPOS				140
#define VALUE1_COLOR			RGB(99,66,0)

#define UNIT1_FONT				FONT_16X24
#define UNIT1_XPOS				(VALUE1_XPOS+250)
#define UNIT1_YPOS				(VALUE1_YPOS-2)
#define UNIT1_COLOR				RGB(99,66,0)

// 2nd measurement (ACV/ACA/FREQ -> Duty)
#define VALUE2_FONT				FONT_16X24
#define VALUE2_XPOS				VALUE1_XPOS
#define VALUE2_YPOS				190
#define VALUE2_COLOR			RGB(99,66,0)

#define UNIT2_FONT				FONT_10X16
#define UNIT2_XPOS				(VALUE2_XPOS+50)
#define UNIT2_YPOS				VALUE2_YPOS
#define UNIT2_COLOR				RGB(99,66,0)

// 3rd measurement (AC V/I -> Freq, FREQ -> V)
#define VALUE3_FONT				FONT_16X24
#define VALUE3_XPOS				VALUE1_XPOS
#define VALUE3_YPOS				220
#define VALUE3_COLOR			RGB(99,66,0)

#define UNIT3_FONT				FONT_10X16
#define UNIT3_XPOS				(VALUE3_XPOS+50)
#define UNIT3_YPOS				VALUE3_YPOS
#define UNIT3_COLOR				RGB(99,66,0)

// TIME
#define TIME_FONT				FONT_10X16
#define TIME_COLOR				RGB(0,0,0)

#define HOLD_XPOS				10
#define HOLD_YPOS				( HEADER_HEIGHT + 20 )

#define AUTO_YPOS				( HEADER_HEIGHT + 20 )

#define START_REPEAT			50

void DrawDot( uint16_t color );

void SetHold( int mode );
void SetAuto( int mode );
void SetScale( uint8_t channel, int scale );

void DrawFooter( char *msg, ... );

void Application( void );

#endif /* CORE_INC_APPLICATION_H_ */
