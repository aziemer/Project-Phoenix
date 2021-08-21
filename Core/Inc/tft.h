/*
 * tft.h
 *
 *  Created on: 06.08.2021
 *      Author: aziemer
 */

#ifndef CORE_INC_TFT_H_
#define CORE_INC_TFT_H_

// Dimensions after rotation !
#define TFT_WIDTH	480
#define TFT_HEIGHT	320

#define VGA_BLACK		0x0000
#define VGA_WHITE		0xFFFF
#define VGA_RED			0xF800
#define VGA_GREEN		0x0400
#define VGA_BLUE		0x001F
#define VGA_SILVER		0xC618
#define VGA_GRAY		0x8410
#define VGA_MAROON		0x8000
#define VGA_YELLOW		0xFFE0
#define VGA_OLIVE		0x8400
#define VGA_LIME		0x07E0
#define VGA_AQUA		0x07FF
#define VGA_TEAL		0x0410
#define VGA_NAVY		0x0010
#define VGA_FUCHSIA		0xF81F
#define VGA_PURPLE		0x8010

#define VGA_TRANSPARENT	0xFFFFFFFF

#define LEFT			0
#define RIGHT			9999
#define CENTER			9998

#define FONT_10X16	1
#define FONT_16X24	2
#define FONT_32X50	3

#define RGB(r,g,b)	(((uint16_t)b*0x1F)/100) | ((((uint16_t)g*0x3F)/100)<<5) | ((((uint16_t)r*0x1F)/100)<<11)

void TFT_setBacklight( uint8_t on );
void TFT_setState( uint8_t on );
void TFT_setBackGround( uint16_t color );
void TFT_setForeGround( uint16_t color );
void TFT_setFont( uint8_t no );
uint8_t TFT_getFontHeight( void );

void TFT_setXPos( uint16_t x );
void TFT_setYPos( uint16_t y );
uint16_t TFT_getXPos( void );
uint16_t TFT_getYPos( void );

void TFT_drawLine( int x1, int y1, int x2, int y2 );
void TFT_drawRect( int x1, int y1, int x2, int y2 );
void TFT_fillRect( int x1, int y1, int x2, int y2 );
void TFT_drawRoundRect( int x1, int y1, int x2, int y2 );
void TFT_fillRoundRect( int x1, int y1, int x2, int y2 );

//void TFT_drawButtonFrame( int x1, int y1, int x2, int y2 );

void TFT_drawCircle( int x, int y, int radius );
void TFT_fillCircle( int x, int y, int radius );

size_t TFT_getStrWidth( char *txt );
int TFT_printf( const char *fmt, ... );

void TFT_Init( void );

// UTFT borrowed functions


#endif /* CORE_INC_TFT_H_ */
