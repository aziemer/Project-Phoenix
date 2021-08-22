/*
 * tft.c
 *
 *  Created on: 06.08.2021
 *      Author: aziemer
 *
 *  Based partially on code from the magnificient GFX library from Adafruit
 *  See license below
 */

/*
This is the core graphics library for all our displays, providing a common
set of graphics primitives (points, lines, circles, etc.).  It needs to be
paired with a hardware-specific library for each display device we carry
(to handle the lower-level functions).

Adafruit invests time and resources providing this open source code, please
support Adafruit & open-source hardware by purchasing products from Adafruit!

Copyright (c) 2013 Adafruit Industries.  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
 */


#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

#include "main.h"
#include "gpio.h"
#include "tft.h"
#include "gfxfont.h"

#include "font32x50.h"
#include "FreeSansBold9pt7b.h"
#include "FreeSansBold12pt7b.h"

#if (BOOTLOADER==0)
const uint16_t tft_init_data[] =		// Command bytes have 0x100 added
{
	// NOP
	0x100,

	0x1F0, 0x5A, 0x5A,
	0x1F1, 0x5A, 0x5A,
	0x1FC, 0xA5, 0xA5,
	0x1F3, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
	0x1F4, 0x02, 0xB6, 0x52, 0x52, 0x52, 0x52, 0x50, 0x32, 0x13, 0x54, 0x51, 0x11, 0x2A, 0x2A, 0xB3,

	// SLEEP OUT
	0x111,

	// DELAY
	0x200 + 1000,

	0x1F2, 0x3C, 0x7E, 0x03, 0x08, 0x08, 0x02, 0x10, 0x00, 0x2F, 0x10, 0xC8, 0x5D, 0x5D,
	0x1F6, 0x29, 0x02, 0x0F, 0x00, 0x14, 0x44, 0x11, 0x15,
	0x1F8, 0x33, 0x00, 0x19, 0x21, 0x40, 0x00, 0x09, 0x0A,
	0x1F7, 0x10, 0x80, 0x10, 0x02,

	// GAMMA ADJUST ???
	0x1F9, 0x04,
	0x1FA, 0x16, 0x2D, 0x09, 0x12, 0x1F, 0x00, 0x00, 0x02, 0x34, 0x32, 0x2A, 0x10, 0x00, 0x00,
	0x1FB, 0x16, 0x2D, 0x09, 0x12, 0x1F, 0x00, 0x00, 0x02, 0x34, 0x32, 0x2A, 0x10, 0x00, 0x00,

	0x1F9, 0x02,
	0x1FA, 0x04, 0x2C, 0x08, 0x1E, 0x30, 0x1D, 0x17, 0x13, 0x24, 0x26, 0x20, 0x06, 0x00, 0x00,
	0x1FB, 0x04, 0x2C, 0x08, 0x1E, 0x30, 0x1D, 0x17, 0x17, 0x24, 0x26, 0x20, 0x06, 0x00, 0x00,

	0x1F9, 0x01,
	0x1FA, 0x22, 0x2E, 0x0A, 0x15, 0x20, 0x04, 0x07, 0x02, 0x38, 0x33, 0x25, 0x00, 0x00, 0x00,
	0x1FB, 0x22, 0x2D, 0x0A, 0x15, 0x20, 0x04, 0x06, 0x02, 0x38, 0x35, 0x25, 0x00, 0x00, 0x00,

	// MEMORY ACCESS CONTROL
	0x136, 0b00101000,				// Bits: MY, MX, MV, ML, BGR, MH, x, x

	// INTERFACE PIXEL FORMAT
	0x13A, 0b01010101,				// Bits: 0 DPI[6:4] 0 DBI[2:0] -> 16 bits/pixel

	// COLUMN ADDRESS SET
	0x12A, 0x00, 0xF0, 0x01, 0x3F,	// Bytes: StartColumnH, StartColumnL, EndColumnH, EndColumnL

	// PAGE ADDRESS SET (row)
	0x12B, 0x00, 0x00, 0x01, 0xDF,	// Bytes: StartRowH, StartRowL, EndRowH, EndRowL

	0x1ED, 0x08,					// ???

	// WRITE CONTROL DISPLAY VALUE
	0x153, 0b00100100,				// Bits: 0 0 BCTRL 0 DD BL 0 0

	// WRITE DISPLAY BRIGHTNESS VALUE
	0x151, 0xFF,					// DBV[7:0]

	// DISPLAY ON
	0x129,

	0xFFFF							// end marker
};
#endif

/* screen coordinates:
 *
 * 0,0					479,0
 * +------------------------+
 * | +----> +X				|
 * | |						|
 * | |						|
 * | |						|
 * | v						|
 * | +Y						|
 * |						|
 * +------------------------+
 * 0,319			  479,319
 */

static uint16_t tft_bgcolor = RGB(0,0,0);
static uint16_t tft_fgcolor = RGB(100,100,100);
static uint16_t tft_xpos = 0;
static uint16_t tft_ypos = 0;
static int8_t tft_font_topy;			// tallest character above base-line (negative Y!)
static int8_t tft_font_bottomy = 0;		// tallest under-cut below base-line (positive Y!)
static uint16_t tft_fontsize = 1;

static const GFXfont *gfxFont = NULL;

static inline void transfer( uint8_t val )
{
	HAL_GPIO_WritePin( LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_RESET );	// low
	GPIOA->BSRR = ( val & 0xFF ) | ( (uint32_t)( ~val & 0xFF ) << 16 );	// set DATA byte
	asm volatile ("nop");
	HAL_GPIO_WritePin( LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_SET );	// high
}

static inline void startWrite( void )
{
	HAL_GPIO_WritePin( LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET );	// low
}

static inline void endWrite( void )
{
	HAL_GPIO_WritePin( LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET );	// high
}

static void lcdWriteData16Repeat( uint16_t data, uint32_t count )
{
	HAL_GPIO_WritePin( LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET );	// low
	transfer( 0x2C );

	HAL_GPIO_WritePin( LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET );	// high
	while( count-- )
	{
		transfer( data >> 8 );
		transfer( data & 0xFF );
	}
}

static void setXY( uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2 )
{
	HAL_GPIO_WritePin( LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET );	// low
	transfer( 0x2A );

	HAL_GPIO_WritePin( LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET );	// high
	transfer( (uint8_t)( x1 >> 8 ) );
	transfer( x1 & 0xFF );
	transfer( (uint8_t)( x2 >> 8 ) );
	transfer( x2 & 0xFF );

	HAL_GPIO_WritePin( LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET );	// low
	transfer( 0x2B );

	HAL_GPIO_WritePin( LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET );	// high
	transfer( (uint8_t)( y1 >> 8 ) );
	transfer( y1 & 0xFF );
	transfer( (uint8_t)( y2 >> 8 ) );
	transfer( y2 & 0xFF );
}

void TFT_setXPos( uint16_t x )
{
	tft_xpos = x;
}

void TFT_setYPos( uint16_t y )
{
	tft_ypos = y;
}

uint16_t TFT_getXPos( void )
{
	return tft_xpos;
}

uint16_t TFT_getYPos( void )
{
	return tft_ypos;
}

static void TFT_setRegister( uint8_t reg, uint8_t num, uint8_t *param )
{
	startWrite();

	HAL_GPIO_WritePin( LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET );	// low
	transfer( reg );

	HAL_GPIO_WritePin( LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET );	// high
	while( num-- ) transfer( *param++ );

	endWrite();
}

void TFT_setBacklight( uint8_t on )
{
	HAL_GPIO_WritePin( BACKLIGHT_GPIO_Port, BACKLIGHT_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET );
}

void TFT_setState( uint8_t on )
{
	TFT_setRegister( on ? 0x29 : 0x28, 0, NULL );
}

void TFT_setBackGround( uint16_t color )
{
	tft_bgcolor = color;
}

void TFT_setForeGround( uint16_t color )
{
	tft_fgcolor = color;
}

static void lcd_write_xy( uint16_t x1, uint16_t y1,  uint16_t x2, uint16_t y2, uint16_t color )
{
	setXY( x1, y1, x2, y2 );
	transfer( color >> 8 );
	transfer( color & 0xFF );
}

static void drawPixel( int x, int y )
{
	startWrite();
	lcd_write_xy( x, y, x, y, tft_fgcolor );
	endWrite();
}

static void drawHLine( int x, int y, int l )
{
	if( l < 0 )
	{
		l = -l;
		x -= l;
	}

	startWrite();
	setXY( x, y, x + l, y );
	lcdWriteData16Repeat( tft_fgcolor, l );
	endWrite();
}

static void drawVLine( int x, int y, int l )
{
	if( l < 0 )
	{
		l = -l;
		y -= l;
	}

	startWrite();
	setXY( x, y, x, y + l );
	lcdWriteData16Repeat( tft_fgcolor, l );
	endWrite();
}

void TFT_drawLine( int x1, int y1, int x2, int y2 )
{
	if( y1 == y2 )
		drawHLine( x1, y1, x2 - x1 );
	else if( x1 == x2 )
		drawVLine( x1, y1, y2 - y1 );
	else
	{
		unsigned int dx = ( x2 > x1 ? x2 - x1 : x1 - x2 );
		short xstep = x2 > x1 ? 1 : -1;
		unsigned int dy = ( y2 > y1 ? y2 - y1 : y1 - y2 );
		short ystep = y2 > y1 ? 1 : -1;
		int col = x1, row = y1;

		startWrite();
		if( dx < dy )
		{
			int t = -( dy >> 1 );
			while( 1 )
			{
				lcd_write_xy( col, row, col, row, tft_fgcolor );
				if( row == y2 )
					return;
				row += ystep;
				t += dx;
				if( t >= 0 )
				{
					col += xstep;
					t -= dy;
				}
			}
		}
		else
		{
			int t = -( dx >> 1 );
			while( 1 )
			{
				lcd_write_xy( col, row, col, row, tft_fgcolor );
				if( col == x2 )
					return;
				col += xstep;
				t += dy;
				if( t >= 0 )
				{
					row += ystep;
					t -= dx;
				}
			}
		}
		endWrite();
	}
}

void TFT_drawRect( int x1, int y1, int x2, int y2 )
{
	drawHLine( x1, y1, x2 - x1 );
	drawHLine( x1, y2, x2 - x1 );
	drawVLine( x1, y1, y2 - y1 );
	drawVLine( x2, y1, y2 - y1 );
}

void TFT_fillRect( int x1, int y1, int x2, int y2 )
{
	startWrite();
	setXY( x1, y1, x2, y2 );
	lcdWriteData16Repeat( tft_fgcolor, (long)( ( x2 - x1 ) + 1 ) * (long)( ( y2 - y1 ) + 1 ) );
	endWrite();
}

void TFT_drawRoundRect( int x1, int y1, int x2, int y2 )
{
	if( ( x2 - x1 ) > 4 && ( y2 - y1 ) > 4 )
	{
		drawPixel( x1 + 1, y1 + 1 );
		drawPixel( x2 - 1, y1 + 1 );
		drawPixel( x1 + 1, y2 - 1 );
		drawPixel( x2 - 1, y2 - 1 );
		drawHLine( x1 + 2, y1, x2 - x1 - 4 );
		drawHLine( x1 + 2, y2, x2 - x1 - 4 );
		drawVLine( x1, y1 + 2, y2 - y1 - 4 );
		drawVLine( x2, y1 + 2, y2 - y1 - 4 );
	}
}

void TFT_fillRoundRect( int x1, int y1, int x2, int y2 )
{
	int i;

	if( ( x2 - x1 ) > 4 && ( y2 - y1 ) > 4 )
	{
		for( i = 0; i < ( ( y2 - y1 ) / 2 ) + 1; ++i )
		{
			switch( i )
			{
			case 0:
				drawHLine( x1 + 2, y1 + i, x2 - x1 - 4 );
				drawHLine( x1 + 2, y2 - i, x2 - x1 - 4 );
				break;
			case 1:
				drawHLine( x1 + 1, y1 + i, x2 - x1 - 2 );
				drawHLine( x1 + 1, y2 - i, x2 - x1 - 2 );
				break;
			default:
				drawHLine( x1, y1 + i, x2 - x1 );
				drawHLine( x1, y2 - i, x2 - x1 );
			}
		}
	}
}

void TFT_drawCircle( int x, int y, int radius )
{
	int f = 1 - radius;
	int ddF_x = 1;
	int ddF_y = -2 * radius;
	int x1 = 0;
	int y1 = radius;

	startWrite();

	lcd_write_xy( x, y + radius, x, y + radius, tft_fgcolor );
	lcd_write_xy( x, y - radius, x, y - radius, tft_fgcolor );
	lcd_write_xy( x + radius, y, x + radius, y, tft_fgcolor );
	lcd_write_xy( x - radius, y, x - radius, y, tft_fgcolor );

	while( x1 < y1 )
	{
		if( f >= 0 )
		{
			y1--;
			ddF_y += 2;
			f += ddF_y;
		}
		x1++;
		ddF_x += 2;
		f += ddF_x;

		lcd_write_xy( x + x1, y + y1, x + x1, y + y1, tft_fgcolor );
		lcd_write_xy( x - x1, y + y1, x - x1, y + y1, tft_fgcolor );
		lcd_write_xy( x + x1, y - y1, x + x1, y - y1, tft_fgcolor );
		lcd_write_xy( x - x1, y - y1, x - x1, y - y1, tft_fgcolor );
		lcd_write_xy( x + y1, y + x1, x + y1, y + x1, tft_fgcolor );
		lcd_write_xy( x - y1, y + x1, x - y1, y + x1, tft_fgcolor );
		lcd_write_xy( x + y1, y - x1, x + y1, y - x1, tft_fgcolor );
		lcd_write_xy( x - y1, y - x1, x - y1, y - x1, tft_fgcolor );
	}

	endWrite();
}

void TFT_fillCircle( int x, int y, int radius )
{
	int x1, y1;

	for( y1 = -radius; y1 <= 0; y1++ )
	{
		for( x1 = -radius; x1 <= 0; x1++ )
		{
			if( x1 * x1 + y1 * y1 <= radius * radius )
			{
				drawHLine( x + x1, y + y1, -2 * x1 );
				drawHLine( x + x1, y - y1, -2 * x1 );
				break;
			}
		}
	}
}

#if 0
static void drawHLine1( int x, int y, int length, uint16_t color )
{
	tft_fgcolor = color;
	setXY( x, y, x + length, y );
	lcdWriteData16Repeat( tft_fgcolor, length );
}

static void drawVLine1( int x, int y, int width, uint16_t color )
{
	tft_fgcolor = color;
	setXY( x, y, x, y + width );
	lcdWriteData16Repeat( tft_fgcolor, width );
}

static uint16_t mkcolor( uint16_t c )
{
	return ( c << 11 ) | ( c << 6 ) | c;
}

void drawButtonFrame( int x1, int y1, int x2, int y2 )
{
	int xx;
	uint8_t colors[25] =
	{
		 0,  0, 12, 18, 18,
		 0, 20, 26, 22, 20,
	    12, 26, 31, 28, 24,
		18, 22, 28, 28, 22,
		18, 20, 24, 22, 20
	};

	startWrite();

	setXY( x1, y1, x1 + 4, y1 + 4 );

	HAL_GPIO_WritePin( LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET );	// low
	transfer( 0x2C );

	for( xx = 0; xx < 25; ++xx )
	{
		uint16_t c = mkcolor( colors[xx] );
		transfer( c >> 8 );
		transfer( c & 0xFF );
	}

	drawHLine1( x1 + 5, y1 + 0, x2 - x1 - 6, mkcolor( colors[4] ) );
	drawHLine1( x1 + 3, y1 + 1, x2 - x1 - 3, mkcolor( colors[9] ) );
	drawHLine1( x1 + 2, y1 + 2, x2 - x1 - 2, mkcolor( colors[14] ) );
	drawHLine1( x1 + 1, y1 + 3, x2 - x1 - 1, mkcolor( colors[19] ) );
	drawHLine1( x1 + 1, y1 + 4, x2 - x1 - 2, mkcolor( colors[24] ) );


	drawVLine1( x1 + 0, y1 + 5, y2 - y1 - 5, mkcolor( colors[4] ) );
	drawVLine1( x1 + 1, y1 + 5, y2 - y1 - 4, mkcolor( colors[9] ) );
	drawVLine1( x1 + 2, y1 + 5, y2 - y1 - 4, mkcolor( colors[14] ) );
	drawVLine1( x1 + 3, y1 + 5, y2 - y1 - 4, mkcolor( colors[19] ) );
	drawVLine1( x1 + 4, y1 + 5, y2 - y1 - 5, mkcolor( colors[24] ) );

	endWrite();
}
#endif

int TFT_printf( const char *fmt, ... )
{
	const uint8_t *bitmap = gfxFont->bitmap;
	int8_t xx, yy, x1, x2, ys, ye, xs, xe;
	uint8_t xAdvance, blksize = tft_fontsize * tft_fontsize, count, lines;
	uint8_t bits = 0, start_bits;
	uint8_t bit = 0, start_bit;
	uint16_t bo, start_bo, color;
	char txt[200], *ptr = txt;
	int len, ret;
	va_list ap;
	va_start( ap, fmt );

	*txt = 0;
	ret = vsnprintf( txt, sizeof(txt), fmt, ap );
	txt[sizeof(txt)-1] = 0;
	len = strlen( txt );
	va_end( ap );

	// tft_ypos = 0 is base line, y can go below. NEGATIVE y is ABOVE base
	// tft_xpos = 0 is character left side, x can go left of it. NEGATIVE x is LEFT of 0.

	while( len-- )
	{
		uint8_t c = *ptr++;

		if( c >= gfxFont->first && c <= gfxFont->last )
		{
			bits = bit = 0;

			if( gfxFont->glyph )
			{
				const GFXglyph *glyph = &gfxFont->glyph[c - gfxFont->first];
				bo = glyph->bitmapOffset;

				// area of bitmap data
				ys = glyph->yOffset;
				ye = ys + glyph->height;
				xs = glyph->xOffset;
				xe = xs + glyph->width;

				xAdvance = glyph->xAdvance;
			}
			else
			{
				bo = (uint16_t)( c - gfxFont->first ) * gfxFont->fontHeight * ( ( gfxFont->fontWidth + 7 ) / 8 );

				// area of bitmap data
				ys = -gfxFont->fontHeight;
				ye = 0;
				xs = 0;
				xe = gfxFont->fontWidth;

				xAdvance = gfxFont->fontWidth + 2;
			}

			// canvas area
			x1 = ( xs < 0 ) ? xs : 0;
			x2 = ( xe > xAdvance ) ? xe : xAdvance;

			startWrite();

			setXY(	tft_xpos + x1, tft_ypos + tft_fontsize * tft_font_topy,
					tft_xpos + tft_fontsize * x2 - 1, tft_ypos + tft_fontsize * tft_font_bottomy - 1 );

			HAL_GPIO_WritePin( LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET );	// low
			transfer( 0x2C );
			HAL_GPIO_WritePin( LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET );	// high

			for( yy = tft_font_topy; yy < tft_font_bottomy; ++yy )
			{
				start_bo = bo;
				start_bit = bit;
				start_bits = bits;

				for( lines = 0; lines < tft_fontsize; ++lines )		// repeat each line 'tft_fontsize' times
				{
					bit = start_bit;
					bits = start_bits;
					bo = start_bo;

					for( xx = x1; xx < x2; ++xx )					// output 'tft_fontsize' pixels per dot
					{
						color = tft_bgcolor;

						if( xx >= xs && xx < xe && yy >= ys && yy < ye )
						{
							if( !( bit++ & 7 ) ) bits = bitmap[ bo++ ];
							if( bits & 0x80 ) color = tft_fgcolor;
							bits <<= 1;
						}

						for( count = blksize; count; --count )
						{
							transfer( color >> 8 );
							transfer( color & 0xFF );
						}
					}
				}
			}

			endWrite();

			tft_xpos += tft_fontsize * xAdvance;
		}
	}

	return ret;
}

void TFT_setFontSize( uint8_t size )
{
	tft_fontsize = size;
}

uint8_t TFT_getFontHeight( void )
{
	return tft_fontsize * ( abs( tft_font_bottomy - tft_font_topy ) + 1 );	// topy < 0, bottomy >= 0
}

size_t TFT_getStrWidth( char *txt )
{
	size_t len = 0;
	uint8_t c;

	while( ( c = *txt++ ) > 0 )
	{
		if( gfxFont->glyph )
		{
			if( c >= gfxFont->first && c <= gfxFont->last )
			{
				len += gfxFont->glyph[c - gfxFont->first].xAdvance;
			}
		}
		else
			len += gfxFont->xAdvance;
	}

	return len * tft_fontsize;
}

void TFT_setFont( uint8_t no )
{
	switch( no )
	{
	default:
	case FONT_10X16:	gfxFont = &FreeSansBold9pt7b; break;
	case FONT_16X24:	gfxFont = &FreeSansBold12pt7b; break;
	case FONT_32X50:	gfxFont = &font32x50; break;
	}

	if( gfxFont->glyph )
	{
		uint8_t c;
		int16_t yy;

		tft_font_topy = 127;
		tft_font_bottomy = -127;

		for( c = gfxFont->first; c <= gfxFont->last; ++c )
		{
			const GFXglyph *glyph = &gfxFont->glyph[c - gfxFont->first];

			if( glyph->yOffset < tft_font_topy )
				tft_font_topy = glyph->yOffset;

			yy = glyph->yOffset + glyph->height;
			if( yy > tft_font_bottomy )
				tft_font_bottomy = yy;
		}
	}
	else
	{
		tft_font_topy = 1 - gfxFont->fontHeight;
		tft_font_bottomy = 0;
	}
	tft_fontsize = 1;
}

#if (BOOTLOADER==0)

static void delay_us( uint16_t us )
{
	while( us-- )
		asm volatile ("nop");
}

#endif

void TFT_Init( void )
{
	GPIO_InitTypeDef GPIO_InitStruct = { .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_FREQ_HIGH, .Mode = GPIO_MODE_OUTPUT_PP };

	/* Configure TFT data GPIOs as outputs */
	GPIO_InitStruct.Pin = 0xFF;
	HAL_GPIO_Init( LCD_D0_GPIO_Port, &GPIO_InitStruct );

	/* Configure TFT control GPIOs as outputs */
	GPIO_InitStruct.Pin = LCD_CS_Pin | LCD_RS_Pin | LCD_WR_Pin;
	HAL_GPIO_Init( LCD_RS_GPIO_Port, &GPIO_InitStruct );

#if (BOOTLOADER==0)

	delay_us( 60000U );

	uint16_t i = 0, val;

	startWrite();

	while( ( val = tft_init_data[i++] ) != 0xFFFF )	// END
	{
		if( val >= 0x200 )					// DELAY
		{
			delay_us( val -= 0x200 );
		}
		else if( val >= 0x100 )				// COMMAND
		{
			HAL_GPIO_WritePin( LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET );	// low
			transfer( val & 0xFF );
			delay_us( 50 );
		}
		else								// DATA
		{
			HAL_GPIO_WritePin( LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET );	// high
			transfer( val & 0xFF );
			delay_us( 50 );
		}
	}

	endWrite();

#endif
	TFT_setBacklight( 1 );
}
