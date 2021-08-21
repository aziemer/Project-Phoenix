/*
 * gfxfont.h
 *
 *  Created on: 18.08.2021
 *      Author: aziemer
 *
 */

/*
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

#ifndef CORE_INC_GFXFONT_H_
#define CORE_INC_GFXFONT_H_

#include <stdint.h>

/// Font data stored PER GLYPH
typedef struct {
	uint16_t bitmapOffset;	// Offset into GFXfont->bitmap
	uint8_t width;			// Bitmap dimensions in pixels
	uint8_t height;			// Bitmap dimensions in pixels
	uint8_t xAdvance;		// Distance to advance cursor (x axis)
	int8_t xOffset;			// X dist from cursor pos to UL corner
	int8_t yOffset;			// Y dist from cursor pos to UL corner
} GFXglyph;

/// Data stored for FONT AS A WHOLE
typedef struct {
	const uint8_t *bitmap;	// Glyph bitmaps, concatenated
	const GFXglyph *glyph;	// Glyph array
	uint8_t first;			// ASCII extents (first char)
	uint8_t last;			// ASCII extents (last char)
	uint8_t fontWidth;		// old style font -> no glyph array
	uint8_t fontHeight;		// highest char + undercut
	uint8_t xAdvance;		// old style font -> no glyph array
	uint8_t yAdvance;		// Newline distance (y axis)
} GFXfont;

#define PROGMEM

#endif /* CORE_INC_GFXFONT_H_ */
