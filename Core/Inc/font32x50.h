/*
  UTFT.cpp - Multi-Platform library support for Color TFT LCD Boards
  Copyright (C)2015 Rinky-Dink Electronics, Henning Karlsen. All right reserved

  This library is the continuation of my ITDB02_Graph, ITDB02_Graph16
  and RGB_GLCD libraries for Arduino and chipKit. As the number of
  supported display modules and controllers started to increase I felt
  it was time to make a single, universal library as it will be much
  easier to maintain in the future.

  Basic functionality of this library was origianlly based on the
  demo-code provided by ITead studio (for the ITDB02 modules) and
  NKC Electronics (for the RGB GLCD module/shield).

  This library supports a number of 8bit, 16bit and serial graphic
  displays, and will work with both Arduino, chipKit boards and select
  TI LaunchPads. For a full list of tested display modules and controllers,
  see the document UTFT_Supported_display_modules_&_controllers.pdf.

  When using 8bit and 16bit display modules there are some
  requirements you must adhere to. These requirements can be found
  in the document UTFT_Requirements.pdf.
  There are no special requirements when using serial displays.

  You can find the latest version of the library at
  http://www.RinkyDinkElectronics.com/

  This library is free software; you can redistribute it and/or
  modify it under the terms of the CC BY-NC-SA 3.0 license.
  Please see the included documents for further information.

  Commercial use of this library requires you to buy a license that
  will allow commercial use. This includes using the library,
  modified or not, as a tool to sell products.

  The license applies to all part of the library including the
  examples and tools supplied with the library.
*/

#ifndef __FONT32X50_H_
#define __FONT32X50_H_

// SevenSegNumFont.c
// Font Size	: 32x50
// Memory usage	: 2004 bytes
// # characters	: 10

const uint8_t font32x50Bitmaps[] =
{
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0xFF,0xFE,0x00,
	0x01,0xFF,0xFF,0x00,
	0x03,0xFF,0xFF,0x80,
	0x01,0xFF,0xFF,0x60,
	0x0C,0xFF,0xFE,0xF0,
	0x1E,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3E,0x00,0x00,0x78,
	0x38,0x00,0x00,0x18,
	0x20,0x00,0x00,0x08,
	0x00,0x00,0x00,0x00,
	0x20,0x00,0x00,0x00,
	0x38,0x00,0x00,0x18,
	0x3E,0x00,0x00,0x78,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x1E,0x00,0x00,0xF0,
	0x0C,0xFF,0xFE,0x60,
	0x01,0xFF,0xFF,0x00,
	0x03,0xFF,0xFF,0x80,
	0x01,0xFF,0xFF,0x00,
	0x00,0xFF,0xFE,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,  // 0

	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x00,0x00,0x00,0xF0,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x00,0x78,0x00,0x00,0x00,0x18,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x00,0x00,0x00,0x78,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x00,0xF0,0x00,0x00,0x00,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 1
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFE,0x00,0x01,0xFF,0xFF,0x00,0x03,0xFF,0xFF,0x80,0x01,0xFF,0xFF,0x60,0x00,0xFF,0xFE,0xF0,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x00,0x78,0x01,0xFF,0xFE,0x18,0x03,0xFF,0xFF,0x88,0x0F,0xFF,0xFF,0xE0,0x27,0xFF,0xFF,0xC0,0x39,0xFF,0xFF,0x00,0x3E,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x1E,0x00,0x00,0x00,0x0C,0xFF,0xFE,0x00,0x01,0xFF,0xFF,0x00,0x03,0xFF,0xFF,0x80,0x01,0xFF,0xFF,0x00,0x00,0xFF,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 2
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFE,0x00,0x01,0xFF,0xFF,0x00,0x03,0xFF,0xFF,0x80,0x01,0xFF,0xFF,0x60,0x00,0xFF,0xFE,0xF0,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x00,0x78,0x01,0xFF,0xFE,0x18,0x03,0xFF,0xFF,0x88,0x0F,0xFF,0xFF,0xE0,0x07,0xFF,0xFF,0xC0,0x01,0xFF,0xFF,0x18,0x00,0x00,0x00,0x78,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x00,0xF0,0x00,0xFF,0xFE,0x60,0x01,0xFF,0xFF,0x00,0x03,0xFF,0xFF,0x80,0x01,0xFF,0xFF,0x00,0x00,0xFF,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 3
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x0C,0x00,0x00,0xF0,0x1E,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3E,0x00,0x00,0x78,0x39,0xFF,0xFE,0x18,0x23,0xFF,0xFF,0x88,0x0F,0xFF,0xFF,0xE0,0x07,0xFF,0xFF,0xC0,0x01,0xFF,0xFF,0x18,0x00,0x00,0x00,0x78,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x00,0xF0,0x00,0x00,0x00,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 4
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFE,0x00,0x01,0xFF,0xFF,0x00,0x03,0xFF,0xFF,0x80,0x01,0xFF,0xFF,0x00,0x0C,0xFF,0xFE,0x00,0x1E,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3E,0x00,0x00,0x00,0x39,0xFF,0xFE,0x00,0x23,0xFF,0xFF,0x80,0x0F,0xFF,0xFF,0xE0,0x07,0xFF,0xFF,0xC0,0x01,0xFF,0xFF,0x18,0x00,0x00,0x00,0x78,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x00,0xF0,0x00,0xFF,0xFE,0x60,0x01,0xFF,0xFF,0x00,0x03,0xFF,0xFF,0x80,0x01,0xFF,0xFF,0x00,0x00,0xFF,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 5
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFE,0x00,0x01,0xFF,0xFF,0x00,0x03,0xFF,0xFF,0x80,0x01,0xFF,0xFF,0x00,0x0C,0xFF,0xFE,0x00,0x1E,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x3E,0x00,0x00,0x00,0x39,0xFF,0xFE,0x00,0x23,0xFF,0xFF,0x80,0x0F,0xFF,0xFF,0xE0,0x27,0xFF,0xFF,0xC0,0x39,0xFF,0xFF,0x18,0x3E,0x00,0x00,0x78,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x1E,0x00,0x00,0xF0,0x0C,0xFF,0xFE,0x60,0x01,0xFF,0xFF,0x00,0x03,0xFF,0xFF,0x80,0x01,0xFF,0xFF,0x00,0x00,0xFF,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 6
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFE,0x00,0x01,0xFF,0xFF,0x00,0x03,0xFF,0xFF,0x80,0x01,0xFF,0xFF,0x60,0x00,0xFF,0xFE,0xF0,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x00,0x78,0x00,0x00,0x00,0x18,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x00,0x00,0x00,0x78,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x00,0xF0,0x00,0x00,0x00,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 7

	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0xFF,0xFE,0x00,
	0x01,0xFF,0xFF,0x00,
	0x03,0xFF,0xFF,0x80,
	0x01,0xFF,0xFF,0x60,
	0x0C,0xFF,0xFE,0xF0,
	0x1E,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3E,0x00,0x00,0x78,
	0x39,0xFF,0xFE,0x18,
	0x23,0xFF,0xFF,0x88,
	0x0F,0xFF,0xFF,0xE0,
	0x27,0xFF,0xFF,0xC0,
	0x39,0xFF,0xFF,0x18,
	0x3E,0x00,0x00,0x78,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x3F,0x00,0x01,0xF8,
	0x1E,0x00,0x00,0xF0,
	0x0C,0xFF,0xFE,0x60,
	0x01,0xFF,0xFF,0x00,
	0x03,0xFF,0xFF,0x80,
	0x01,0xFF,0xFF,0x00,
	0x00,0xFF,0xFE,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,  // 8

	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFE,0x00,0x01,0xFF,0xFF,0x00,0x03,0xFF,0xFF,0x80,0x01,0xFF,0xFF,0x60,0x0C,0xFF,0xFE,0xF0,0x1E,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3F,0x00,0x01,0xF8,0x3E,0x00,0x00,0x78,0x39,0xFF,0xFE,0x18,0x23,0xFF,0xFF,0x88,0x0F,0xFF,0xFF,0xE0,0x07,0xFF,0xFF,0xC0,0x01,0xFF,0xFF,0x18,0x00,0x00,0x00,0x78,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x01,0xF8,0x00,0x00,0x00,0xF0,0x00,0xFF,0xFE,0x60,0x01,0xFF,0xFF,0x00,0x03,0xFF,0xFF,0x80,0x01,0xFF,0xFF,0x00,0x00,0xFF,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 9

	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x0F,0xF0,0x00,
	0x00,0x0F,0xF0,0x00,
	0x00,0x0F,0xF0,0x00,
	0x00,0x0F,0xF0,0x00,
	0x00,0x0F,0xF0,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,  // .

	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x01,0x00,0x00,
	0x00,0x03,0x80,0x00,
	0x00,0x07,0xC0,0x00,
	0x00,0x07,0xC0,0x00,
	0x00,0x07,0xC0,0x00,
	0x00,0x07,0xC0,0x00,
	0x00,0x07,0xC0,0x00,
	0x00,0x07,0xC0,0x00,
	0x00,0x07,0xC0,0x00,
	0x01,0xFF,0xFF,0x00,
	0x07,0xFF,0xFF,0xC0,
	0x0F,0xFF,0xFF,0xE0,
	0x07,0xFF,0xFF,0xC0,
	0x01,0xFF,0xFF,0x00,
	0x00,0x07,0xC0,0x00,
	0x00,0x07,0xC0,0x00,
	0x00,0x07,0xC0,0x00,
	0x00,0x07,0xC0,0x00,
	0x00,0x07,0xC0,0x00,
	0x00,0x07,0xC0,0x00,
	0x00,0x07,0xC0,0x00,
	0x00,0x03,0x80,0x00,
	0x00,0x01,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,  // +

	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x01,0xFF,0xFF,0x00,
	0x07,0xFF,0xFF,0xC0,
	0x0F,0xFF,0xFF,0xE0,
	0x07,0xFF,0xFF,0xC0,
	0x01,0xFF,0xFF,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,  // -
};

const GFXglyph font32x50Gyphs[] = {
	{   0,  0,  0, 34, 0,   0},  // 0x20 ' '
	{   0,  0,  0, 34, 0,   0},  // 0x21 '!'
	{   0,  0,  0, 34, 0,   0},  // 0x22 '"'
	{   0,  0,  0, 34, 0,   0},  // 0x23 '#'
	{   0,  0,  0, 34, 0,   0},  // 0x24 '$'
	{   0,  0,  0, 34, 0,   0},  // 0x25 '%'
	{   0,  0,  0, 34, 0,   0},  // 0x26 '&'
	{   0,  0,  0, 34, 0,   0},  // 0x27 '''
	{   0,  0,  0, 34, 0,   0},  // 0x28 '('
	{   0,  0,  0, 34, 0,   0},  // 0x29 ')'
	{   0,  0,  0, 34, 0,   0},  // 0x2A '*'
	{2200, 32, 50, 34, 0, -49},  // 0x2B '+'
	{   0,  0,  0, 10, 0,   0},  // 0x2C ','
	{2400, 32, 50, 34, 0, -49},  // 0x2D '-'
	{2000, 32, 50, 34, 0, -49},  // 0x2E '.'
	{   0,  0,  0, 34, 0, -49},  // 0x2F '/'
	{   0, 32, 50, 34, 0, -49},  // 0x30 '0'
	{ 200, 32, 50, 34, 0, -49},  // 0x31 '1'
	{ 400, 32, 50, 34, 0, -49},  // 0x32 '2'
	{ 600, 32, 50, 34, 0, -49},  // 0x33 '3'
	{ 800, 32, 50, 34, 0, -49},  // 0x34 '4'
	{1000, 32, 50, 34, 0, -49},  // 0x35 '5'
	{1200, 32, 50, 34, 0, -49},  // 0x36 '6'
	{1400, 32, 50, 34, 0, -49},  // 0x37 '7'
	{1600, 32, 50, 34, 0, -49},  // 0x38 '8'
	{1800, 32, 50, 34, 0, -49}   // 0x39 '9'
};

const GFXfont font32x50 = {
	font32x50Bitmaps,
	font32x50Gyphs,
	0x20, 0x39,
	32, 50,
	0, 52
};

#endif /* __INC_FONT32X50_H_ */
