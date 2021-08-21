/*
 * kbd.h
 *
 *  Created on: 06.08.2021
 *      Author: aziemer
 */

#ifndef CORE_INC_KBD_H_
#define CORE_INC_KBD_H_

enum {
	KEY_NONE = 0,
	KEY_F4,		KEY_F3,		KEY_F2,		KEY_F1,		KEY_VOLT,
	KEY_F5,		KEY_LEFT,	KEY_MATH,	KEY_SAVE,	KEY_AMP,
	KEY_DUAL,	KEY_RANGE,	KEY_UTIL,	KEY_REC,	KEY_OHM,
	KEY_EXIT,	KEY_DOWN,	KEY_UP,		KEY_RUN,	KEY_CAP,
	KEY_TEMP,	KEY_SPARE,	KEY_RIGHT,	KEY_PORT,	KEY_FREQ
} KEYS;

void  KBD_Init( void );
uint8_t KBD_Read( void );

#endif /* CORE_INC_KBD_H_ */
