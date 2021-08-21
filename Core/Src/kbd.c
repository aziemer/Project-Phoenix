/*
 * kbd.c
 *
 *  Created on: 06.08.2021
 *      Author: aziemer
 */

#include "main.h"
#include "gpio.h"
#include "kbd.h"

static GPIO_TypeDef *drive_port[] = {	KBD_COL1_GPIO_Port,	KBD_COL2_GPIO_Port,	KBD_COL3_GPIO_Port,	KBD_COL4_GPIO_Port,	KBD_COL5_GPIO_Port };
static uint16_t drive_pin[] = {			KBD_COL1_Pin,		KBD_COL2_Pin,		KBD_COL3_Pin,		KBD_COL4_Pin,		KBD_COL5_Pin };

static GPIO_TypeDef *sense_port[] = {	KBD_ROW1_GPIO_Port,	KBD_ROW2_GPIO_Port,	KBD_ROW3_GPIO_Port,	KBD_ROW4_GPIO_Port,	KBD_ROW5_GPIO_Port };
static uint16_t sense_pin[] = {			KBD_ROW1_Pin,		KBD_ROW2_Pin,		KBD_ROW3_Pin,		KBD_ROW4_Pin,		KBD_ROW5_Pin };

/* Configure KBD drive and sense GPIOs */
void KBD_Init( void )
{
	GPIO_InitTypeDef GPIO_InitStruct = { .Speed = GPIO_SPEED_FREQ_LOW };
	uint8_t i;

	for( i = 0; i < 5; ++i )
	{
		/* Configure drive pins as open-drain outputs, and set high -> Hi-Z */
		GPIO_InitStruct.Pin = drive_pin[i];
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init( drive_port[i], &GPIO_InitStruct );

		HAL_GPIO_WritePin( drive_port[i], drive_pin[i], GPIO_PIN_SET );

		/* Configure sense pins as inputs with pull-ups, and set high -> pull up */
		GPIO_InitStruct.Pin = sense_pin[i];
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init( sense_port[i], &GPIO_InitStruct );

		HAL_GPIO_WritePin( sense_port[i], sense_pin[i], GPIO_PIN_SET );
	}
}

uint8_t KBD_Read( void )
{
	uint8_t key = 0, row, col;

	/* scan keyboard matrix */
	for( row = 4; row < 5 && !key; --row )		// drive PC13 first, as it seems to always be low (?)
	{
		HAL_GPIO_WritePin( drive_port[row], drive_pin[row], GPIO_PIN_RESET );	// drive pin low

		/* scan column pins for low value -> key pressed */
		for( col = 0; col < 5; ++col )
		{
			if( HAL_GPIO_ReadPin( sense_port[col], sense_pin[col] ) == GPIO_PIN_RESET )
			{
				key = 1 + 5 * col + row;
				break;
			}
		}

		HAL_GPIO_WritePin( drive_port[row], drive_pin[row], GPIO_PIN_SET );		// make drive pin Hi-Z again
	}

	return key;
}
