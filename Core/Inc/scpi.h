/*
 * scpi.h
 *
 *  Created on: 22.08.2021
 *      Author: aziemer
 */

#ifndef CORE_INC_SCPI_H_
#define CORE_INC_SCPI_H_

int scpi_print( const char *fmt, ... );

void SCPI_CDC_RxCallback( uint8_t *Buf, uint16_t Len );

char *SCPI_Execute( char *command );

void Do_SCPI( void );


#endif /* CORE_INC_SCPI_H_ */
