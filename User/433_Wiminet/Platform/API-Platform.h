// #############################################################################
// *****************************************************************************
//          Copyright (c) 2007-2008, WiMi-net (Beijing) Tech. Co., Ltd.
//      THIS IS AN UNPUBLISHED WORK CONTAINING CONFIDENTIAL AND PROPRIETARY
//         INFORMATION WHICH IS THE PROPERTY OF WIMI-NET TECH. CO., LTD.
//
//    ANY DISCLOSURE, USE, OR REPRODUCTION, WITHOUT WRITTEN AUTHORIZATION FROM
//               WIMI-NET TECH. CO., LTD, IS STRICTLY PROHIBITED.
// *****************************************************************************
// #############################################################################
//
// File:    API-Platform.h
// Author:  Mickle.ding
// Created: 5/18/2007
//
// Description:  
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#ifndef _API_PLATFORM_INC_

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define _API_PLATFORM_INC_


// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define LITTLE_ENDIAN_MODE                                     0X00

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define BIG_ENDIAN_MODE                                        0X01




// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#ifndef DEBUG

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define DEBUG                                                  0X00

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#endif


// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define DEBUG_PORT                                             ( DEBUG & 0X7F )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define DEBUG_OPEN                                             ( DEBUG & 0X80 )




// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#if ( DEBUG_PORT == 0X01 )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-UART0.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define PutString( X )                                          PutUART0String( X )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#elif ( DEBUG_PORT == 0X02 )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-UART1.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define PutString( X )                                          PutUART1String( X )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#elif ( DEBUG_PORT == 0X03 )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-USART0.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define PutString( X )                                          PutUSART0String( X )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#elif ( DEBUG_PORT == 0X04 )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-USART1.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define PutString( X )                                          PutUSART1String( X )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#elif ( DEBUG_PORT == 0X05 )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-USART2.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define PutString( X )                                          PutUSART2String( X )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#elif ( DEBUG_PORT == 0X06 )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-LEUART0.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define PutString( X )                                          PutLEUART0String( X )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#elif ( DEBUG_PORT == 0X07 )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-LEUART1.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define PutString( X )                                          PutLEUART1String( X )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#elif ( DEBUG_PORT == 0X08 )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define PutString( X )                                          WriteSocket0String( X )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#elif ( DEBUG_PORT == 0X09 )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define PutString( X )                                          WriteSocket1String( X )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#elif ( DEBUG_PORT == 0X0A )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define PutString( X )                                          WriteSocket2String( X )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#elif ( DEBUG_PORT == 0X0B )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define PutString( X )                                          WriteSocket3String( X )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#else

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define PutString( X )                                          

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#endif




// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#if defined ( __CC_ARM ) 

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define XDATA

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define IDATA

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define CODE

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define CPU_ENDIAN_MODE                                        LITTLE_ENDIAN_MODE                                         

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#elif defined ( __C51__ )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define CPU_ENDIAN_MODE                                        BIG_ENDIAN_MODE                                     
 
// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define XDATA                                                  xdata

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define IDATA                                                  idata

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define CODE                                                   code

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#elif defined ( __CX51__ )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define CPU_ENDIAN_MODE                                        BIG_ENDIAN_MODE                                     

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define XDATA                                                  xdata

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define IDATA                                                  idata

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define CODE                                                   code

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#endif

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
unsigned short ntohs( unsigned short iValue );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
unsigned long ntohl( unsigned long dwValue );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void debugthread( char * pThreadName, unsigned char iStatus );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#endif
