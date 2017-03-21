/************************************************************************/
/*									*/
/*  HBridge.h - Motor Controller H-Bridge Functions                     */
/*									*/
/************************************************************************/
/*  Author: Michael T. Alexander					*/
/*  Copyright 2016, Digilent Inc.					*/
/************************************************************************/
/*  Module Description: 						*/
/*									*/
/*  This module contains declarations of routines that can be used to	*/
/*  configure and control the Motor Controller H-Bridge.                */
/*									*/
/************************************************************************/
/*  Revision History:						        */
/*									*/
/*  03/03/2016 (MichaelA): created			                */
/*  09/09/2016 (MichaelA): added HBridgeSetBrakeOverride and            */
/*      HBridgeClrBrakeOverride to allow brake override mode to be      */
/*      enabled/disabled                                                */
/*									*/
/************************************************************************/

#if !defined(_HBRIDGE_INC)
#define	_HBRIDGE_INC

#include "stdtypes.h"

/* ------------------------------------------------------------ */
/*                  Miscellaneous Declarations			*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*                  General Type Declarations			*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*                  Variable Declarations			*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*                  Procedure Declarations			*/
/* ------------------------------------------------------------ */

void    HBridgeTick();
void    HBridgeInit();
void    HBridgeSetVoltage(int16_t vltgSet);
void    HBridgeSetMaxVoltage(int32_t vltgSetMax);
int32_t HBridgeGetMaxVoltage();
void    HBridgeToggleBrakeCoast();
void    HBridgeSetBrakeCoast(BOOL fBC);
BOOL    HBridgeGetBrakeCoast();
void    HBridgeSetBrakeOverride(BOOL fBC);
void    HBridgeClrBrakeOverride();
void    HBridgeBrakeCoast();

/* ------------------------------------------------------------ */


#endif
