/************************************************************************/
/*									*/
/*  Cmd.h - Motor Controller Command Functions                          */
/*									*/
/************************************************************************/
/*  Author: Michael T. Alexander					*/
/*  Copyright 2016, Digilent Inc.					*/
/************************************************************************/
/*  Module Description: 						*/
/*									*/
/*  This module contains declarations of routines that can be used to	*/
/*  enqueue (send) and dequeue (process) commands to change the         */
/*  operating state of the motor controller or to retrieve information  */
/*  about the current state of the motor controller.                    */
/*									*/
/************************************************************************/
/*  Revision History:						        */
/*									*/
/*  04/26/2016 (MichaelA): created			                */
/*									*/
/************************************************************************/

#if !defined(_CMD_INC)
#define	_CMD_INC

#include "stdtypes.h"

/* ------------------------------------------------------------ */
/*                  Miscellaneous Declarations			*/
/* ------------------------------------------------------------ */

/* Define the commands that can be sent via the command queue.
*/
#define cmdForceNeutral     0x00
#define cmdVoltageMode      0x01
#define cmdVoltageSet       0x02
#define cmdVoltageRateSet   0x03
#define cmdMaxVoltageSet    0x48

/* Declare macros used to add the various commands to the queue.
*/
#define CmdForceNeutral() \
        FCmdAddToQueue(cmdForceNeutral, 0,0,0)

#define CmdVoltageMode(bEnable) \
        FCmdAddToQueue(cmdVoltageMode, bEnable, 0, 0)

#define CmdVoltageSet(vltgSet) \
        FCmdAddToQueue(cmdVoltageSet, vltgSet, 0, 0)

#define CmdVoltageRateSet(vltgRateSet) \
        FCmdAddToQueue(cmdVoltageRateSet, vltgRateSet, 0, 0)

#define CmdMaxVoltageSet(vltgMaxSet) \
        FCmdAddToQueue(cmdMaxVoltageSet, vltgMaxSet, 0, 0)

/* ------------------------------------------------------------ */
/*                  General Type Declarations			*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*                  Variable Declarations			*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*                  Procedure Declarations			*/
/* ------------------------------------------------------------ */

BOOL    FCmdAddToQueue(DWORD cmdAdd, DWORD param1, DWORD param2, DWORD param3);
void    CmdProcessQueue(BOOL fFaultActive);

/* ------------------------------------------------------------ */


#endif
