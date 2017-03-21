/************************************************************************/
/*									*/
/*  Cmd.c - Motor Controller Command Functions                          */
/*									*/
/************************************************************************/
/*  Author: Michael T. Alexander					*/
/*  Copyright 2016, Digilent Inc.					*/
/************************************************************************/
/*  Module Description: 						*/
/*									*/
/*  This module contains definitions of routines that can be used to    */
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

/* ------------------------------------------------------------ */
/*		Include File Definitions			*/
/* ------------------------------------------------------------ */

#include <xc.h>
#include <string.h>
#include "DMC60.h"
#include "stdtypes.h"
#include "Ctrlr.h"
#include "HBridge.h"
#include "Cmd.h"

/* ------------------------------------------------------------ */
/*		Local Type Definitions				*/
/* ------------------------------------------------------------ */

/* Declare a data structure that can be used to store a command
** for the purposes of implementing a command queue.
*/
typedef struct {
    DWORD   cmd;
    DWORD   param1;
    DWORD   param2;
    DWORD   param3;
} CMDENTRY;

/* Define the maximum number of commands that can be placed in
** the command queue.
*/
#define ccmdQueueMax        32

/* ------------------------------------------------------------ */
/*		Global Variables				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Local Variables					*/
/* ------------------------------------------------------------ */

/* Declare the variables used to implement the command queue.
*/
static volatile CMDENTRY    rgcmdQueue[ccmdQueueMax];
static volatile WORD        ccmdCur = 0;
static volatile WORD        icmdRead = 0;
static volatile WORD        icmdWrite = 0;

/* ------------------------------------------------------------ */
/*		Forward Declarations				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Interrupt Service Routines	            	*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Procedure Definitions				*/
/* ------------------------------------------------------------ */
/***	FCmdAddToQueue
**
**  Parameters:
**      cmdAdd  - command code for the command being added to the queue
**      param1  - first command specific parameter
**      param2  - second command specific parmaeter
**      param3  - third command specific parameter
**
**  Return Value:
**      fTrue if command is successfully added to the queue, fFalse if the
**      command cannot be added because the queue is full
**
**  Errors:
**      none
**
**  Description:
**      This function adds the specified command, along with any required
**      parameters, to the command queue. Any command added to the queue via
**      this function will be processed at a later point in time when the
**      CmdProcessQueue() function is called.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
BOOL
FCmdAddToQueue(DWORD cmdAdd, DWORD param1, DWORD param2, DWORD param3) {

    /* Make sure that there is enough space before adding another command
    ** to the queue.
    */
    if ( ccmdQueueMax > ccmdCur ) {
        /* There's enough space so add the command to the queue, update the
        ** write index and the count, and return true.
        */
        rgcmdQueue[icmdWrite].cmd = cmdAdd;
        rgcmdQueue[icmdWrite].param1 = param1;
        rgcmdQueue[icmdWrite].param2 = param2;
        rgcmdQueue[icmdWrite].param3 = param3;
        icmdWrite = (icmdWrite + 1) % ccmdQueueMax;
        ccmdCur++;

        return fTrue;
    }
    else {
        /* There isn't enough space to add the command.
        */
        return fFalse;
    }
}

/* ------------------------------------------------------------ */
/***	CmdProcessQueue
**
**  Parameters:
**      fFaultActive    - fTrue if there is currently a fault active that should
**                        prevent the changes to the current output voltage
**                        setting
**                      - fFalse if there are no faults active that should
**                        prevent the output voltage from being changed
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function processes any commands that are in the command queue. The
**      commands are processed in the order in which they were received (FIFO).
**      If fTrue is specified for fFaultActive then any commands that set
**      the voltage of the H-Bridge will be discarded, with no change being made
**      to the output voltage setting.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
void
CmdProcessQueue(BOOL fFaultActive) {

    /* Process all of the commands that presently in the queue.
    */
    while ( 0 < ccmdCur ) {

        switch ( rgcmdQueue[icmdRead].cmd ) {

            case cmdForceNeutral:
                CtrlrForceNeutral();
                break;

            case cmdVoltageMode:
                /* M00TODO: implement this.
                */
                break;

            case cmdVoltageSet:
                /* Only set the target voltage if there isn't presently a fault.
                */
                if ( ! fFaultActive ) {
                    CtrlrSetVoltage((int16_t)rgcmdQueue[icmdRead].param1);
                }
                break;

            case cmdVoltageRateSet:
                CtrlrSetVoltageRampRate((WORD)rgcmdQueue[icmdRead].param1);
                break;

            case cmdMaxVoltageSet:
                /* M00TODO: fix typecast, as max voltage is supposed to be an
                ** 8.8 unsigned.
                */
                HBridgeSetMaxVoltage((int32_t)rgcmdQueue[icmdRead].param1);
                break;

            default:
                /* Unknown or unimplemented commands will be discarded.
                */
                break;
        }

        icmdRead = (icmdRead + 1) % ccmdQueueMax;
        ccmdCur--;
    }
}

/******************************************************************************/