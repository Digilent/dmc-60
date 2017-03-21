/************************************************************************/
/*									*/
/*  Ctrlr.h - Motor Controller Control Loop Functions                   */
/*									*/
/************************************************************************/
/*  Author: Michael T. Alexander					*/
/*  Copyright 2016, Digilent Inc.					*/
/************************************************************************/
/*  Module Description: 						*/
/*									*/
/*  This module contains declarations of routines that can be used to	*/
/*  configure and control the state of the motor Controller.            */
/*  Additionally, it declares routines that implement the different     */
/*  control types.                                                      */
/*									*/
/************************************************************************/
/*  Revision History:						        */
/*									*/
/*  04/13/2016 (MichaelA): created			                */
/*									*/
/************************************************************************/

#if !defined(_CTRLR_INC)
#define	_CTRLR_INC

#include "stdtypes.h"

/* ------------------------------------------------------------ */
/*                  Miscellaneous Declarations			*/
/* ------------------------------------------------------------ */

/* Define the controller link types.
*/
#define linktNone   0
#define linktServo  1
#define linktCAN    2

/* Define the different controller faults that may be signaled.
*/
#define fltOverCurrent      0x0001
#define fltOverTemp         0x0002
#define fltUnderVoltage     0x0004
#define fltGateDriver       0x0008
#define fltComm             0x0010

/* Define macro used to convert milliseconds to ticks and ticks to milliseconds.
*/
#define CtrlrMsToTicks(tms)     (tms << 1)
#define CtrlrTicksToMs(ctick)   (ctick >> 1)

/* Define a macro that can be used to cause the next interrupt to be skipped.
** This macro should be used immediately following any operation that would
** cause the CPU to stall or disable interrupts for a significant period of
** time. By skipping the next interrupt we ensure that no ADC conversion is
** started too quickly for the sampling time to be met, and thus, prevent
** incorrect ADC readings that could otherwise occur.
*/
#define CtrlrSkipNextInterrupt() \
    TMR4 = 0; \
    IFS1bits.T4IF = 0

/* ------------------------------------------------------------ */
/*                  General Type Declarations			*/
/* ------------------------------------------------------------ */

/* Define a data structure to keep track of flags that represent the
** state of the controller.
*/
typedef struct {
    union {
        struct {
            unsigned fFault:1;
            unsigned fHalt:1;
            unsigned fReduceDTC:1;
            unsigned :13;
        };
        WORD    fsFlags;
    };
} CTRLRFLAGS;

/* ------------------------------------------------------------ */
/*                  Variable Declarations			*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*                  Procedure Declarations			*/
/* ------------------------------------------------------------ */

void    CtrlrControlLoop();

void    CtrlrInit();

void    CtrlrSetLinkType(BYTE linktSet);
BYTE    CtrlrGetLinkType();

void    CtrlrForceNeutral();

void    CtrlrSetVoltage(int16_t vltgSet);
int16_t CtrlrGetVoltage();
int16_t CtrlrGetTargetVoltage();
void    CtrlrSetVoltageRampRate(WORD vltgRampSet);
WORD    CtrlrGetVoltageRampRate();

void    CtrlrSetFaultTime(WORD tmsFault);
WORD    CtrlrGetFaultTime();

void    CtrlrSignalFault(WORD fltSignal);
WORD    CtrlrGetFaults();
WORD    CtrlrGetStickyFaults(BOOL fClear);
BYTE    CtrlrGetCurrentFaults();
BYTE    CtrlrGetOverTempFaults();
BYTE    CtrlrGetUVFaults();
BYTE    CtrlrGetGDriverFaults();
BYTE    CtrlrGetCommFaults();
void    CtrlrClearFaultCounts(BYTE fsClear);

void    CtrlrSetHalt();
void    CtrlrClearHalt();
BOOL    CtrlrIsHalted();
BOOL    CtrlrIsDTCReduced();

void    CtrlrVoltageMode();
int32_t CtrlrMplrFromTemp(int16_t tmpCur);

/* ------------------------------------------------------------ */


#endif
