/************************************************************************/
/*									*/
/*  prodcfg.h - MC Bootloader Firmware Product Configuration FIle       */
/*									*/
/************************************************************************/
/*  Author: Michael T. Alexander					*/
/*  Copyright 2016, Digilent Inc.					*/
/************************************************************************/
/*  Module Description: 						*/
/*									*/
/*  This header file is used to define the capabilities of the Motor    */
/*  Controller Bootloader Firmware. This includes specification of the  */
/*  Firmware Version.                                                   */
/*									*/
/************************************************************************/
/*  Revision History:						        */
/*									*/
/*  09/20/2016 (MichaelA): created			                */
/*									*/
/************************************************************************/

#if !defined(_PRODCFG_INC)
#define	_PRODCFG_INC

#include "stdtypes.h"

/* ------------------------------------------------------------ */
/*                  Miscellaneous Declarations			*/
/* ------------------------------------------------------------ */

/* Define the Firmware Revision. Please note that the firmware
** revision consists of a major version and a minor version number.
** The firmware revision is stored in flash as a 16-bit value with
** the minor revision being stored as the least significant byte
** and the major revision at the most significant byte.
*/
#define fwverMjr    1
#define fwverMin    1

/* Define the different types of firwmare images.
*/
#define imgtypApp       0
#define imgtypBoot      1
#define imgtypAuxBoot   2

/* ------------------------------------------------------------ */
/*                  General Type Declarations			*/
/* ------------------------------------------------------------ */

typedef struct {
    union {
        struct {
            unsigned fStayInBootloader:1;
            unsigned fAckSoftReset:1;
            unsigned :14;
        };
        WORD    fsFlags;
    };
} BOOTFLAGS;

/* ------------------------------------------------------------ */
/*                  Variable Declarations			*/
/* ------------------------------------------------------------ */

extern const BYTE imgtypCur;
extern WORD sessidPrev;
extern BOOTFLAGS bootflgs;


/* ------------------------------------------------------------ */
/*                  Procedure Declarations			*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */


#endif
