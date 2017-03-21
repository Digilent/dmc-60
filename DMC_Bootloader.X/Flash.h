/************************************************************************/
/*									*/
/*  Flash.h - DSPIC33 Flash Read / Write Functions                      */
/*									*/
/************************************************************************/
/*  Author: Michael T. Alexander					*/
/*  Copyright 2016, Digilent Inc.					*/
/************************************************************************/
/*  Module Description: 						*/
/*									*/
/*  This module contains declarations of routines that can be used to	*/
/*  read and write to a flash section of the DSPIC33.                   */
/*									*/
/************************************************************************/
/*  Revision History:						        */
/*									*/
/*  09/16/2016 (MichaelA): created			                */
/*									*/
/************************************************************************/

#if !defined(_FLASH_INC)
#define	_FLASH_INC

#include "stdtypes.h"

/* ------------------------------------------------------------ */
/*                  Miscellaneous Declarations			*/
/* ------------------------------------------------------------ */

/* Define indexs for the flash memory pages that are valid for
** application images and aux bootloaders.
*/
#define iflashpAppMin   0
#define	iflashpAppMax   32

/* ------------------------------------------------------------ */
/*                  General Type Declarations			*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*                  Variable Declarations			*/
/* ------------------------------------------------------------ */

__psv__ extern const DWORD __attribute__((space(psv), address(0x000000))) _RESET;

/* ------------------------------------------------------------ */
/*                  Procedure Declarations			*/
/* ------------------------------------------------------------ */

void    FlashRead(_prog_addressT  paddrRead, WORD* pbufRead, WORD cbRead);
void    FlashWrite(_prog_addressT  paddrWrite, WORD* pbufWrite, WORD cwWrite);

void    FlashErasePage(WORD ipageErase);
void    FlashRead24(_prog_addressT  paddrRead, DWORD* pbufRead, WORD cdwRead);
void    FlashWrite24(_prog_addressT  paddrWrite, const DWORD* pbufWrite, WORD cdwWrite);
BOOL    FlashEraseWriteVerifyPage24(WORD ipageWrite, const DWORD* pbufWrite, WORD cdwWrite, BYTE cRetry);

/* ------------------------------------------------------------ */


#endif
