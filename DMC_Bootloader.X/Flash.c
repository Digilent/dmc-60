/************************************************************************/
/*									*/
/*  Flash.c - DSPIC33 Flash Read / Write Functions                      */
/*									*/
/************************************************************************/
/*  Author: Michael T. Alexander					*/
/*  Copyright 2016, Digilent Inc.					*/
/************************************************************************/
/*  Module Description: 						*/
/*									*/
/*  This module contains definitions of routines that can be used to	*/
/*  read and write to a flash page of the DSPIC33.                      */
/*									*/
/************************************************************************/
/*  Revision History:						        */
/*									*/
/*  09/16/2016 (MichaelA): created			                */
/*  09/19/2016 (MichaelA): updated comments in header blocks            */
/*									*/
/************************************************************************/

/* ------------------------------------------------------------ */
/*		Include File Definitions			*/
/* ------------------------------------------------------------ */

#include <xc.h>
#include <libpic30.h>
#include <string.h>
#include "DMC.h"
#include "stdtypes.h"
#include "Flash.h"

/* ------------------------------------------------------------ */
/*		Local Type Definitions				*/
/* ------------------------------------------------------------ */

/* Define macros that can be used to disable and enable interrupts.
*/
#define DISABLE_INTERRUPTS \
        INTCON2bits.GIE = 0;

#define ENABLE_INTERRUPTS \
        INTCON2bits.GIE = 1;

/* ------------------------------------------------------------ */
/*		Global Variables				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Local Variables					*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Forward Declarations				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Interrupt Service Routines	            	*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Procedure Definitions				*/
/* ------------------------------------------------------------ */
/***	FlashRead
**
**  Parameters:
**      paddrRead   - program address in flash to read from
**      pbufRead    - pointer to buffer to receive data read from flash
**      cbRead      - number of bytes to read from flash
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine reads the specified number of words from flash starting at
**      the address specified by paddrRead, into the buffer specified by
**      pbufRead.
**
**  Notes:
**      Interrupts should be disabled while reading from flash. It is the
**      caller's responsbility to disable and then re-enable interrupts.
*/
void
FlashRead(_prog_addressT  paddrRead, WORD* pbufRead, WORD cbRead) {

    unsigned int    tblpgSave;

    /* Save the TBLPAG register.
    */
    tblpgSave = TBLPAG;

    /* Attepmt to load the configuration from a Flash based EEPROM.
    */
    _memcpy_p2d16(pbufRead, paddrRead, cbRead);

    TBLPAG = tblpgSave;
}

/* ------------------------------------------------------------ */
/***	FlashWrite
**
**  Parameters:
**      paddrWrite  - program address in flash to write to
**      pbufWrite   - pointer to buffer to containg data to write to flash
**      cwRead      - number of words to write to flash
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine writes the specified number of words to flash starting at
**      the address specified by paddrWrite, from the buffer specified by
**      pbufWrite.
**
**  Notes:
**      This function erases an entire flash page. It is the callers
**      responsibility to ensure that any unmodified data is read, buffered, and
**      then passed to this funciton to be re-written after the erase operation.
**
**      If the word count is greater than 2 then the first two words are assumed
**      to be magic words used to indicate that the flash page is valid. The
**      flash section is erased, all other words are written, and then the first
**      two words are written last.
**
**      Interrupts should be disabled while write to flash. It is the
**      caller's responsbility to disable and then re-enable interrupts.
*/
void
FlashWrite(_prog_addressT  paddrWrite, WORD* pbufWrite, WORD cwWrite) {

    unsigned int    tblpgSave;
    WORD            cwProg;

    /* Save the TBLPAG register.
    */
    tblpgSave = TBLPAG;

    /* Erase the Flash page.
    */
    _erase_flash(paddrWrite);

    /* Write data to flash. If the word count is greater than 2 then we will
    ** assume that the first two words are used to store magic words, which will
    ** be used later to determine whether or not the data in the flash page is
    ** valid. Therefore these words will be skipped initially and written last.
    */
    if ( 2 < cwWrite ) {
        cwProg = 2;
    }
    else {
        cwProg = 0;
    }
    
    while ( cwProg < cwWrite ) {
        if ( (cwProg + 1) < cwWrite ) {
            _write_flash_word32(paddrWrite + (cwProg << 1), pbufWrite[cwProg], pbufWrite[cwProg + 1]);
            cwProg += 2;
        }
        else {
            _write_flash_word32(paddrWrite + (cwProg << 1), pbufWrite[cwProg], 0);
            cwProg += 1;
        }
    }

    /* If the word count is greater than two then we still need to write the
    ** last two words to flash.
    */
    if ( 2 < cwWrite ) {
        _write_flash_word32(paddrWrite, pbufWrite[0], pbufWrite[1]);
    }

    /* Restore the TBLPAG register.
    */
    TBLPAG = tblpgSave;
}

/* ------------------------------------------------------------ */
/***	FlashErasePage
**
**  Parameters:
**      ipageErase  - page number to erase
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine erases the page of flash memory specified by ipageErase.
**      Please note that no bounds checking is performed. The caller must ensure
**      that the page specified is valid and that erasing the page will not
**      nuke the firmware.
**
**  Notes:
**      Interrupts should be disabled while erasing flash. It is the
**      caller's responsbility to disable and then re-enable interrupts.
**
**      According to the DSPIC33 datasheet it may take up to 23.1ms to erase a
**      page of flash. During this time the CPU is stalled.
*/
void
FlashErasePage(WORD ipageErase) {

    _prog_addressT  paddr;

    _init_prog_address(paddr, _RESET);

    paddr += ((_prog_addressT)ipageErase * 2048);

    _erase_flash(paddr);
}

void
FlashRead24(_prog_addressT  paddrRead, DWORD* pbufRead, WORD cdwRead) {

}

/* ------------------------------------------------------------ */
/***	FlashWrite24
**
**  Parameters:
**      paddrWrite  - program address in flash to write to
**      pbufWrite   - pointer to buffer to containg data to write to flash
**      cdwRead     - number of 24-bit words to write to flash
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine writes the specified number of 24-bit words to flash
**      starting at the address specified by paddrWrite, from the buffer
**      specified by pbufWrite. A minimum of 2 words must be written so the
**      starting address should be aligned to a two word boundary and the
**      minimum word count should be 2. If an odd word count is specified then
**      the last 24-bit word will be written with 0x000000.
**
**  Notes:
**      This function does NOT erase the flash page. It is the callers
**      responsibility to ensure that any 24-bit words that written were
**      previously erased.
**
**      Interrupts should be disabled while write to flash. It is the
**      caller's responsbility to disable and then re-enable interrupts.
*/
void
FlashWrite24(_prog_addressT  paddrWrite, const DWORD* pbufWrite, WORD cdwWrite) {

    unsigned int    tblpgSave;
    WORD            cdwProg;

    /* Save the TBLPAG register.
    */
    tblpgSave = TBLPAG;

    cdwProg = 0;
    while ( cdwProg < cdwWrite ) {
        if ( (cdwProg + 1) < cdwWrite ) {
            _write_flash_word48(paddrWrite + (cdwProg << 1), pbufWrite[cdwProg], pbufWrite[cdwProg + 1]);
            cdwProg += 2;
        }
        else {
            _write_flash_word48(paddrWrite + (cdwProg << 1), pbufWrite[cdwProg], 0);
            cdwProg += 1;
        }
    }

    /* Restore the TBLPAG register.
    */
    TBLPAG = tblpgSave;
}

/* ------------------------------------------------------------ */
/***	FlashEraseWriteVerifyPage24
**
**  Parameters:
**      ipageWrite  - number of flash page to erase and write
**      pbufWrite   - pointer to buffer to containg data to write to flash
**      cdwWrite    - number of 24-bit words to write to flash
**      cRetry      - number of times to retry the operation if verification
**                    fails
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine erases the page of flash memory specified by ipageErase,
**      then writes the specified number of 24-bit words to the flash starting
**      at the beginning page, and then verifies that the flash memory contents
**      match the data that was supposed to be written. A minimum of 2 words
**      must be written so the starting address should be aligned to a two word
**      boundary and the minimum word count should be 2. If an odd word count is
**      specified then the last 24-bit word will be written with 0x000000.
**
**  Notes:
**      Interrupts should be disabled while write to flash. It is the
**      caller's responsbility to disable and then re-enable interrupts.
*/
BOOL
FlashEraseWriteVerifyPage24(WORD ipageWrite, const DWORD* pbufWrite, WORD cdwWrite, BYTE cRetry) {

    _prog_addressT  paddr;
    unsigned int    tblpgSave;
    WORD            cdwProg;
    DWORD           dwTemp;

    /* Save the TBLPAG register.
    */
    tblpgSave = TBLPAG;

    _init_prog_address(paddr, _RESET);
    paddr += ((_prog_addressT)ipageWrite * 2048);

lEraseProgVerify:

    /* Erase the flash page.
    */
    _erase_flash(paddr);

    /* Write the specified number of DWORDS to the flash page.
    */
    cdwProg = 0;
    while ( cdwProg < cdwWrite ) {
        if ( (cdwProg + 1) < cdwWrite ) {
            _write_flash_word48(paddr + (cdwProg << 1), pbufWrite[cdwProg], pbufWrite[cdwProg + 1]);
            cdwProg += 2;
        }
        else {
            _write_flash_word48(paddr + (cdwProg << 1), pbufWrite[cdwProg], 0);
            cdwProg += 1;
        }
    }

    /* Verify that the flash was written correctly.
    */
    cdwProg = 0;
    while ( cdwProg < cdwWrite ) {
        _memcpy_p2d24(&dwTemp, paddr + (cdwProg << 1), 3);
        if ( ( dwTemp & 0x00FFFFFF) != (pbufWrite[cdwProg] & 0x00FFFFFF) ) {
            if ( 0 < cRetry ) {
                cRetry--;
                goto lEraseProgVerify;
            }

            TBLPAG = tblpgSave;
            return fFalse;
        }

        cdwProg++;
    }

    /* Restore the TBLPAG register.
    */
    TBLPAG = tblpgSave;

    return fTrue;
}