
/************************************************************************/
/*									*/
/*  Crc.c - DSPIC33 CRC Calculation Functions                           */
/*									*/
/************************************************************************/
/*  Author: Michael T. Alexander					*/
/*  Copyright 2016, Digilent Inc.					*/
/************************************************************************/
/*  Module Description: 						*/
/*									*/
/*  This module contains definitions of routines that can be used to	*/
/*  calculate the CRC for a block of data.                              */
/*									*/
/************************************************************************/
/*  Revision History:						        */
/*									*/
/*  10/05/2016 (MichaelA): created			                */
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
#include "Crc.h"

/* ------------------------------------------------------------ */
/*		Local Type Definitions				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Global Variables				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Local Variables					*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Forward Declarations				*/
/* ------------------------------------------------------------ */

DWORD   MirrorDword(DWORD v);

/* ------------------------------------------------------------ */
/*		Interrupt Service Routines	            	*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Procedure Definitions				*/
/* ------------------------------------------------------------ */
/***	CrcInit
**
**  Parameters:
**      none
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine initializes the DSPIC33 CRC module so that it may be used
**      to calculate a 32-bit CRC from 16-bit data. The CRC moudle is then
**      enabled and ready for use.
*/
void
CrcInit() {

    CRCCON1bits.CRCEN = 0; // disable CRC module
    CRCCON1bits.CRCGO = 0; // CRC serial shift turned off
    CRCCON1bits.CRCISEL = 0; // set interrupt flag when shift is complete and results are ready
    CRCCON1bits.LENDIAN = 1; // use little endian for shifting

    CRCCON2bits.PLEN = 0x1F; // set polynomial length to 32-bits
    CRCXORL = 0x1DB7; // set low word of polynomial
    CRCXORH = 0x04C1; // set high word of polynomial
    CRCCON2bits.DWIDTH = 0x07; // set data width to 8-bits

    CRCWDATL = 0; // clear low word of CRC shift register
    CRCWDATH = 0; // clear high word of CRC shift register

    CRCCON1bits.CRCEN = 1; // enable CRC module
}

/* ------------------------------------------------------------ */
/***	CrcTerm
**
**  Parameters:
**      none
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine disables the DSPIC33 CRC module.
*/
void
CrcTerm() {

    CRCCON1bits.CRCEN = 0; // disable CRC module
    IFS4bits.CRCIF = 0; // clear interrupt flag
}

/* ------------------------------------------------------------ */
/***	CrcCalcCRC32
**
**  Parameters:
**      crcInit     - initial value to use when performing CRC calculation
**      pbBuf       - pointer to a buffer containing data for CRC calculation
**      cbBuf       - number of bytes in the buffer
**
**  Return Value:
**      32-bit CRC computed from the specified data
**
**  Errors:
**      none
**
**  Description:
**      This routine calculates a CRC32 for the data specified by the buffer
**      pwBuf.
**
**  Notes:
**      This operation may take a long time to complete.
*/
DWORD
CrcCalcCRC32(DWORD crcInit, const BYTE* pbBuf, WORD cbBuf) {

    DWORD   crcResult;
    BYTE*   pb;

    pb = (BYTE*)&CRCDATL;

    CRCWDATL = (WORD)(crcInit & 0x0000FFFF);
    CRCWDATH = (WORD)(crcInit >> 16);

    IFS4bits.CRCIF = 0;
    CRCCON1bits.CRCGO =1 ;

    do {
        while ( 1 != CRCCON1bits.CRCMPT );
        while (( 0 == CRCCON1bits.CRCFUL ) && ( 0 < cbBuf )) {
            *pb = *pbBuf;
            pbBuf++;
            cbBuf--;
        }
    } while ( 0 < cbBuf );

    while ( 0 == IFS4bits.CRCIF ) {
        Nop();
    }

    CRCCON1bits.CRCGO =0;

    // Correct number of 0 bits appended to the end of the data stream because of indirect crc method
    // The number of 0 bits should be equal to the CRC width ?16x 0 bits for 16-bit CRCs and
    // 32x 0 bits for 32-bit CRCs
    CRCDATL = 0x0000;
    while(CRCCON1bits.CRCFUL == 1);
    CRCDATL = 0x0000;
    IFS4bits.CRCIF=0;
    CRCCON1bits.CRCGO =1;

    while ( 0 == IFS4bits.CRCIF ) {
        Nop();
    }

    CRCCON1bits.CRCGO = 0;

    crcResult = ((((DWORD)CRCWDATH) << 16) | (DWORD)CRCWDATL);

    return MirrorDword(crcResult);
}

/* ------------------------------------------------------------ */
/***	MirrorDword
**
**  Parameters:
**      dw      - DWORD to mirror
**
**  Return Value:
**      mirrored double word
**
**  Errors:
**      none
**
**  Description:
**      This routine returns a mirroed copy of the double word that's specified
**      by dw.
*/
DWORD
MirrorDword(DWORD dw)
{
    DWORD   r;
    DWORD   s;

    r = dw;
    s = 31;
    for (dw >>= 1; dw; dw >>= 1)
    {
      r <<= 1;
      r |= dw & 1;
      s--;
    }
    r <<= s;
    return (r);
}