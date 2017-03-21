
/************************************************************************/
/*									*/
/*  Cfg.c - Motor Controller Flash Configuration Functions              */
/*									*/
/************************************************************************/
/*  Author: Michael T. Alexander					*/
/*  Copyright 2016, Digilent Inc.					*/
/************************************************************************/
/*  Module Description: 						*/
/*									*/
/*  This module contains definitions of routines that can be used to	*/
/*  read the motor configuration from flash and to write the current    */
/*  configuration to flash as necessary.                                */
/*									*/
/************************************************************************/
/*  Revision History:						        */
/*									*/
/*  05/20/2016 (MichaelA): created			                */
/*  10/12/2016 (MichaelA): added a bunch of functions for getting and   */
/*      setting additional fields of information                        */
/*									*/
/************************************************************************/

/* ------------------------------------------------------------ */
/*		Include File Definitions			*/
/* ------------------------------------------------------------ */

#include <xc.h>
#include <libpic30.h>
#include <string.h> // M00TODO: remove this later
#include "DMC60.h"
#include "stdtypes.h"
#include "Servo.h"
#include "Flash.h"
#include "Can.h"
#include "Cfg.h"

/* ------------------------------------------------------------ */
/*		Local Type Definitions				*/
/* ------------------------------------------------------------ */

/* Define magic words used for reading and writing flash memory.
*/
#define mgkWord1    0x9B25
#define mgkWord2    0xD96E

/* Define macros that can be used to disable and enable interrupts.
*/
#define DISABLE_INTERRUPTS \
        INTCON2bits.GIE = 0;

#define ENABLE_INTERRUPTS \
        INTCON2bits.GIE = 1;

/* ------------------------------------------------------------ */
/*		Global Variables				*/
/* ------------------------------------------------------------ */

__psv__ extern const WORD __attribute__((space(psv), address(0x107FE))) _FWVERAPP;
__psv__ extern const WORD __attribute__((space(psv), address(0x157EA))) _FWVERBOOT;

/* ------------------------------------------------------------ */
/*		Local Variables					*/
/* ------------------------------------------------------------ */

const WORD __attribute__ ((space(prog), aligned(_FLASH_PAGE), section(".eeprom_dspic33"), address(0x10800))) pageEEP[_FLASH_PAGE];
const WORD __attribute__ ((space(prog), aligned(_FLASH_PAGE), section(".strdesc_dspic33"), address(0x11000))) pageSTRDESC[_FLASH_PAGE];

static STRDESC      strdescCur;
static WORD         rgwEEP[_FLASH_PAGE];
static DESCTBLENTRY rgdesctbl[cdesctblMax];
static MTRCFG*      pmtrcfgCur;
static DWORD        pdidDevice;

/* ------------------------------------------------------------ */
/*		Forward Declarations				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Interrupt Service Routines	            	*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Procedure Definitions				*/
/* ------------------------------------------------------------ */
/***	CfgInit
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
**      This function reads the EEPROM and String Descriptors from
**      RAM and intializes the in memory copy of these sections. If the
**      flash is blank or corrupt then both the configuration and string
**      descriptors are default initialize and then written to their applicable
**      sections of flash.
**
**  Notes:
**      This funciton disables interrupts while reading and writing flash.
**      Interrupts are enabled once these operations have been completed.
*/
void
CfgInit() {

    _prog_addressT  paddr;

    /* Disable interrupts. M00TODO: consider whether or not this actually needs
    ** to be done.
    */
    DISABLE_INTERRUPTS

    /* Read the device's PDID.
    */
    _init_prog_address(paddr, _FUID0);
    FlashRead(paddr, (WORD*)(&pdidDevice), 4);

    /* Read the EEPROM into ram.
    */
    _init_prog_address(paddr, pageEEP);
    FlashRead(paddr, rgwEEP, sizeof(rgwEEP));

    /* Point the motor config pointer at the RAM copy of the EEPROM. Later we
    ** will make modifications to the RAM copy and then write the entire EEPROM
    ** to flash.
    */
    pmtrcfgCur = (MTRCFG*)rgwEEP;

    /* Make sure that the motor configuration read from the flash is valid, and
    ** if not, default initialize it.
    */
    if (( mgkWord1 != pmtrcfgCur->mgk1 ) || ( mgkWord2 != pmtrcfgCur->mgk2 )) {
        // Init Version 0 fields.
        pmtrcfgCur->mgk1 = mgkWord1;
        pmtrcfgCur->mgk2 = mgkWord2;
        pmtrcfgCur->ver = verMtrCfgApp;
        pmtrcfgCur->devnum = devnumDefault;
        strcpy(pmtrcfgCur->szDevName, "Motor Controller");
        // Init Version 1 fields.
        pmtrcfgCur->servocal.cntServoMin = cntServoMinDefault;
        pmtrcfgCur->servocal.cntServoMax = cntServoMaxDefault;
        pmtrcfgCur->servocal.cntServoNeutralMin = cntServoNeutralMinDefault;
        pmtrcfgCur->servocal.cntServoNeutralMax = cntServoNeutralMaxDefault;
        pmtrcfgCur->fBrake = fFalse;

        FlashWrite(paddr,rgwEEP, sizeof(rgwEEP) >> 1);
    }

    if ( verMtrCfgApp > pmtrcfgCur->ver ) {
        pmtrcfgCur->ver = verMtrCfgApp;
        pmtrcfgCur->servocal.cntServoMin = cntServoMinDefault;
        pmtrcfgCur->servocal.cntServoMax = cntServoMaxDefault;
        pmtrcfgCur->servocal.cntServoNeutralMin = cntServoNeutralMinDefault;
        pmtrcfgCur->servocal.cntServoNeutralMax = cntServoNeutralMaxDefault;
        pmtrcfgCur->fBrake = fFalse;

        FlashWrite(paddr,rgwEEP, sizeof(rgwEEP) >> 1);
    }

    /* Read the string descriptors from flash.
    */
    _init_prog_address(paddr, pageSTRDESC);
    FlashRead(paddr, (WORD*)(&strdescCur), sizeof(STRDESC));

    /* If both magic words aren't present then assume that the flash is corrupt,
    ** default initialize all string descriptors, and write the struct to flash.
    */
    if (( mgkWord1 != strdescCur.mgk1 ) || ( mgkWord2 != strdescCur.mgk2 )) {
        strdescCur.mgk1 = mgkWord1;
        strdescCur.mgk2 = mgkWord2;
        strdescCur.ver = verStrDescApp;
        strcpy(strdescCur.szManName, szManNameDefault);
        strcpy(strdescCur.szProdName, szProdNameDefault);
        strcpy(strdescCur.szManDate, szManDateDefault);
        strcpy(strdescCur.szHardwareVer, szHardwareVerDefault);
        strcpy(strdescCur.szSN, szSNDefault);
        FlashWrite(paddr,(WORD*)(&strdescCur), sizeof(STRDESC) >> 1);
    }

    /* Populate the descriptor table.
    */
    rgdesctbl[idxSzDevName].idfld = idfldSzDevName;
    rgdesctbl[idxSzDevName].cbfld = strlen(pmtrcfgCur->szDevName);
    rgdesctbl[idxSzDevName].pb = (const BYTE*)pmtrcfgCur->szDevName;

    rgdesctbl[idxSzManName].idfld = idfldSzManName;
    rgdesctbl[idxSzManName].cbfld = strlen(strdescCur.szManName);
    rgdesctbl[idxSzManName].pb = (const BYTE*)strdescCur.szManName;

    rgdesctbl[idxSzProdName].idfld = idfldSzProdName;
    rgdesctbl[idxSzProdName].cbfld = strlen(strdescCur.szProdName);
    rgdesctbl[idxSzProdName].pb = (const BYTE*)strdescCur.szProdName;

    rgdesctbl[idxSzManDate].idfld = idfldSzManDate;
    rgdesctbl[idxSzManDate].cbfld = strlen(strdescCur.szManDate);
    rgdesctbl[idxSzManDate].pb = (const BYTE*)strdescCur.szManDate;

    rgdesctbl[idxSzHardwareVer].idfld = idfldSzHardWareVer;
    rgdesctbl[idxSzHardwareVer].cbfld = strlen(strdescCur.szHardwareVer);
    rgdesctbl[idxSzHardwareVer].pb = (const BYTE*)strdescCur.szHardwareVer;

    rgdesctbl[idxSzSN].idfld = idfldSzSN;
    rgdesctbl[idxSzSN].cbfld = strlen(strdescCur.szSN);
    rgdesctbl[idxSzSN].pb = (const BYTE*)strdescCur.szSN;

    /* Reneable interrupts now that we are done programming flash memory.
    */
    ENABLE_INTERRUPTS
}

/* ------------------------------------------------------------ */
/***	CfgWriteEeprom
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
**      This routine writes the RAM copy of the motor controller configuration
**      to the EEPROM section of the flash memory. An entire page of flash is
**      written.
**
**  Notes:
**      Interrupts must be disabled to ensure that this operation as atomic. It
**      Is the callers responsbility to disable and re-enable interrupts.
**
**      It may take up to 54ms to write a full page of flash.
*/
void
CfgWriteEeprom() {

    _prog_addressT  paddr;

    /* Write the RAM contents of the EEPROM to the flash.
    */
    _init_prog_address(paddr, pageEEP);
    FlashWrite(paddr, rgwEEP, sizeof(rgwEEP) >> 1);
}

/* ------------------------------------------------------------ */
/***	CfgWriteStrDescToFlash
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
**      This routine writes the RAM copy of the string descriptor table to the
**      string descriptor section of flash memory.
**
**  Notes:
**      Interrupts must be disabled to ensure that this operation as atomic. It
**      Is the callers responsbility to disable and re-enable interrupts.
*/
void
CfgWriteStrDescToFlash() {

    _prog_addressT  paddr;

    /* Write the string descriptors to flash.
    */
    _init_prog_address(paddr, pageSTRDESC);
    FlashWrite(paddr, (WORD*)(&strdescCur), sizeof(STRDESC) >> 1);
}

/* ------------------------------------------------------------ */
/***	CfgGetDescTable
**
**  Parameters:
**      pbBuf   - pointer to buffer to receive a copy of the descriptor table
**      cbBuf   - size of the buffer
**
**  Return Value:
**      number of bytes copied to the buffer
**
**  Errors:
**      none
**
**  Description:
**      This routine copies the descriptor table, including the field ID and
**      field byte count, to the user specified buffer.
*/
WORD
CfgGetDescTable(BYTE* pbBuf, WORD cbBuf) {

    WORD    i;
    WORD    cb;

    i = 0;
    cb = 0;
    while (( i < cdesctblMax ) && ( (rgdesctbl[i].cbfld + cb + 2) < cbBuf )) {
        *pbBuf++ = rgdesctbl[i].idfld;
        *pbBuf++ = rgdesctbl[i].cbfld;
        memcpy(pbBuf, rgdesctbl[i].pb, rgdesctbl[i].cbfld);
        pbBuf += rgdesctbl[i].cbfld;
        cb += rgdesctbl[i].cbfld + 2;
        i++;
    }

    return cb;
}

/* ------------------------------------------------------------ */
/***	CfgGetPdid
**
**  Parameters:
**      none
**
**  Return Value:
**      the device's PDID
**
**  Errors:
**      none
**
**  Description:
**      This routine returns the device's PDID.
*/
DWORD
CfgGetPdid() {

    return pdidDevice;
}

/* ------------------------------------------------------------ */
/***	CfgGetAppFwver
**
**  Parameters:
**      none
**
**  Return Value:
**      application firmware version as currently stored in flash
**
**  Errors:
**      none
**
**  Description:
**      This routine returns firmware version of the application image in
**      flash.
*/
WORD
CfgGetAppFwver() {

    return _FWVERAPP;
}

/* ------------------------------------------------------------ */
/***	CfgGetBootFwver
**
**  Parameters:
**      none
**
**  Return Value:
**      application firmware version as currently stored in flash
**
**  Errors:
**      none
**
**  Description:
**      This routine returns firmware version of the bootloader image in
**      flash.
*/
WORD
CfgGetBootFwver() {

    return _FWVERBOOT;
}

/* ------------------------------------------------------------ */
/***	CfgGetDevnum
**
**  Parameters:
**      none
**
**  Return Value:
**      CAN bus device number assigned for this device
**
**  Errors:
**      none
**
**  Description:
**      This routine returns the CAN bus device number that should be assigned
**      to this device, as currently specified in the EEPROM (configuration)
**      section of flash memory.
*/
BYTE
CfgGetDevnum() {

    return pmtrcfgCur->devnum;
}

/* ------------------------------------------------------------ */
/***	CfgGetServoCal
**
**  Parameters:
**      psrvcalRead - pointer to struct to receive servo calibration constants
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine returns the servo calibration constants for this device as
**      they are previously set in flash memory.
*/
void
CfgGetServoCal(SERVOCAL* psrvcalRead) {

    if ( NULL != psrvcalRead ) {
        psrvcalRead->cntServoMin = pmtrcfgCur->servocal.cntServoMin;
        psrvcalRead->cntServoMax = pmtrcfgCur->servocal.cntServoMax;
        psrvcalRead->cntServoNeutralMin = pmtrcfgCur->servocal.cntServoNeutralMin;
        psrvcalRead->cntServoNeutralMax = pmtrcfgCur->servocal.cntServoNeutralMax;
    }
}

/* ------------------------------------------------------------ */
/***	CfgGetBrakeCoast
**
**  Parameters:
**      none
**
**  Return Value:
**      Brake / Coast setting for this device
**
**  Errors:
**      none
**
**  Description:
**      This routine returns the current Brake / Coast setting for this device.
**      If the device is configured to brake then fTrue will be returned. If the
**      the device is configured to coast then fFalse will be returned.
*/
BOOL
CfgGetBrakeCoast() {

    return pmtrcfgCur->fBrake;
}

/* ------------------------------------------------------------ */
/***	CfgGetSzDevName
**
**  Parameters:
**      none
**
**  Return Value:
**      pointer to NUL terminated string containing the Device Name
**
**  Errors:
**      none
**
**  Description:
**      This routine returns a NUL terminated string containing the device name
**      as specified in the current motor configuration, as read from the EEPROM
**      section of flash.
*/
const char*
CfgGetSzDevName() {

    return (const char*)(pmtrcfgCur->szDevName);
}

/* ------------------------------------------------------------ */
/***	CfgGetSzManName
**
**  Parameters:
**      none
**
**  Return Value:
**      pointer to NUL terminated string containing the Manufacturer Name
**      string descriptor
**
**  Errors:
**      none
**
**  Description:
**      This routine returns a NUL terminated string containing the
**      Manufacturer Name as specified in the string descriptor section of the
**      flash memory.
*/
const char*
CfgGetSzManName() {

    return (const char*)(&strdescCur.szManName);
}

/* ------------------------------------------------------------ */
/***	CfgGetSzProdName
**
**  Parameters:
**      none
**
**  Return Value:
**      pointer to NUL terminated string containing the Product Name
**      string descriptor
**
**  Errors:
**      none
**
**  Description:
**      This routine returns a NUL terminated string containing the
**      Product Name as specified in the string descriptor section of the
**      flash memory.
*/
const char*
CfgGetSzProdName() {

    return (const char*)(&strdescCur.szProdName);
}

/* ------------------------------------------------------------ */
/***	CfgGetSzManDate
**
**  Parameters:
**      none
**
**  Return Value:
**      pointer to NUL terminated string containing the Manufacturing Date
**      string descriptor
**
**  Errors:
**      none
**
**  Description:
**      This routine returns a NUL terminated string containing the
**      Manufacturing Date as specified in the string descriptor section of the
**      flash memory.
*/
const char*
CfgGetSzManDate() {

    return (const char*)(&strdescCur.szManDate);
}

/* ------------------------------------------------------------ */
/***	CfgGetSzHardwareVer
**
**  Parameters:
**      none
**
**  Return Value:
**      pointer to NUL terminated string containing the Hardware Version
**      string descriptor
**
**  Errors:
**      none
**
**  Description:
**      This routine returns a NUL terminated string containing the
**      Hardware Version as specified in the string descriptor section of the
**      flash memory.
*/
const char*
CfgGetSzHardwareVer() {

    return (const char*)(&strdescCur.szHardwareVer);
}

/* ------------------------------------------------------------ */
/***	CfgGetSzSN
**
**  Parameters:
**      none
**
**  Return Value:
**      pointer to NUL terminated string containing the Serial Number
**      string descriptor
**
**  Errors:
**      none
**
**  Description:
**      This routine returns a NUL terminated string containing the
**      Serial Number as specified in the string descriptor section of the
**      flash memory.
*/
const char*
CfgGetSzSN() {

    return (const char*)(&strdescCur.szSN);
}

/* ------------------------------------------------------------ */
/***	CfgSetDevnum
**
**  Parameters:
**      devnumSet   - CAN bus device number to set
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets the CAN bus device number associated with this device
**      in the RAM copy of the motor configuration and writes the motor
**      configuration to the EEPROM section of the flash memory.
**
**  Notes:
**      Interrupts are disabled while the flash is being written and enabled
**      once the operaiton has been completed.
*/
void
CfgSetDevnum(BYTE devnumSet) {

    DISABLE_INTERRUPTS
    pmtrcfgCur->devnum = devnumSet;
    CfgWriteEeprom();
    ENABLE_INTERRUPTS
}

/* ------------------------------------------------------------ */
/***	CfgSetServoCal
**
**  Parameters:
**      psrvcalWrite - pointer to struct containing servo calibration constants
**                     to be written to flash
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets the servo calibratoin constants in the RAM copy of the
**      motor configuration and writes the motor configuration to the EEPROM
**      section of the flash memory.
**
**  Notes:
**      Interrupts are disabled while the flash is being written and enabled
**      once the operaiton has been completed.
*/
void
CfgSetServoCal(SERVOCAL* psrvcalWrite) {

    if ( NULL != psrvcalWrite ) {
        DISABLE_INTERRUPTS
        pmtrcfgCur->servocal.cntServoMin = psrvcalWrite->cntServoMin;
        pmtrcfgCur->servocal.cntServoMax = psrvcalWrite->cntServoMax;
        pmtrcfgCur->servocal.cntServoNeutralMin = psrvcalWrite->cntServoNeutralMin;
        pmtrcfgCur->servocal.cntServoNeutralMax = psrvcalWrite->cntServoNeutralMax;
        CfgWriteEeprom();
        ENABLE_INTERRUPTS
    }
}

/* ------------------------------------------------------------ */
/***	CfgSetBrakeCoast
**
**  Parameters:
**      fBrake  - current brake / coast setting (fTrue = brake, fFalse = coast)
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets the brake / coast setting in the RAM copy of the
**      motor configuration and writes the motor configuration to the EEPROM
**      section of the flash memory.
**
**  Notes:
**      Interrupts are disabled while the flash is being written and enabled
**      once the operaiton has been completed.
*/
void
CfgSetBrakeCoast(BOOL fBrake) {

    DISABLE_INTERRUPTS
    pmtrcfgCur->fBrake = fBrake;
    CfgWriteEeprom();
    ENABLE_INTERRUPTS
}

/* ------------------------------------------------------------ */
/***	CfgSetSzDevName
**
**  Parameters:
**      szDevName   - zero terminated device name string
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets the Device Name string in the RAM copy of the motor
**      configuration and writes the motor configuration to the EEPROM section
**      of the flash memory.
**
**  Notes:
**      Interrupts are disabled while the flash is being written and enabled
**      once the operaiton has been completed.
*/
void
CfgSetSzDevName(const char* szDevName) {

    DISABLE_INTERRUPTS
    strncpy(pmtrcfgCur->szDevName, szDevName, cchDevNameMax);
    pmtrcfgCur->szDevName[cchDevNameMax] = '\0';
    CfgWriteEeprom();
    rgdesctbl[idxSzDevName].cbfld = strlen(pmtrcfgCur->szDevName);
    ENABLE_INTERRUPTS
}

/* ------------------------------------------------------------ */
/***	CfgSetSzManName
**
**  Parameters:
**      szManName   - zero terminated manufacturer name string
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets the Manufacturer Name string in the RAM copy of the
**      string descriptors and then writes the string descriptor table to the
**      string descriptor section of the flash memory.
**
**  Notes:
**      Interrupts are disabled while the flash is being written and enabled
**      once the operation has been completed.
*/
void
CfgSetSzManName(const char* szManName) {

    DISABLE_INTERRUPTS
    strncpy(strdescCur.szManName, szManName, cchManNameMax);
    strdescCur.szManName[cchManNameMax] = '\0';
    CfgWriteStrDescToFlash();
    rgdesctbl[idxSzManName].cbfld = strlen(strdescCur.szManName);
    ENABLE_INTERRUPTS
}

/* ------------------------------------------------------------ */
/***	CfgSetSzProdName
**
**  Parameters:
**      szProdName   - zero terminated product name string
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets the Product Name string in the RAM copy of the
**      string descriptors and then writes the string descriptor table to the
**      string descriptor section of the flash memory.
**
**  Notes:
**      Interrupts are disabled while the flash is being written and enabled
**      once the operation has been completed.
*/
void
CfgSetSzProdName(const char* szProdName) {

    DISABLE_INTERRUPTS
    strncpy(strdescCur.szProdName, szProdName, cchProdNameMax);
    strdescCur.szProdName[cchProdNameMax] = '\0';
    CfgWriteStrDescToFlash();
    rgdesctbl[idxSzProdName].cbfld = strlen(strdescCur.szProdName);
    ENABLE_INTERRUPTS
}

/* ------------------------------------------------------------ */
/***	CfgSetSzManDate
**
**  Parameters:
**      szManDate   - zero terminated manufacturing date string
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets the Manufacturing Date string in the RAM copy of the
**      string descriptors and then writes the string descriptor table to the
**      string descriptor section of the flash memory.
**
**  Notes:
**      Interrupts are disabled while the flash is being written and enabled
**      once the operation has been completed.
*/
void
CfgSetSzManDate(const char* szManDate) {

    DISABLE_INTERRUPTS
    strncpy(strdescCur.szManDate, szManDate, cchManDateMax);
    strdescCur.szManDate[cchManDateMax] = '\0';
    CfgWriteStrDescToFlash();
    rgdesctbl[idxSzManDate].cbfld = strlen(strdescCur.szManDate);
    ENABLE_INTERRUPTS
}

/* ------------------------------------------------------------ */
/***	CfgSetSzHardwareVer
**
**  Parameters:
**      szHardwareVer   - zero terminated hardware version string
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets the Hardware Version string in the RAM copy of the
**      string descriptors and then writes the string descriptor table to the
**      string descriptor section of the flash memory.
**
**  Notes:
**      Interrupts are disabled while the flash is being written and enabled
**      once the operation has been completed.
*/
void
CfgSetSzHardwareVer(const char* szHardwareVer) {

    DISABLE_INTERRUPTS
    strncpy(strdescCur.szHardwareVer, szHardwareVer, cchHardwareVerMax);
    strdescCur.szHardwareVer[cchHardwareVerMax] = '\0';
    CfgWriteStrDescToFlash();
    rgdesctbl[idxSzHardwareVer].cbfld = strlen(strdescCur.szHardwareVer);
    ENABLE_INTERRUPTS
}

/* ------------------------------------------------------------ */
/***	CfgSetSzSN
**
**  Parameters:
**      szSN    - zero terminated serial number string
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets the Serial Number string in the RAM copy of the
**      string descriptors and then writes the string descriptor table to the
**      string descriptor section of the flash memory.
**
**  Notes:
**      Interrupts are disabled while the flash is being written and enabled
**      once the operation has been completed.
*/
void
CfgSetSzSN(const char* szSN) {

    DISABLE_INTERRUPTS
    strncpy(strdescCur.szSN, szSN, cchSNMax);
    strdescCur.szSN[cchSNMax] = '\0';
    CfgWriteStrDescToFlash();
    rgdesctbl[idxSzSN].cbfld = strlen(strdescCur.szSN);
    ENABLE_INTERRUPTS
}

/******************************************************************************/
