/************************************************************************/
/*									*/
/*  Cfg.h - Motor Controller Flash Configuration Functions              */
/*									*/
/************************************************************************/
/*  Author: Michael T. Alexander					*/
/*  Copyright 2016, Digilent Inc.					*/
/************************************************************************/
/*  Module Description: 						*/
/*									*/
/*  This module contains declarations of routines that can be used to	*/
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

#if !defined(_CFG_INC)
#define	_CFG_INC

#include "stdtypes.h"

/* ------------------------------------------------------------ */
/*                  Miscellaneous Declarations			*/
/* ------------------------------------------------------------ */

/* Define the character length for all string descriptors.
*/
#define cchManNameMax       28
#define cchProdNameMax      28
#define cchManDateMax       20
#define cchHardwareVerMax   8
#define cchSNMax            12
#define cchDevNameMax       64

/* Define the string descriptors used for default initialization
** when the string descriptor flash section is invalid.
*/
#define szManNameDefault        "Digilent"

#if defined(DMC60)
#define szProdNameDefault       "DMC60"
#elif defined(DMC60C)
#define szProdNameDefault       "DMC60C"
#endif

#define szManDateDefault        "UNKNOWN"
#define szHardwareVerDefault    "UNKNOWN"
#define szSNDefault             "UNKNOWN"

/* Define the field identifiers for the descriptor table.
*/
#define idfldNone           0
#define idfldSzDevName      1
#define idfldSzManName      2
#define idfldSzProdName     3
#define idfldSzManDate      4
#define idfldSzHardWareVer  5
#define idfldSzSN           6

/* Define the index in the descriptor table for the various
** strings.
*/
#define idxSzDevName        0
#define idxSzManName        1
#define idxSzProdName       2
#define idxSzManDate        3
#define idxSzHardwareVer    4
#define idxSzSN             5

/* Define the descriptor table size. Please note that this is
** based on the number of strings included in the descriptor
** table.
*/
#define cdesctblMax         idxSzSN + 1

/* ------------------------------------------------------------ */
/*                  General Type Declarations			*/
/* ------------------------------------------------------------ */

#pragma pack(push, 2)
typedef struct {
    WORD    mgk1;
    WORD    mgk2;
    WORD    ver;
    char    szManName[cchManNameMax + 1];
    char    szProdName[cchProdNameMax + 1];
    char    szManDate[cchManDateMax + 1];
    char    szHardwareVer[cchHardwareVerMax + 1];
    char    szSN[cchSNMax + 1];
} STRDESC;
#pragma pack(pop)

#define verStrDescCur   0
#define verStrDescApp   verStrDescCur

#pragma pack(push, 2)
typedef struct {
    /* Version 0 fields start here. */
    WORD        mgk1;
    WORD        mgk2;
    WORD        ver;
    BYTE        devnum; // CAN DEVICE NUMBER
    char        szDevName[cchDevNameMax + 1];
    /* Version 1 fields start here. */
    SERVOCAL    servocal;
    BOOL        fBrake;
} MTRCFG;
#pragma pack(pop)

#define verMtrcfgCur    1
#define verMtrCfgApp    verMtrcfgCur

typedef struct {
    BYTE        idfld;
    BYTE        cbfld;
    const BYTE* pb;
} DESCTBLENTRY;

/* ------------------------------------------------------------ */
/*                  Variable Declarations			*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*                  Procedure Declarations			*/
/* ------------------------------------------------------------ */

void            CfgInit();

void            CfgWriteEeprom();
void            CfgWriteStrDescToFlash();

WORD            CfgGetDescTable(BYTE* pbBuf, WORD cbBuf);
DWORD           CfgGetPdid();
WORD            CfgGetAppFwver();
WORD            CfgGetBootFwver();

BYTE            CfgGetDevnum();
void            CfgGetServoCal(SERVOCAL* psrvcalRead);
BOOL            CfgGetBrakeCoast();

const char*     CfgGetSzDevName();
const char*     CfgGetSzManName();
const char*     CfgGetSzProdName();
const char*     CfgGetSzManDate();
const char*     CfgGetSzHardwareVer();
const char*     CfgGetSzSN();

void            CfgSetDevnum(BYTE devnumSet);
void            CfgSetServoCal(SERVOCAL* psrvcalWrite);
void            CfgSetBrakeCoast(BOOL fBrake);

void            CfgSetSzDevName(const char* szDevName);
void            CfgSetSzManName(const char* szManName);
void            CfgSetSzProdName(const char* szProdName);
void            CfgSetSzManDate(const char* szManDate);
void            CfgSetSzHardwareVer(const char* szHardwareVer);
void            CfgSetSzSN(const char* szSN);

/* ------------------------------------------------------------ */

#endif
