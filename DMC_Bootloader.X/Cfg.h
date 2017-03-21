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
/*  09/16/2016 (MichaelA): created			                */
/*  09/20/2016 (MichaelA): added CfgGetAppFwver() and CfgGetBootFwver() */
/*      functions                                                       */
/*									*/
/************************************************************************/

#if !defined(_CFG_INC)
#define	_CFG_INC

#include "stdtypes.h"

/* ------------------------------------------------------------ */
/*                  Miscellaneous Declarations			*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*                  General Type Declarations			*/
/* ------------------------------------------------------------ */

#define cchManNameMax       28
#define cchProdNameMax      28
#define cchManDateMax       20
#define cchHardwareVerMax   8
#define cchSNMax            12
#define cchDevNameMax       64

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

#define verStrDescCur           0
#define verStrDescBootloader    verStrDescCur

#pragma pack(push, 2)
typedef struct {
    WORD    mgk1;
    WORD    mgk2;
    WORD    ver;
    BYTE    devnum; // CAN DEVICE NUMBER
    char    szDevName[cchDevNameMax + 1];
} MTRCFG;
#pragma pack(pop)

#define verMtrcfgCur        0 // bootloader
#define verMtrCfgBootloader verMtrcfgCur

/* Define the string descriptors used for default initialization when the string
** descriptor flash section is invalid.
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


#define idfldNone           0
#define idfldSzDevName      1
#define idfldSzManName      2
#define idfldSzProdName     3
#define idfldSzManDate      4
#define idfldSzHardWareVer  5
#define idfldSzSN           6

#define idxSzDevName        0
#define idxSzManName        1
#define idxSzProdName       2
#define idxSzManDate        3
#define idxSzHardwareVer    4
#define idxSzSN             5

#define cdesctblMax         idxSzSN + 1

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
void            CfgTerm();

WORD            CfgGetDescTable(BYTE* pbBuf, WORD cbBuf);

DWORD           CfgGetPdid();
WORD            CfgGetAppFwver();
WORD            CfgGetBootFwver();

BYTE            CfgGetDevnum();
void            CfgSetDevnum(BYTE devnumSet);

const char*     CfgGetSzDevName();
void            CfgSetSzDevName(const char* szDevName);

void            CfgWriteEeprom();

const char*     CfgGetSzManName();
const char*     CfgGetSzProdName();
const char*     CfgGetSzManDate();
const char*     CfgGetSzHardwareVer();
const char*     CfgGetSzSN();

void            CfgSetSzManName(const char* szManName);
void            CfgSetSzProdName(const char* szProdName);
void            CfgSetSzManDate(const char* szManDate);
void            CfgSetSzHardwareVer(const char* szHardwareVer);
void            CfgSetSzSN(const char* szSN);

void            CfgReadStrDescFromFlash();
void            CfgWriteStrDescToFlash();

/* ------------------------------------------------------------ */


#endif
