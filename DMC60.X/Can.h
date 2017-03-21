/************************************************************************/
/*									*/
/*  Can.h - Motor Controller CAN Interface Functions                    */
/*									*/
/************************************************************************/
/*  Author: Michael T. Alexander					*/
/*  Copyright 2016, Digilent Inc.					*/
/************************************************************************/
/*  Module Description: 						*/
/*									*/
/*  This module contains declarations of routines that are uesd to      */
/*  implement the CAN interface.                                        */
/*									*/
/************************************************************************/
/*  Revision History:						        */
/*									*/
/*  08/23/2016 (MichaelA): created			                */
/*  10/14/2016 (MichaelA): added CanCmdGetFwver()                       */
/*									*/
/************************************************************************/

#if !defined(_CAN_INC)
#define	_CAN_INC

#include "stdtypes.h"

/* ------------------------------------------------------------ */
/*                  Miscellaneous Declarations			*/
/* ------------------------------------------------------------ */

/* Define the default device number used for identifying this device on the
** CAN bus.
*/
#define devnumDefault           0
#define devnumMax               62
#define devnumBroadcast         63

/* Define the default and maximum number of ticks required between periodic
** status frames.
*/
#define ctickStsAnalogDefault   200 // 100ms
#define ctickStsAnalogMax       60000 // 30 seconds

/* Define the maximum number of CAN messages that can be processeed during a
** single call to the interrupt service routine.
*/
#define ccmsgProcMax            4

/* Define macro used to convert milliseconds to ticks.
*/
#define CanMsToTicks(tms)   (tms << 1)

/* Define ECAN errors for use with ECAN Interrupt Code Register.
*/
#define CAN_TRB0_ICODE          0
#define CAN_TRB1_ICODE          1
#define CAN_TRB2_ICODE          2
#define CAN_TRB3_ICODE          3
#define CAN_TRB4_ICODE          4
#define CAN_TRB5_ICODE          5
#define CAN_TRB6_ICODE          6
#define CAN_TRB7_ICODE          7
#define CAN_RB8_ICODE           8
#define CAN_RB9_ICODE           9
#define CAN_RB10_ICODE          10
#define CAN_RB11_ICODE          11
#define CAN_RB12_ICODE          12
#define CAN_RB13_ICODE          13
#define CAN_RB14_ICODE          14
#define CAN_RB15_ICODE          15
#define CAN_RB16_ICODE          16
#define CAN_RB17_ICODE          17
#define CAN_RB18_ICODE          18
#define CAN_RB19_ICODE          19
#define CAN_RB20_ICODE          20
#define CAN_RB21_ICODE          21
#define CAN_RB22_ICODE          22
#define CAN_RB23_ICODE          23
#define CAN_RB24_ICODE          24
#define CAN_RB25_ICODE          25
#define CAN_RB26_ICODE          26
#define CAN_RB27_ICODE          27
#define CAN_RB28_ICODE          28
#define CAN_RB29_ICODE          29
#define CAN_RB30_ICODE          30
#define CAN_RB31_ICODE          31
#define CAN_ERROR_ICODE         65
#define CAN_WAKE_ICODE          66
#define CAN_RXOVF_ICODE         67
#define CAN_FIFO_AFULL_ICODE    68

/* Define the index of the TX status frames within the CAN message
** buffer.
*/
#define icodeTxEnum         CAN_TRB0_ICODE
#define idxTxEnum           0
#define txreqEnum           C1TR01CONbits.TXREQ0

#define icodeVendorDin      CAN_TRB1_ICODE
#define idxVendorDin        1
#define txreqVendorDin      C1TR01CONbits.TXREQ1

#define icodeStsAnalog      CAN_TRB3_ICODE
#define idxStsAnalog        3
#define txreqStsAnalog      C1TR23CONbits.TXREQ3

#define icodeVendorSts      CAN_TRB7_ICODE
#define idxVendorSts        7
#define txreqVendorSts      C1TR67CONbits.TXREQ7

/* Define index/offsets used to access the fields of a status
** frame within the data field of a CAN message buffer.
*/
#define ibStsAnalogIn	0
#define ibStsCurrent    2
#define ibStsTemp	4
#define ibStsVoltage	6

/* Define masks for the CAN bus message ID acceptance filters.
*/
#define mskDigilentMtrCtrlr 0xFFEB0000 // match device type and manufacturer
#define mskBroadcast        0x00000000 // allow everything

/* Define the message ID's used for the acceptance filters.
*/
#define msgidDigilent   0x02060000 // device type = motor controller, manufacturer = Digilent
#define msgidSysCtrl    0x00000000 // dev type and manufacturer must be 0 here

/* Define the messages ID's used for receiving and transmitting CAN frames.
*/
#define msgidControl0   0x02060000
#define msgidStsAnalog  0x020614C0

/* Define message ID's used for parsing received frames.
*/
#define msgidEnumRcv    0x00000240

/* Define message ID's used for transmitting frames.
*/
#define msgidEnumSnd    0x0206F000 // API = 0x3C0
#define msgidDescSnd    0x0206F040 // API = 0x3C1

#define msgidVendorCmd  0x0206FC00 // API = 0x3F0
#define msgidVendorDout 0x0206FC40 // API = 0x3F1
#define msgidVendorDin  0x0206FC80 // API = 0x3F2
#define msgidVendorSts  0x0206FCC0 // API = 0x3F3

/* Define the API's that may be specified within a CAN message ID. Please note
** that the API is specified in bits 15:6 of the EID.
*/
#define apiControl0     0x0000
#define apiSysEnum      0x0009

#define apiVendorCmd    0x03F0
#define apiVendorDout   0x03F1
#define apiVendorDin    0x03F2
#define apiVendorSts    0x03F3

/* Define size limits for the DTO and DTI buffers.
*/
#define cbDtoMax        4100
#define cbDtiMax        256

/* Define the number of bytes expected for the different CAN frames that we may
** receive.
*/
#define cbControl0      8

/* Define the different modes that can be set by Control 0.
*/
#define modeVoltage     0
#define modeNoDrive     15

/* ------------------------------------------------------------ */
/*                  General Type Declarations			*/
/* ------------------------------------------------------------ */

/* Define a data structure to keep track of flags that represent the
** state of the CAN interface.
*/
typedef struct {
    union {
        struct {
            unsigned fBusDetected:1;
            unsigned fEnabled:1;
            unsigned fTxEnumResp:1;
            unsigned :13;
        };
        WORD    fsFlags;
    };
} CANFLAGS;

/* Define a data structure to represent a CAN message that can be stored
** in RAM and in the CAN module buffers.
*/
typedef struct {
    union {
        struct {
            unsigned IDE:1;
            unsigned SRR:1;
            unsigned SID:11;
            unsigned :3;
        };
        WORD    w0;
    };
    union {
        struct {
            unsigned EIDh:12;
            unsigned :4;
        };
        WORD    w1;
    };
    union {
        struct {
            unsigned DLC:4;
            unsigned RB0:1;
            unsigned :3;
            unsigned RB1:1;
            unsigned RTR:1;
            unsigned EIDl:6;
        };
        WORD    w2;
    };
    union {
        BYTE    rgbData[8];
        struct {
            WORD    w3;
            WORD    w4;
            WORD    w5;
            WORD    w6;
        };
    };
    union {
        struct {
            unsigned :8;
            unsigned FILHIT:5;
            unsigned :3;
        };
        WORD    w7;
    };
} CANMSG;

/* Define a data structure to represent CAN Control Frame 0.
*/
typedef struct {
    unsigned    modeSelect : 4;
    unsigned    fOverrideBC : 1;
    unsigned    fBrakeCoast : 1;
    unsigned    rsv0 : 2;
    unsigned    rsv1 : 8;
    unsigned    rsv2 : 8;
    unsigned    rsv3 : 8;
    int16_t     vltgSet;
    WORD        vltgRampSet;
} CANCTRL0;

/* Define data structures to represent the frames returned in response to an
** enumeration request.
*/
typedef struct {
    WORD    sessid;
    DWORD   pdid;
} ENUMRSP0;

typedef struct {
    WORD    sessid;
    union {
        struct {
            unsigned imgtyp:2;
            unsigned rsv:14;
        };
        WORD flgs;
    };
    WORD    fwverApp;
    WORD    fwverBoot;
} ENUMRSP1;

/* Define a data structure used to represent the frame returned in response to
** a get firmware verison request.
*/
typedef struct {
    union {
        struct {
            unsigned imgtyp:2;
            unsigned rsv:14;
        };
        WORD flgs;
    };
    WORD    fwverApp;
    WORD    fwverBoot;
} FWVERRSP;

/* Define data structures used to represent Vendor Command and Status Packets.
*/
typedef struct {
    WORD    sessid;
    WORD    cmd;
    WORD    PARAM1;
    WORD    PARAM2;
} VENDORCMD;

typedef struct {
    WORD    cerc;
    WORD    cb;
} VENDORSTS;

/* Define Vendor Commands.
*/
#define vcmdNoCommand           0x00

#define vcmdSetDevNumber        0x01
#define vcmdSetDevName          0x02
#define vcmdSetManName          0x03
#define vcmdSetProdName         0x04
#define vcmdSetManDate          0x05
#define vcmdSetHardwareVer      0x06
#define vcmdSetSN               0x07

#define vcmdGetDescriptors      0x60
#define vcmdGetFwver            0x61

#define vcmdEnterBootloader     0xF0
#define vcmdSoftReset           0xF1
#define vcmdEraseWriteFlashPage 0xF2


/* Define CAN Bus error codes that are returned in status packets.
*/
#define cercNoError             0x00
#define cercNotSupported        0x01
#define cercBadParameter        0x02
#define cercDataRcvMore         0x03
#define cercInBootloader        0x04
#define cercCrcMismatch         0x05
#define cercFlashWriteFailed    0x06
#define cercAckReset            0x07

/* ------------------------------------------------------------ */
/*                  Variable Declarations			*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*                  Procedure Declarations			*/
/* ------------------------------------------------------------ */

void    CanInterruptHandler();
void    CanBusDetected();
void    CanAckSoftReset();
void    CanPeriodicLoop();
void    CanInit();
void    CanTimeout();

void    CanXfrDataOut(CANMSG* pcmsg);

void    CanDispatchVendorCmd(CANMSG* pcmsg);
void    CanCmdSetDevNumber(VENDORCMD* pvcmd);
void    CanCmdSetDevName(VENDORCMD* pvcmd);
void    CanCmdSetManName(VENDORCMD* pvcmd);
void    CanCmdSetProdName(VENDORCMD* pvcmd);
void    CanCmdSetManDate(VENDORCMD* pvcmd);
void    CanCmdSetHardwareVer(VENDORCMD* pvcmd);
void    CanCmdSetSN(VENDORCMD* pvcmd);

void    CanCmdGetDescriptors(VENDORCMD* pvcmd);
void    CanCmdGetFwver(VENDORCMD* pvcmd);

/* ------------------------------------------------------------ */


#endif