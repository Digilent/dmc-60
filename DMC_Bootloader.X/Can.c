/************************************************************************/
/*									*/
/*  Can.c - Motor Controller Servo CAN Interface Functions              */
/*									*/
/************************************************************************/
/*  Author: Michael T. Alexander					*/
/*  Copyright 2016, Digilent Inc.					*/
/************************************************************************/
/*  Module Description: 						*/
/*									*/
/*  This module contains definition of routines that are uesd to        */
/*  implement the CAN interface.                                        */
/*									*/
/************************************************************************/
/*  Revision History:						        */
/*									*/
/*  08/23/2016 (MichaelA): created			                */
/*  10/07/2016 (MichaelA): finished implementation of firmware download */
/*  10/10/2016 (MichaelA): added readback verification of flash that    */
/*      was written as part of a firmware download                      */
/*  10/11/2016 (MichaelA): modified CanCmdSetDevNumber() and            */
/*      CanXfrDataOut() to LedDispProgFlash() when a flash write has    */
/*      successfully completed                                          */
/*  10/14/2016 (MichaelA): modified vendor command dispatcher such that */
/*      the current Session ID is saved in persistent memory prior to   */
/*      performing a soft reset. This allows the host to continue to    */
/*      send vendor commands to the device with the same Session ID     */
/*      after a reset has occured, which removes the need to            */
/*      re-enumerate the bus                                            */
/*									*/
/************************************************************************/

/* ------------------------------------------------------------ */
/*		Include File Definitions			*/
/* ------------------------------------------------------------ */

#include <xc.h>
#include <string.h>
#include <libpic30.h>
#include "DMC.h"
#include "stdtypes.h"
#include "prodcfg.h"
#include "Led.h"
#include "Adc.h"
#include "Can.h"
#include "Flash.h"
#include "Cfg.h"
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

/* Declare variable used to store the CAN message buffer.
*/
#define ccmsgBufMax 32
static CANMSG rgcmsgBuf[ccmsgBufMax] __attribute__((aligned(ccmsgBufMax * 16)));

/* Declare variable used to keep track of the CAN interface flags.
*/
static volatile CANFLAGS    canflgs;

/* Declare variable used to keep track of this device's CAN Device Number.
*/
static volatile BYTE        devnumCur;

/* Declare variable used to keep track of this device's CAN Session ID.
*/
static volatile WORD        sessidCur;

/* Declare variables used to keep track of ticks before responding to commands
** that require delays.
*/
static volatile WORD        ctickEnumResp;

/* Define variabls used for the DTO and DTI buffers. These buffers are used to
** receive and transmit large chunks of data in respone to Vendor Commands
** received over the CAN interface.
*/
static volatile BYTE        rgbDtoBuf[cbDtoMax];
static volatile WORD        cbDtoCur;
static volatile WORD        cbDtoMac;

static volatile BYTE        rgbDtiBuf[cbDtiMax];
static volatile WORD        cbDtiCur;
static volatile WORD        cbDtiMac;

static volatile WORD        vcmdTransCur;

/* Define variables used to keep track of the information that's
** required to perform flash page writes during firmware updates.
*/
static volatile WORD        ipageWrite;
static volatile WORD        imgtypWrite;
static volatile WORD        fwverWrite;
static volatile DWORD       rgdwLastPage[2];

/* ------------------------------------------------------------ */
/*                  Macros                                      */
/* ------------------------------------------------------------ */

/* Define macros that can be used to disable and enable interrupts.
*/
#define DISABLE_INTERRUPTS \
        INTCON2bits.GIE = 0;

#define ENABLE_INTERRUPTS \
        INTCON2bits.GIE = 1;

/* ------------------------------------------------------------ */
/*		Forward Declarations				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Interrupt Service Routines	            	*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Procedure Definitions				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/***	CanInterruptHandler
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
**      This routine serves as the interrupt handler for CAN
**      interrupts. It deals with transmitting and receiving CAN messages,
**      and also handles bus errors.
**
**  Notes:
**      This routine does NOT have to be called from an actual interrupt
**      service routine. However, it should only be called when the CAN
**      interrupt flag is set. It is the caller's responsibility to clear the
**      main CAN interrupt flag.
*/
void
CanInterruptHandler() {

    CANMSG  cmsg;
    WORD    ccmsg;
    WORD    cbTrans;

    if (( CAN_TRB0_ICODE <= C1VECbits.ICODE ) && ( CAN_TRB7_ICODE >= C1VECbits.ICODE )) {

        switch ( C1VECbits.ICODE ) {
            case icodeTxEnum:
                if ( canflgs.fTxEnumResp ) {
                    /* Transmitted the first response frame. Populate the second
                    ** respone frame and transmit it. Note: we won't set the
                    ** device number because we want to transmit the second
                    ** response with the same device number used to transmit the
                    ** first response, regardless of whether or not our number
                    ** has changed.
                    */
                    rgcmsgBuf[idxTxEnum].DLC = sizeof(ENUMRSP1);
                    ((ENUMRSP1*)(&(rgcmsgBuf[idxTxEnum].rgbData[0])))->sessid = sessidCur;
                    ((ENUMRSP1*)(&(rgcmsgBuf[idxTxEnum].rgbData[0])))->imgtyp = imgtypCur;
                    ((ENUMRSP1*)(&(rgcmsgBuf[idxTxEnum].rgbData[0])))->rsv = 0;
                    ((ENUMRSP1*)(&(rgcmsgBuf[idxTxEnum].rgbData[0])))->fwverApp = CfgGetAppFwver();
                    ((ENUMRSP1*)(&(rgcmsgBuf[idxTxEnum].rgbData[0])))->fwverBoot = CfgGetBootFwver();
                    txreqEnum = 1;
                    canflgs.fTxEnumResp = 0;
                }
                break;

            case icodeVendorDin:
                /* Finish transferring a Data IN frame. Check to see if we need
                ** to transmit another frame, and if so, queue it up.
                */
                if ( cbDtiCur < cbDtiMac ) {
                    /* Determine how many bytes to transmit this time.
                    */
                    cbTrans = 8;
                    if ( (cbDtiCur + cbTrans) > cbDtiMac ) {
                        cbTrans = cbDtiMac - cbDtiCur;
                    }

                    /* Copy the next chunk of data to the message buffer,
                    ** increment the count of bytes transferred, and mark the
                    ** buffer for transmission.
                    */
                    memcpy(rgcmsgBuf[idxVendorDin].rgbData, ((BYTE*)rgbDtiBuf) + cbDtiCur, cbTrans);
                    rgcmsgBuf[idxVendorDin].DLC = cbTrans;
                    rgcmsgBuf[idxVendorDin].EIDl = devnumCur;
                    cbDtiCur += cbTrans;
                    txreqVendorDin = 1;
                }
                break;
            default:
                break;
        }

        C1INTFbits.TBIF = 0;
    }
    else if (( CAN_RB8_ICODE <= C1VECbits.ICODE ) && ( CAN_RB31_ICODE >= C1VECbits.ICODE )) {
        /* Receive buffer interrupt. We assume that no overflow occured because
        ** had an overflow occured we should have received an error code that
        ** corresponds to the overflow condition.
        */

        CanBusDetected();

        /* Process messages received in the FIFO buffer. The number of messages
        ** processed per pass to the interrupt service routine should be limited
        ** so that other interrupts can execute in a timely manner.
        */
        ccmsg = 0;
        do {
            /* Copy the message from the FIFO and advance the read pointer.
            */
            memcpy(&cmsg, &(rgcmsgBuf[C1FIFObits.FNRB]), sizeof(CANMSG));
            ccmsg++;
            if ( 15 >= C1FIFObits.FNRB ) {
                    C1RXFUL1 = ~(1 << C1FIFObits.FNRB);
            }
            else {
                    C1RXFUL2 = ~(1 << (C1FIFObits.FNRB - 16));
            }

            /* If we received any messages that only contain the standard ID
            ** then we should discard them and move on to the next message.
            ** Note: this can only happen while using acceptance mask 1.
            */
            if ( 0 == cmsg.IDE ) {
                continue;
            }

            if ( 0 == cmsg.FILHIT ) {
                /* This is a message targeting a Digilent motor controller. The
                ** lower 6-bits of the EID contain the device number. Check to
                ** see if the device number matches, and if so, look for an API
                ** match.
                */
                if ( cmsg.EIDl != devnumCur ) {
                    continue;
                }

                switch ( cmsg.EIDh & 0x03FF ) {
                    case apiVendorCmd:
                        CanDispatchVendorCmd(&cmsg);
                        break;
                        
                    case apiVendorDout:
                        CanXfrDataOut(&cmsg);
                        break;

                    default:
                        break;
                }
            }
            else {
                /* Assume that this message matched the system control API
                ** because we only have two filters enabled.
                */
                switch ( cmsg.EIDh & 0x03FF ) {
                    case apiSysEnum:
                        /* Enumeration request received.*/
                        
                        /* If we are still in the process of transmitting a
                        ** response to a previous enumeration request then don't
                        ** transmit a new one.
                        */
                        if (( canflgs.fTxEnumResp ) || ( 1 == txreqEnum )) {
                            continue;
                        }
                        
                        /* Populate the appropriate message buffer with a
                        ** response.
                        */
                        rgcmsgBuf[idxTxEnum].EIDl = devnumCur & 0x3F;
                        rgcmsgBuf[idxTxEnum].DLC = sizeof(ENUMRSP0);
                        ((ENUMRSP0*)(&(rgcmsgBuf[idxTxEnum].rgbData[0])))->sessid = sessidCur;
                        ((ENUMRSP0*)(&(rgcmsgBuf[idxTxEnum].rgbData[0])))->pdid = CfgGetPdid();
                        
                        /* A delay is required before transmitting the response,
                        ** as all devices on the bus that receive an enumeration
                        ** request will respond to it. If our current device
                        ** address is the default device address then generate a
                        ** random delay between 0.5ms and 63.5ms using Timer 3.
                        */
                        if ( 0 == devnumCur ) {
                            ctickEnumResp = TMR3 & 0x007F;
                            if ( 0 == ctickEnumResp ) {
                                ctickEnumResp = 1;
                            }
                        }
                        else {
                            ctickEnumResp = CanMsToTicks(devnumCur);
                        }

                        canflgs.fTxEnumResp = 1;
                        
                        break;
                    default:
                        break;
                }
            }
        }
        while (( C1FIFObits.FNRB < C1FIFObits.FBP ) && ( ccmsg < ccmsgProcMax ));

        if ( C1FIFObits.FNRB >= C1FIFObits.FBP ) {
            C1INTFbits.RBIF = 0;
        }
    }
    else if ( CAN_ERROR_ICODE == C1VECbits.ICODE ) {

        /* This may be a TX or RX warning. A warning can be caused by loss of
        ** arbitration.
        */
        if ( 0 == (C1INTF & 0x3800) ) {
            C1INTFbits.IVRIF = 0;
            C1INTFbits.ERRIF = 0;
            return;
        }

        /* An error occured. Assume that the CAN link was lost, abort any
        ** pending transfers, and end the session.
        */
        C1TR01CONbits.TXREQ0 = 0;
        C1TR01CONbits.TXREQ1 = 0;
        C1TR23CONbits.TXREQ2 = 0;
        C1TR23CONbits.TXREQ3 = 0;
        C1TR45CONbits.TXREQ4 = 0;
        C1TR45CONbits.TXREQ5 = 0;
        C1TR67CONbits.TXREQ6 = 0;
        C1TR67CONbits.TXREQ7 = 0;

        canflgs.fBusDetected = 0;
        canflgs.fTxEnumResp = 0;

        /* Since the CAN bus link was lost we need to change the mask for Filter
        ** 1 to Mask 1 so that any and all messages transmitted on the CAN bus
        ** will be received, which will allow us to detect the presence of the
        ** bus should the link be re-established.
        */
        C1FMSKSEL1bits.F1MSK = 0b01; // use mask 1

        /* Clear the error interrupt flags.
        */
        C1INTFbits.IVRIF = 0;
        C1INTFbits.ERRIF = 0;
    }
    else if ( CAN_RXOVF_ICODE == C1VECbits.ICODE ) {
        /* A receive buffer overflowed. M00TODO: figure out what action to take
        ** when this happens. If we only store messages in a FIFO and always
        ** consume the entire FIFO then this may never happen.
        */
        C1INTFbits.RBOVIF = 0;
    }
    else {
        /* Some other unhandled interrupt. We should never get here unless we
        ** enabled an interrupt source that we are presently testing for.
        */
        C1INTFbits.FIFOIF = 0;
        C1INTFbits.WAKIF = 0;
    }
}

/* ------------------------------------------------------------ */
/***	CanBusDetected
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
**      If the bus detected flag isn't already set then it's set, a Session ID
**      is generated,  and any other required CAN variables are initialized.
*/
void
CanBusDetected() {

    /* Check to see if the bus was previously detected.
    */
    if ( 0 == canflgs.fBusDetected ) {
        canflgs.fBusDetected = 1;

        /* If we ended up in the bootloader as the result of the
        ** vcmdEnterBootloader then not only is the host expecting a status
        ** frame from us, but it also expects this device to have the same
        ** Session ID. Therefore, the application firmware saves it's Session ID
        ** in sessidPrev. If this Session ID is set the we should use it instead
        ** of generating a new one.
        */
        if ( 0 < sessidPrev ) {
            sessidCur = sessidPrev;
            sessidPrev = 0; // generate new Session ID next time
        }
        else {
            sessidCur = TMR3;
            if ( 0 == sessidCur ) {
                sessidCur = 1;
            }
        }

        /* Now that the CAN bus has been detected change the mask for
        ** acceptance Filter 1 to mask 0 so that Filter 1 only accepts
        ** System Control Messages instead of all messages. This will reduce
        ** the number of interrupts that we have to process and the number
        ** of messages we have to ignore.
        */
        C1FMSKSEL1bits.F1MSK = 0b00; // use mask 0
    }
}

/* ------------------------------------------------------------ */
/***	CanInit
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
**      This function initializes the CAN Interface.
*/
void
CanInit() {

    /* Disable CAN interrupts.
    */
    IEC2bits.C1IE = 0;

    /* Configure the CAN RX pin as an input.
    */
    trsCanRx |= (1 << bnCanRx);

    /* Configure the CAN TX pin as an output and drive it to the recessive state
    ** (high).
    */
    latCanTx |= (1 << bnCanTx);
    trsCanTx &= ~(1 << bnCanTx);

    /* Place the CAN module in configuraiton mode.
    */
    C1CTRL1bits.REQOP  = 0b100; // set configuration mode
    while ( 0b100 != C1CTRL1bits.OPMODE );

    /* Clear the CAN interface flags.
    */
    canflgs.fsFlags = 0;

    /* Read the device number from the configuration memory and set it.
    */
    devnumCur = CfgGetDevnum();

    /* Set the default Session ID. Note that this should never actually be used.
    */
    sessidCur = 0;

    /* Set the number of ticks required before transmitting a resposne frame to
    ** certain commands. Please note that a value of 0 indicates that no
    ** response is to be transmitted. A non-zero value should be set by the
    ** interrupt handler when a command requiring a delayed response is
    ** received.
    */
    ctickEnumResp = 0;

    /* Initialize the DTO and DTI buffers and their associated variables.
    */
    cbDtoCur = 0;
    cbDtoMac = 0;

    cbDtiCur = 0;
    cbDtoMac = 0;

    vcmdTransCur = vcmdNoCommand;

    /* Initialize variables used for writing flash pages during firmware
    ** updates.
    */
    ipageWrite = 0;
    imgtypWrite = imgtypApp;
    fwverWrite = 0xFFFF;

    /* Map the pins for the CAN module.
    */
    __builtin_write_OSCCONL(OSCCON & ~(1<<6));
    RPINR26bits.C1RXR = ppsCanRx;
    ppsCanTx = 0b001110; // CAN1 TX
    __builtin_write_OSCCONL(OSCCON | (1<<6));

    C1CTRL1bits.CSIDL = 0; // continue operation during idle mode
    C1CTRL1bits.CANCKS = 0; // FCAN = FPB = 64 MHz
    C1CTRL1bits.CANCAP = 0; // disable CAN input capture
    C1CTRL1bits.WIN = 0; // access CAN control and status registers

    /* Configure the CAN module for a baud rate of 1 Mbit with 16 TQ.
    */
    C1CFG1bits.SJW = 0b11; // sync jump width is 4 x TQ
    C1CFG1bits.BRP = 1; // set baud rate to 1Mbit with 16TQ

    C1CFG2bits.WAKFIL = 0; // CAN not used for wake-up
    C1CFG2bits.SEG2PH = 4;// phase 2 length is 5 x TQ
    C1CFG2bits.SEG2PHTS = 1; // phase 2 is freely programmable
    C1CFG2bits.SAM = 1; // bus line is sampled 3 times at sample point
    C1CFG2bits.SEG1PH = 4;// phase 1 length is 5 x TQ
    C1CFG2bits.PRSEG = 4; // propagation time is 5 x TQ

    /* Configure buffer count and FIFO settings.
    */
    C1FCTRLbits.DMABS = 0b110; // 32 buffers
    C1FCTRLbits.FSA = 8; // FIFO starts at buffer 8

    /* Configure the first 8 message buffers for TX. Do not respond to remote
    ** frames. Use nautral priority (e.g. buffer 0 is lowest, buffer 7 is
    ** highest) by setting all user priorities to the same value.
    */
    C1TR01CONbits.TXEN0 = 1; // transmit buffer
    C1TR01CONbits.RTREN0 = 0; // ignore remote frames
    C1TR01CONbits.TX0PRI = 0b11; // highest priority

    C1TR01CONbits.TXEN1 = 1; // transmit buffer
    C1TR01CONbits.RTREN1 = 0; // ignore remote frames
    C1TR01CONbits.TX1PRI = 0b11; // highest priority

    C1TR23CONbits.TXEN2 = 1; // transmit buffer
    C1TR23CONbits.RTREN2 = 0; // ignore remote frames
    C1TR23CONbits.TX2PRI = 0b11; // highest priority

    C1TR23CONbits.TXEN3 = 1; // transmit buffer
    C1TR23CONbits.RTREN3 = 0; // ignore remote frames
    C1TR23CONbits.TX3PRI = 0b11; // highest priority

    C1TR45CONbits.TXEN4 = 1; // transmit buffer
    C1TR45CONbits.RTREN4 = 0; // ignore remote frames
    C1TR45CONbits.TX4PRI = 0b11; // highest priority

    C1TR45CONbits.TXEN5 = 1; // transmit buffer
    C1TR45CONbits.RTREN5 = 0; // ignore remote frames
    C1TR45CONbits.TX5PRI = 0b11; // highest priority

    C1TR67CONbits.TXEN6 = 1; // transmit buffer
    C1TR67CONbits.RTREN6 = 0; // ignore remote frames
    C1TR67CONbits.TX6PRI = 0b11; // highest priority

    C1TR67CONbits.TXEN7 = 1; // transmit buffer
    C1TR67CONbits.RTREN7 = 0; // ignore remote frames
    C1TR67CONbits.TX7PRI = 0b11; // highest priority

    /* Initialize the message buffers.
    */
    memset(rgcmsgBuf, 0, sizeof(rgcmsgBuf));

    rgcmsgBuf[idxVendorDin].IDE = 1; // transmit extended id
    rgcmsgBuf[idxVendorDin].SRR = 1; // must be set to '1' for IDE = '1'
    rgcmsgBuf[idxVendorDin].SID = (msgidVendorDin >> 18);
    rgcmsgBuf[idxVendorDin].EIDh = ((msgidVendorDin & 0x0003FFFF) >> 6);
    rgcmsgBuf[idxVendorDin].EIDl = devnumCur & 0x3F; // lower 6-bits of EID is always device number
    rgcmsgBuf[idxVendorDin].DLC = sizeof(ENUMRSP0);

    rgcmsgBuf[idxVendorSts].IDE = 1; // transmit extended id
    rgcmsgBuf[idxVendorSts].SRR = 1; // must be set to '1' for IDE = '1'
    rgcmsgBuf[idxVendorSts].SID = (msgidVendorSts >> 18);
    rgcmsgBuf[idxVendorSts].EIDh = ((msgidVendorSts & 0x0003FFFF) >> 6);
    rgcmsgBuf[idxVendorSts].EIDl = devnumCur & 0x3F; // lower 6-bits of EID is always device number
    rgcmsgBuf[idxVendorSts].DLC = sizeof(VENDORSTS);

    rgcmsgBuf[idxTxEnum].IDE = 1; // transmit extended id
    rgcmsgBuf[idxTxEnum].SRR = 1; // must be set to '1' for IDE = '1'
    rgcmsgBuf[idxTxEnum].SID = (msgidEnumSnd >> 18);
    rgcmsgBuf[idxTxEnum].EIDh = ((msgidEnumSnd & 0x0003FFFF) >> 6);
    rgcmsgBuf[idxTxEnum].EIDl = devnumCur & 0x3F; // lower 6-bits of EID is always device number
    rgcmsgBuf[idxTxEnum].DLC = sizeof(ENUMRSP0);

    /* Disable the DMA channels used by the CAN module.
    */
    DMA0CONbits.CHEN = 0; // disable channel
    DMA1CONbits.CHEN = 0; // disable channel

    /* Configure DMA Channel 0 to transmit ECAN messages.
    */
    DMA0CONbits.SIZE = 0; // perform word transfer
    DMA0CONbits.DIR = 1; // read from RAM write to peripheral
    DMA0CONbits.HALF = 0; // interrupt after all data has transferred
    DMA0CONbits.NULLW = 0; // normal operation
    DMA0CONbits.AMODE = 0b10; // peripheral indirect address mode
    DMA0CONbits.MODE = 0b00; // continuous mode, no ping-pong

    DMA0REQbits.IRQSEL = 0b01000110; // ECAN1 TX Data Request

    DMA0CNT = 7; // 8 words per transfer request
    DMA0PAD = (volatile unsigned int)&C1TXD;

    DMA0STAL = (unsigned int)&rgcmsgBuf;
    DMA0STAH = (unsigned int)&rgcmsgBuf;

    /* Configure DMA Channel 1 to receive ECAN messagse.
    */
    DMA1CONbits.SIZE = 0; // perform word transfer
    DMA1CONbits.DIR = 0; // read from peripheral write to RAM
    DMA1CONbits.HALF = 0; // interrupt after all data has transferred
    DMA1CONbits.NULLW = 0; // normal operation
    DMA1CONbits.AMODE = 0b10; // peripheral indirect address mode
    DMA1CONbits.MODE = 0b00; // continuous mode, no ping-pong

    DMA1REQbits.IRQSEL = 0b00100010; // ECAN1 RX Data Request

    DMA1CNT = 7; // 8 words per transfer request
    DMA1PAD = (volatile unsigned int)&C1RXD;
    DMA1STAL = (unsigned int)&rgcmsgBuf;
    DMA1STAH = (unsigned int)&rgcmsgBuf;

    /* Enable the DMA channels.
    */
    DMA0CONbits.CHEN = 1;
    DMA1CONbits.CHEN = 1;

    /* Set the Window bit so that we can configure the message acceptance
    ** filters and buffer pointers.
    */
    C1CTRL1bits.WIN = 1;

    /* Assign all acceptance filters to FIFO.
    */
    C1BUFPNT1 = 0xFFFF;
    C1BUFPNT2 = 0xFFFF;
    C1BUFPNT3 = 0xFFFF;
    C1BUFPNT4 = 0xFFFF;

    /* Configure acceptance filter standard and extended ID for filters 0 and 1.
    */
    C1RXF0SIDbits.SID = (msgidDigilent >> 18);
    C1RXF0SIDbits.EID = ((msgidDigilent & 0x0003FFFF) >> 16);
    C1RXF0SIDbits.EXIDE = 1; // match only messsages with extended id
    C1RXF0EID = (msgidDigilent & 0x0000FFFF);

    C1RXF1SIDbits.SID = ((DWORD)msgidSysCtrl >> 18);
    C1RXF1SIDbits.EID = ((msgidSysCtrl & 0x0003FFFF) >> 16);
    C1RXF1SIDbits.EXIDE = 1; // match only messsages with extended id
    C1RXF1EID = (msgidSysCtrl & 0x0000FFFF);

    /* Configure acceptance mask 0.
    */
    C1RXM0SID = ((mskDigilentMtrCtrlr) >> 16);
    C1RXM0EID = mskDigilentMtrCtrlr & 0x0000FFFF;

    /* Configure acceptance mask 1.
    */
    C1RXM1SID = (((DWORD)mskBroadcast) >> 16);
    C1RXM1EID = mskBroadcast & 0x0000FFFF;

    /* Use Mask 0 for all acceptance filters except filter 1, which will use
    ** mask 1 at startup so that we can detect the CAN bus is present.
    */
    C1FMSKSEL1 = 0; // use mask 0
    C1FMSKSEL2 = 0; // use mask 0
    C1FMSKSEL1bits.F1MSK = 0b01; // use mask 1

    /* Enable acceptance filters 0 and 1.
    */
    C1FEN1 = 0;
    C1FEN1bits.FLTEN0 = 1;
    C1FEN1bits.FLTEN1 = 1;

    /* Clear the Window bit so that we can finish configuring the CAN module.
    */
    C1CTRL1bits.WIN = 0;

    /* Configure CAN interrupts.
    */
    C1INTEbits.ERRIE = 1; // enable error interrupt
    C1INTEbits.FIFOIE = 0; // disable FIFO almost full interrupt
    C1INTEbits.IVRIE = 1; // enable invalid message interupt
    C1INTEbits.RBIE = 1; // enable receive buffer interrupt
    C1INTEbits.RBOVIE = 1; // enable receive buffer overflow interrupt
    C1INTEbits.TBIE = 1; // enable transmit buffer interrupt
    C1INTEbits.WAKIE = 0; // disable wake interrupt
    IPC8bits.C1IP = 7; // set interrupt priority
    IFS2bits.C1IF = 0; // clear interrupt flag

    /* Place the CAN module in normal operation mode mode.
    */
    C1CTRL1bits.REQOP  = 0; // set normal operation mode
    while ( 0 != C1CTRL1bits.OPMODE );
}

/* ------------------------------------------------------------ */
/***	CanTerm
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
**      This function disables the CAN Interface and sets the registers back
**      to their default state.
*/
void
CanTerm() {

    C1CTRL1bits.REQOP  = 0b100; // set configuration mode
    while ( 0b100 != C1CTRL1bits.OPMODE );

    C1CTRL1bits.WIN = 0; // access CAN control and status registers

    C1CFG1 = 0;
    C1CFG2 = 0;
    C1FEN1 = 0;

    C1FMSKSEL1 = 0;
    C1FMSKSEL2 = 0;

    C1RXFUL1 = 0;
    C1RXFUL2 = 0;
    C1RXOVF1 = 0;
    C1RXOVF2 = 0;

    C1TR01CON = 0;
    C1TR23CON = 0;
    C1TR45CON = 0;
    C1TR67CON = 0;

    C1FCTRL = 0;
    C1INTE = 0;
    C1INTF = 0;
    IFS2bits.C1IF = 0;

    C1CTRL1bits.WIN = 1; // access message buffers and acceptance filters

    C1RXF0SID = 0;
    C1RXF1SID = 0;
    C1RXF2SID = 0;
    C1RXF3SID = 0;
    C1RXF4SID = 0;
    C1RXF5SID = 0;
    C1RXF6SID = 0;
    C1RXF7SID = 0;
    C1RXF8SID = 0;
    C1RXF9SID = 0;
    C1RXF10SID = 0;
    C1RXF11SID = 0;
    C1RXF12SID = 0;
    C1RXF13SID = 0;
    C1RXF14SID = 0;
    C1RXF15SID = 0;

    C1RXF0EID = 0;
    C1RXF1EID = 0;
    C1RXF2EID = 0;
    C1RXF3EID = 0;
    C1RXF4EID = 0;
    C1RXF5EID = 0;
    C1RXF6EID = 0;
    C1RXF7EID = 0;
    C1RXF8EID = 0;
    C1RXF9EID = 0;
    C1RXF10EID = 0;
    C1RXF11EID = 0;
    C1RXF12EID = 0;
    C1RXF13EID = 0;
    C1RXF14EID = 0;
    C1RXF15EID = 0;

    C1RXM0SID = 0;
    C1RXM1SID = 0;
    C1RXM2SID = 0;

    C1RXM0EID = 0;
    C1RXM1EID = 0;
    C1RXM2EID = 0;

    C1BUFPNT1 = 0;
    C1BUFPNT2 = 0;
    C1BUFPNT3 = 0;
    C1BUFPNT4 = 0;

    C1CTRL1bits.WIN = 0; // access CAN control and status registers

    DMA0CONbits.CHEN = 0; // disable DMA channel 0
    DMA1CONbits.CHEN = 0; // disable DMA channel 1

    /* Map the TX pins to the TRIS port.
    */
    latCanTx |= (1 << bnCanTx);
    __builtin_write_OSCCONL(OSCCON & ~(1<<6));
    RPINR26bits.C1RXR = ppsCanRx;
    ppsCanTx = 0b000000; // TRIS PORT
    __builtin_write_OSCCONL(OSCCON | (1<<6));

    trsCanTx |= (1 << bnCanTx);
}

/* ------------------------------------------------------------ */
/***	CanIsLinkPresent
**
**  Parameters:
**      none
**
**  Return Value:
**      fTrue if the CAN link is present, fFalse otherwise
**
**  Errors:
**      none
**
**  Description:
**      This function returns fTrue if the CAN link is present. If
**      no link is present then fFalse is returned.
*/
BOOL
CanIsLinkPresent() {

    return ( canflgs.fBusDetected ) ? fTrue : fFalse;
}

/* ------------------------------------------------------------ */
/***	CanTick
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
**      This function is called at a fixed interval when Timer 5 rolls over.
**      It implements any CAN operations that require timing and/or delays.
*/
void
CanTick() {

    /* Check to see if we need to transmit the first frame in response to an
    ** enumeration request.
    */
    if (( canflgs.fTxEnumResp ) && ( 0 < ctickEnumResp  )) {
        ctickEnumResp--;
        if ( 0 == ctickEnumResp ) {
            txreqEnum = 1;
        }
    }
}

/* ------------------------------------------------------------ */
/***	CanAckEnterBootloader
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
**      This routine generates a vendor status response to acknowledge
**      that the device has entered the bootloader and queues it for
**      transmission in the vendor status transmit buffer.
**
**  Notes:
**      This function should be called after initialization has been completed
**      and prior to entering the main loop. It should only be called when the
**      application firmware explictly instructed us to stay in the bootloader.
*/
void
CanAckEnterBootloader() {

    if ( txreqVendorSts ) {
        txreqVendorSts = 0;
    }

    rgcmsgBuf[idxVendorSts].DLC = sizeof(VENDORSTS);
    rgcmsgBuf[idxVendorSts].EIDl = devnumCur; // lower 6-bits of EID is always device number
    ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cerc = cercInBootloader;
    ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cb = 0;
    txreqVendorSts = 1;
}

/* ------------------------------------------------------------ */
/***	CanXfrDataOut
**
**  Parameters:
**      pcsmg   -   pointer to a CAN message buffer
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine serves as the data handler for the DTO endpoint. It
**      buffers data that's received on the DTO endpoint and then calls the
**      appropriate data handler once all of the data corresponding to a
**      particular command has been received. A status frame is only generated
**      after all data has been received and the command has been processed or
**      if an error occurs.
*/
void
CanXfrDataOut(CANMSG* pcmsg) {

    DWORD           crc;
    WORD            cerc;
    _prog_addressT  paddr;
    unsigned int    tblpgSave;

    if ( vcmdNoCommand == vcmdTransCur ) {
        return;
    }

    if ( (cbDtoCur + pcmsg->DLC) > cbDtoMac ) {
        if ( txreqVendorSts ) {
            txreqVendorSts = 0;
        }

        rgcmsgBuf[idxVendorSts].DLC = sizeof(VENDORSTS);
        rgcmsgBuf[idxVendorSts].EIDl = devnumCur; // lower 6-bits of EID is always device number
        ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cerc = cercDataRcvMore;
        ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cb = 0;

        txreqVendorSts = 1;

        vcmdTransCur = vcmdNoCommand;

        return;
    }

    memcpy(((BYTE*)rgbDtoBuf) + cbDtoCur, pcmsg->rgbData, pcmsg->DLC);
    cbDtoCur += pcmsg->DLC;
    if ( cbDtoCur == cbDtoMac ) {

        cerc = cercNoError;

        switch ( vcmdTransCur ) {
            case vcmdSetDevName:
                CfgSetSzDevName((const char*)rgbDtoBuf);
                break;

            case vcmdSetManName:
                CfgSetSzManName((const char*)rgbDtoBuf);
                break;

            case vcmdSetProdName:
                CfgSetSzProdName((const char*)rgbDtoBuf);
                break;

            case vcmdSetManDate:
                CfgSetSzManDate((const char*)rgbDtoBuf);
                break;

            case vcmdSetHardwareVer:
                CfgSetSzHardwareVer((const char*)rgbDtoBuf);
                break;

            case vcmdSetSN:
                CfgSetSzSN((const char*)rgbDtoBuf);
                break;

            case vcmdEraseWriteFlashPage:
                /* Calculate the CRC and make sure it matches the CRC that was
                ** sent. If it doesn't match then set an appropriate error code
                ** and break out.
                */
                crc = CrcCalcCRC32(0, (const BYTE*)rgbDtoBuf, 4096);
                if ( crc != *((DWORD*)(&(rgbDtoBuf[4096]))) ) {
                    cerc = cercCrcMismatch;
                    break;
                }

                _init_prog_address(paddr, _RESET);
                paddr += ((_prog_addressT)ipageWrite * 2048);

                if ( iflashpAppMin == ipageWrite ) {
                    if ( imgtypApp == imgtypWrite ) {
                        /* We need to fix up the reset address so that we reset
                        ** to the bootloader startup function. The easiest way
                        ** to do this is to read the goto instruction and reset
                        ** address out of flash and into our buffer.
                        */
                        DISABLE_INTERRUPTS
                        tblpgSave = TBLPAG;
                        _memcpy_p2d24(&(((DWORD*)rgbDtoBuf)[0]), paddr, 3);
                        _memcpy_p2d24(&(((DWORD*)rgbDtoBuf)[1]), paddr + 2, 3);
                        TBLPAG = tblpgSave;
                        ENABLE_INTERRUPTS
                    }

                    DISABLE_INTERRUPTS

                    if ( ! FlashEraseWriteVerifyPage24(ipageWrite, (DWORD*)rgbDtoBuf, 1024, 5) ) {
                        cerc = cercFlashWriteFailed;
                        ENABLE_INTERRUPTS
                        break;
                    }

                    ENABLE_INTERRUPTS

                    /* Now write the last two 24-bit words of the last page,
                    ** which includes the firmware version.
                    */
                    _init_prog_address(paddr, _RESET);
                    paddr += (((_prog_addressT)(iflashpAppMax + 1) * 2048) - 4);
                    DISABLE_INTERRUPTS
                    FlashWrite24(paddr, (DWORD*)rgdwLastPage, 2);
                    ENABLE_INTERRUPTS
                }
                else if ( iflashpAppMax == ipageWrite ) {
                    DISABLE_INTERRUPTS
                    if ( ! FlashEraseWriteVerifyPage24(ipageWrite, (DWORD*)rgbDtoBuf, 1022, 5) ) {
                        cerc = cercFlashWriteFailed;
                        ENABLE_INTERRUPTS
                        break;
                    }
                    ENABLE_INTERRUPTS

                    memcpy((void*)rgdwLastPage, &(((DWORD*)rgbDtoBuf)[1022]), sizeof(rgdwLastPage));
                }
                else {
                    DISABLE_INTERRUPTS
                    if ( ! FlashEraseWriteVerifyPage24(ipageWrite, (DWORD*)rgbDtoBuf, 1024, 5) ) {
                        cerc = cercFlashWriteFailed;
                        ENABLE_INTERRUPTS
                        break;
                    }
                    ENABLE_INTERRUPTS
                }

                break;

            default:
                /* Data packet is associated with an unrecognized command.
                ** We should never get here, and thus, will not generate a
                ** status packet. Unrecognized commands should be handled by
                ** the command dispatcher.
                */
                vcmdTransCur = vcmdNoCommand;
                return;
        }

        vcmdTransCur = vcmdNoCommand;

        if ( txreqVendorSts ) {
            txreqVendorSts = 0;
        }

        rgcmsgBuf[idxVendorSts].DLC = sizeof(VENDORSTS);
        rgcmsgBuf[idxVendorSts].EIDl = devnumCur; // lower 6-bits of EID is always device number
        ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cerc = cerc;
        ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cb = 0;
        txreqVendorSts = 1;

        if ( cercNoError == cerc ) {
            LedDispProgFlash();
        }
    }
}

/* ------------------------------------------------------------ */
/***	CanDispatchVendorCmd
**
**  Parameters:
**      pcmsg   -   pointer to a CAN message buffer
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine calls the correct handler for the specified vendor command.
**      Prior to calling the handler the frame's data length is verified and
**      the session ID is compared against this device's current session ID. If
**      they do not match then the vendor command is discarded.
**
**  Notes:
**      It is the caller's responsibility to verify that the frame contains the
**      correct message ID and that the device number specified in the message
**      ID matches this device's current device number.
*/
void
CanDispatchVendorCmd(CANMSG* pcmsg) {

    VENDORCMD*  pvcmd;

    /* Make sure that the message we received is the correct size.
    */
    if ( sizeof(VENDORCMD) != pcmsg->DLC ) {
        return;
    }

    pvcmd = (VENDORCMD*)(pcmsg->rgbData);

    /* Make sure that the Session ID for this Vendor Command matches our Session
    ** Id.
    */
    if ( pvcmd->sessid != sessidCur ) {
        return;
    }

    /* We do not expect to receive a new command until the command issuer has
    ** received the status from any previous command. If there is an existing
    ** message in the transmit buffer that has not yet been transmitted then
    ** abort it, as we will generate a new status frame later.
    */
    if ( txreqVendorSts ) {
        txreqVendorSts = 0;
    }

    /* Discard any ongoing data transfers.
    */
    vcmdTransCur = vcmdNoCommand;

    /* Call an appropriate handler based on the specified command.
    */
    switch ( pvcmd->cmd ) {
        case vcmdSetDevNumber:
            CanCmdSetDevNumber(pvcmd);
            break;

        case vcmdSetDevName:
            CanCmdSetDevName(pvcmd);
            break;
            
        case vcmdSetManName:
            CanCmdSetManName(pvcmd);
            break;

        case vcmdSetProdName:
            CanCmdSetProdName(pvcmd);
            break;

        case vcmdSetManDate:
            CanCmdSetManDate(pvcmd);
            break;

        case vcmdSetHardwareVer:
            CanCmdSetHardwareVer(pvcmd);
            break;

        case vcmdSetSN:
            CanCmdSetSN(pvcmd);
            break;

        case vcmdGetDescriptors:
            CanCmdGetDescriptors(pvcmd);
            break;

        case vcmdGetFwver:
            CanCmdGetFwver(pvcmd);
            break;

        case vcmdEnterBootloader:
            /* Already runing bootloader so send an error.
            */
            rgcmsgBuf[idxVendorSts].DLC = sizeof(VENDORSTS);
            rgcmsgBuf[idxVendorSts].EIDl = devnumCur; // lower 6-bits of EID is always device number
            ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cerc = cercInBootloader;
            ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cb = 0;
            txreqVendorSts = 1;
            break;

        case vcmdSoftReset:
            sessidPrev = sessidCur;
            bootflgs.fAckSoftReset = 1;
            asm("RESET");
            break;

        case vcmdEraseWriteFlashPage:
            CanCmdEraseWriteFlashPage(pvcmd);
            break;

        default:
            rgcmsgBuf[idxVendorSts].DLC = sizeof(VENDORSTS);
            rgcmsgBuf[idxVendorSts].EIDl = devnumCur; // lower 6-bits of EID is always device number
            ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cerc = cercNotSupported;
            ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cb = 0;
            txreqVendorSts = 1;
            break;
    }
}

/* ------------------------------------------------------------ */
/***	CanCmdSetDevNumber
**
**  Parameters:
**      pvcmd   -   pointer to a vendor command packet
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets the device number to number specified in the first
**      parameter of the command packet and writes it to flash. A status frame
**      containing an error code that indicates success or failure is queued for
**      transmission in the vendor status transmit buffer.
*/
void
CanCmdSetDevNumber(VENDORCMD* pvcmd) {

    WORD    cerc;

    /* M00TODO: consider scenarios under which we should NOT allow our device
    ** number to be changed.
    */

    if ( devnumMax >= pvcmd->PARAM1 ) {
        devnumCur = pvcmd->PARAM1;
        CfgSetDevnum(devnumCur);
        cerc = cercNoError;
    }
    else {
        cerc = cercBadParameter;
    }

    /* Transmit the status frame.
    */
    rgcmsgBuf[idxVendorSts].DLC = sizeof(VENDORSTS);
    rgcmsgBuf[idxVendorSts].EIDl = devnumCur; // lower 6-bits of EID is always device number
    ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cerc = cerc;
    ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cb = 0;
    txreqVendorSts = 1;
    
    LedDispProgFlash();
}

/* ------------------------------------------------------------ */
/***	CanCmdSetDevName
**
**  Parameters:
**      pvcmd   -   pointer to a vendor command packet
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets up the DTO buffer/endpoint to receive the device name
**      string. No status frame is generated unless an error occurs. If an error
**      occurs then a status frame containing an appropriate error code is
**      queued for transmission in the vendor status transmit buffer.
*/
void
CanCmdSetDevName(VENDORCMD* pvcmd) {
    
    /* Make sure that the character count specified for the string is
    ** valid. If not, then return a status packet immediately and return.
    */
    if ( (cchDevNameMax  + 1) < pvcmd->PARAM1 ) {
        rgcmsgBuf[idxVendorSts].DLC = sizeof(VENDORSTS);
        rgcmsgBuf[idxVendorSts].EIDl = devnumCur; // lower 6-bits of EID is always device number
        ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cerc = cercBadParameter;
        ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cb = 0;
        txreqVendorSts = 1;
        return;
    }

    /* Set up the DTO buffer to perform the data transfer.
    */
    cbDtoCur = 0;
    cbDtoMac = pvcmd->PARAM1;

    /* Save the command so that we can write the appropriate descriptor after
    ** all data has been received.
    */
    vcmdTransCur = pvcmd->cmd;
}

/* ------------------------------------------------------------ */
/***	CanCmdSetManName
**
**  Parameters:
**      pvcmd   -   pointer to a vendor command packet
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets up the DTO buffer/endpoint to receive the manufacturer
**      name string. No status frame is generated unless an error occurs. If an
**      error occurs then a status frame containing an appropriate error code is
**      queued for transmission in the vendor status transmit buffer.
*/
void
CanCmdSetManName(VENDORCMD* pvcmd) {

    /* Make sure that the character count specified for the string is
    ** valid. If not, then return a status packet immediately and return.
    */
    if ( (cchManNameMax + 1) < pvcmd->PARAM1 ) {
        rgcmsgBuf[idxVendorSts].DLC = sizeof(VENDORSTS);
        rgcmsgBuf[idxVendorSts].EIDl = devnumCur; // lower 6-bits of EID is always device number
        ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cerc = cercBadParameter;
        ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cb = 0;
        txreqVendorSts = 1;
        return;
    }

    /* Set up the DTO buffer to perform the data transfer.
    */
    cbDtoCur = 0;
    cbDtoMac = pvcmd->PARAM1;

    /* Save the command so that we can write the appropriate descriptor after
    ** all data has been received.
    */
    vcmdTransCur = pvcmd->cmd;
}

/* ------------------------------------------------------------ */
/***	CanCmdSetProdName
**
**  Parameters:
**      pvcmd   -   pointer to a vendor command packet
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets up the DTO buffer/endpoint to receive the product name
**      string. No status frame is generated unless an error occurs. If an error
**      occurs then a status frame containing an appropriate error code is
**      queued for transmission in the vendor status transmit buffer.
*/
void
CanCmdSetProdName(VENDORCMD* pvcmd) {

    /* Make sure that the character count specified for the string is
    ** valid. If not, then return a status packet immediately and return.
    */
    if ( (cchProdNameMax + 1) < pvcmd->PARAM1 ) {
        rgcmsgBuf[idxVendorSts].DLC = sizeof(VENDORSTS);
        rgcmsgBuf[idxVendorSts].EIDl = devnumCur; // lower 6-bits of EID is always device number
        ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cerc = cercBadParameter;
        ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cb = 0;
        txreqVendorSts = 1;
        return;
    }

    /* Set up the DTO buffer to perform the data transfer.
    */
    cbDtoCur = 0;
    cbDtoMac = pvcmd->PARAM1;

    /* Save the command so that we can write the appropriate descriptor after
    ** all data has been received.
    */
    vcmdTransCur = pvcmd->cmd;
}

/* ------------------------------------------------------------ */
/***	CanCmdSetManDate
**
**  Parameters:
**      pvcmd   -   pointer to a vendor command packet
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets up the DTO buffer/endpoint to receive the
**      manufacturing date string. No status frame is generated unless an error
**      occurs. If an error occurs then a status frame containing an appropriate
**      error code is queued for transmission in the vendor status transmit
**      buffer.
*/
void
CanCmdSetManDate(VENDORCMD* pvcmd) {

    /* Make sure that the character count specified for the string is
    ** valid. If not, then return a status packet immediately and return.
    */
    if ( (cchManDateMax + 1) < pvcmd->PARAM1 ) {
        rgcmsgBuf[idxVendorSts].DLC = sizeof(VENDORSTS);
        rgcmsgBuf[idxVendorSts].EIDl = devnumCur; // lower 6-bits of EID is always device number
        ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cerc = cercBadParameter;
        ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cb = 0;
        txreqVendorSts = 1;
        return;
    }

    /* Set up the DTO buffer to perform the data transfer.
    */
    cbDtoCur = 0;
    cbDtoMac = pvcmd->PARAM1;

    /* Save the command so that we can write the appropriate descriptor after
    ** all data has been received.
    */
    vcmdTransCur = pvcmd->cmd;
}

/* ------------------------------------------------------------ */
/***	CanCmdSetHardwareVer
**
**  Parameters:
**      pvcmd   -   pointer to a vendor command packet
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets up the DTO buffer/endpoint to receive the hardware
**      version string. No status frame is generated unless an error occurs. If
**      an error occurs then a status frame containing an appropriate error code
**      is queued for transmission in the vendor status transmit buffer.
*/
void
CanCmdSetHardwareVer(VENDORCMD* pvcmd) {

    /* Make sure that the character count specified for the string is
    ** valid. If not, then return a status packet immediately and return.
    */
    if ( (cchHardwareVerMax + 1) < pvcmd->PARAM1 ) {
        rgcmsgBuf[idxVendorSts].DLC = sizeof(VENDORSTS);
        rgcmsgBuf[idxVendorSts].EIDl = devnumCur; // lower 6-bits of EID is always device number
        ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cerc = cercBadParameter;
        ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cb = 0;
        txreqVendorSts = 1;
        return;
    }

    /* Set up the DTO buffer to perform the data transfer.
    */
    cbDtoCur = 0;
    cbDtoMac = pvcmd->PARAM1;

    /* Save the command so that we can write the appropriate descriptor after
    ** all data has been received.
    */
    vcmdTransCur = pvcmd->cmd;
}

/* ------------------------------------------------------------ */
/***	CanCmdSetSN
**
**  Parameters:
**      pvcmd   -   pointer to a vendor command packet
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets up the DTO buffer/endpoint to receive the serial
**      number string. No status frame is generated unless an error occurs. If
**      an error occurs then a status frame containing an appropriate error code
**      is queued for transmission in the vendor status transmit buffer.
*/
void
CanCmdSetSN(VENDORCMD* pvcmd) {

    /* Make sure that the character count specified for the string is
    ** valid. If not, then return a status packet immediately and return.
    */
    if ( (cchSNMax + 1) < pvcmd->PARAM1 ) {
        rgcmsgBuf[idxVendorSts].DLC = sizeof(VENDORSTS);
        rgcmsgBuf[idxVendorSts].EIDl = devnumCur; // lower 6-bits of EID is always device number
        ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cerc = cercBadParameter;
        ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cb = 0;
        txreqVendorSts = 1;
        return;
    }

    /* Set up the DTO buffer to perform the data transfer.
    */
    cbDtoCur = 0;
    cbDtoMac = pvcmd->PARAM1;

    /* Save the command so that we can write the appropriate descriptor after
    ** all data has been received.
    */
    vcmdTransCur = pvcmd->cmd;
}

/* ------------------------------------------------------------ */
/***	CanCmdGetDescriptors
**
**  Parameters:
**      pvcmd   -   pointer to a vendor command packet
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets up the DTI buffer/endpoint to transmit the device
**      descriptors. A status frame containing the number of bytes that the
**      host/master should expect to receive is queued for tranmission in the
**      vendor status transmit buffer. The first DTI frame is also queued for
**      tranmission in the vendor DTI transmit buffer.
*/
void
CanCmdGetDescriptors(VENDORCMD* pvcmd) {

    WORD    cbTrans;

    /* If we received a new request for the Descriptor Table and we are
    ** presently transmitting data over the Vendor Data In Endpoint then we
    ** should discard any data waiting to be sent.
    */
    if ( txreqVendorDin ) {
        txreqVendorDin = 0;
    }

    /* Get the descriptor table and copy it to the DTI buffer.
    */
    cbDtiCur = 0;
    cbDtiMac = CfgGetDescTable((BYTE*)rgbDtiBuf, cbDtiMax);

    /* Transmit the status frame. Include the count of bytes to be transferred
    ** over the Vendor DTI endpoint.
    */
    rgcmsgBuf[idxVendorSts].DLC = sizeof(VENDORSTS);
    rgcmsgBuf[idxVendorSts].EIDl = devnumCur; // lower 6-bits of EID is always device number
    ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cerc = cercNoError;
    ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cb = cbDtiMac;
    txreqVendorSts = 1;

    /* Transmit the first Data IN frame. Please note that the VendorDti transmit
    ** buffer must be lower priority than the VendorSts transmit buffer, as the
    ** master expects to receive the status frame before any data frames arrive.
    */
    if ( cbDtiCur < cbDtiMac ) {
        /* Determine how many bytes to transmit this time.
        */
        cbTrans = 8;
        if ( (cbDtiCur + cbTrans) > cbDtiMac ) {
            cbTrans = cbDtiMac - cbDtiCur;
        }

        /* Copy the next chunk of data to the message buffer,
        ** increment the count of bytes transferred, and mark the
        ** buffer for transmission.
        */
        memcpy(rgcmsgBuf[idxVendorDin].rgbData, ((BYTE*)rgbDtiBuf) + cbDtiCur, cbTrans);
        rgcmsgBuf[idxVendorDin].DLC = cbTrans;
        rgcmsgBuf[idxVendorDin].EIDl = devnumCur;
        cbDtiCur += cbTrans;
        txreqVendorDin = 1;
    }
}

/* ------------------------------------------------------------ */
/***	CanCmdGetFwver
**
**  Parameters:
**      pvcmd   -   pointer to a vendor command packet
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets up the DTI buffer/endpoint to transmit a packet
**      containing the device's firmware revisions (bootloader and application).
**      A status frame containing the number of bytes that the host/master
**      should expect to receive is queued for tranmission in the vendor status
**      transmit buffer. The first DTI frame is also queued for tranmission in
**      the vendor DTI transmit buffer.
*/
void
CanCmdGetFwver(VENDORCMD* pvcmd) {

    WORD    cbTrans;

    /* If we are presently transmitting data over the Vendor Data In Endpoint
    ** then we should discard any data waiting to be sent, as we are now
    ** processing a new command.
    */
    if ( txreqVendorDin ) {
        txreqVendorDin = 0;
    }

    /* Set up the DTI buffer to perform the data transfer.
    */
    cbDtiCur = 0;
    cbDtiMac = sizeof(FWVERRSP);

    ((FWVERRSP*)rgbDtiBuf)->imgtyp = imgtypCur;
    ((FWVERRSP*)rgbDtiBuf)->rsv = 0;
    ((FWVERRSP*)rgbDtiBuf)->fwverApp =CfgGetAppFwver();
    ((FWVERRSP*)rgbDtiBuf)->fwverBoot = CfgGetBootFwver();

    /* Transmit the status frame. Include the count of bytes to be transferred
    ** over the Vendor DTI endpoint.
    */
    rgcmsgBuf[idxVendorSts].DLC = sizeof(VENDORSTS);
    rgcmsgBuf[idxVendorSts].EIDl = devnumCur; // lower 6-bits of EID is always device number
    ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cerc = cercNoError;
    ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cb = cbDtiMac;
    txreqVendorSts = 1;


    /* Determine how many bytes to transmit this time.
    */
    cbTrans = 8;
    if ( (cbDtiCur + cbTrans) > cbDtiMac ) {
        cbTrans = cbDtiMac - cbDtiCur;
    }

    /* Copy the next chunk of data to the message buffer,
    ** increment the count of bytes transferred, and mark the
    ** buffer for transmission.
    */
    memcpy(rgcmsgBuf[idxVendorDin].rgbData, ((BYTE*)rgbDtiBuf) + cbDtiCur, cbTrans);
    rgcmsgBuf[idxVendorDin].DLC = cbTrans;
    rgcmsgBuf[idxVendorDin].EIDl = devnumCur;
    cbDtiCur += cbTrans;
    txreqVendorDin = 1;
}

/* ------------------------------------------------------------ */
/***	CanCmdEraseWriteFlashPage
**
**  Parameters:
**      pvcmd   -   pointer to a vendor command packet
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets up the DTO buffer/endpoint to receive a flash page as
**      part of an application/auxiliary bootloader firmware download. No status
**      frame is generated unless an error occurs. If an error occurs then a
**      status frame containing an appropriate error code is queued for
**      transmission in the vendor status transmit buffer.
*/
void
CanCmdEraseWriteFlashPage(VENDORCMD* pvcmd) {

    /* Make sure that the image type being written is one of the allowable
    ** types. Since this is the bootloader we can only write application images
    ** and aux bootloaders to the flash.
    */
    if (( imgtypApp != pvcmd->PARAM1 ) && ( imgtypAuxBoot != pvcmd->PARAM1 )) {
        rgcmsgBuf[idxVendorSts].DLC = sizeof(VENDORSTS);
        rgcmsgBuf[idxVendorSts].EIDl = devnumCur; // lower 6-bits of EID is always device number
        ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cerc = cercBadParameter;
        ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cb = 0;
        txreqVendorSts = 1;
        return;
    }

    /* Make sure that the flash page that was specified is valid for an
    ** application or aux boot image.
    */
    if (( iflashpAppMin > pvcmd->PARAM2 ) ||
        ( iflashpAppMax < pvcmd->PARAM2 )) {
        rgcmsgBuf[idxVendorSts].DLC = sizeof(VENDORSTS);
        rgcmsgBuf[idxVendorSts].EIDl = devnumCur; // lower 6-bits of EID is always device number
        ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cerc = cercBadParameter;
        ((VENDORSTS*)(&(rgcmsgBuf[idxVendorSts].rgbData[0])))->cb = 0;
        txreqVendorSts = 1;
        return;
    }

    /* Set up the DTO buffer to perform the data transfer.
    */
    cbDtoCur = 0;
    cbDtoMac = 4100; // 4K byte page and 4 byte CRC32

    /* Save the image type and the page number that's to be written.
    */
    imgtypWrite = pvcmd->PARAM1;
    ipageWrite = pvcmd->PARAM2;

    /* Save the command so that we can perform the flash write once an entire
    ** flash page has been received.
    */
    vcmdTransCur = pvcmd->cmd;
}