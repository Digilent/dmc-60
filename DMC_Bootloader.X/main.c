/************************************************************************/
/*									*/
/*  main.c  --  DMC Bootloader Firmware Main Program Module             */
/*									*/
/*									*/
/************************************************************************/
/*  Author: Michael T. Alexander                                        */
/*  Copyright 2015, Digilent Inc.					*/
/************************************************************************/
/*  Module Description: 						*/
/*									*/
/*  This module is the main program module of the DMC60 / DMC60C        */
/*  Bootloader.                                                         */
/*                                                                      */
/************************************************************************/
/*  Revision History:						        */
/*									*/
/*  09/13/2016 (MichaelA): created			                */
/*									*/
/************************************************************************/

#include <xc.h>

/* ------------------------------------------------------------ */
/*                    USER ID                                   */
/* ------------------------------------------------------------ */

#if defined(DMC60)
#define prodid  0x601
#define var     0x001
#define fwid    0x01
#elif defined(DMC60C)
#define prodid  0x602
#define var     0x001
#define fwid    0x01
#endif

_FUID0((var << 8) | fwid)
_FUID1((prodid << 4) | (var >> 8))
_FUID2(0)
_FUID3(0)

/* ------------------------------------------------------------ */
/*                    Configuration Bits                        */
/* ------------------------------------------------------------ */

// FICD
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config ALTI2C1 = OFF            // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
#pragma config WDTWIN = WIN25           // Watchdog Window Select bits (WDT Window is 25% of WDT period)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON              // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FOSC
#pragma config POSCMD = EC              // Primary Oscillator Mode Select bits (EC (External Clock) Mode)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function bit (OSC2 is general purpose digital I/O pin)
#pragma config IOL1WAY = OFF            // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSECMD           // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config PWMLOCK = OFF            // PWM Lock Enable bit (PWM registers may be written without key sequence)
#pragma config IESO = ON                // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF                // General Segment Code-Protect bit (General Segment Code protect is Disabled)

/* ------------------------------------------------------------ */
/*                  Include File Definitions                    */
/* ------------------------------------------------------------ */

#include <libq.h>
#include <libpic30.h>
#include <stdlib.h>
#include "DMC.h"
#include "stdtypes.h"
#include "Cfg.h"
#include "Led.h"
#include "Flash.h"
#include "Can.h"
#include "prodcfg.h"
#include "Crc.h"

/* ------------------------------------------------------------ */
/*                  Local Type Definitions	                */
/* ------------------------------------------------------------ */

typedef void __attribute__((far, noreturn)) (*FNAPPSTART)(void);

/* ------------------------------------------------------------ */
/*                  Global Variables                            */
/* ------------------------------------------------------------ */

extern __attribute__((space(prog))) WORD _FWVERBOOT;
WORD __attribute__((section("__FWVERBOOT.sec"),space(prog))) _FWVERBOOT = ((fwverMjr << 8) | fwverMin) ;

const BYTE  imgtypCur = imgtypBoot;

__attribute__((persistent, address(0x4FFC))) WORD   sessidPrev;
__attribute__((persistent, address(0x4FFE))) BOOTFLAGS  bootflgs;


static FNAPPSTART AppStart = (FNAPPSTART) 0x000200;

/* ------------------------------------------------------------ */
/*                  Local Variables                             */
/* ------------------------------------------------------------ */



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
/*                  Forward Declarations                        */
/* ------------------------------------------------------------ */

void	DeviceInit(void);
void    DeviceTerm(void);
void	AppInit(void);
void    AppTerm(void);

#define DELAY_20_MS \
{\
    WORD i;\
    WORD j;\
    for ( i = 0; i < 2; i++ ) {\
        for ( j = 0; j < 58200; j++ ) {\
            asm("nop");\
        }\
    }\
}\

/* ------------------------------------------------------------ */
/*                  Interrupt Service Routines                  */
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*                  Procedure Definitions                       */
/* ------------------------------------------------------------ */
/***	main
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
**      Main program module. Performs basic board initialization
**      and then enters the main program loop.
*/
int main(void) {

    DeviceInit();
    AppInit();

    /* If we haven't been explicitly instructed to stay in the bootloader then
    ** we should check to see if the application image is present. If it's
    ** present then we should shutdown any peripherals that we may have enabled
    ** and jump to the application startup routine.
    */
    if ( 0 == bootflgs.fStayInBootloader ) {
        if ( 0xFFFF != CfgGetAppFwver() ) {
            /* Application image appears to be present. Perform cleanup and
            ** shutdown the peripherals before jumping to the application
            ** startup routine.
            */
            AppTerm();
            DeviceTerm();

            /* Jump to the application startup code. Please note that there is
            ** no return from this function call.
            */
            AppStart();
        }
    }
    else {
        bootflgs.fStayInBootloader = 0;
        CanBusDetected();
        CanAckEnterBootloader();
    }

    while ( fTrue ) {
        /* Check to see if a CAN interrupt has occured. If so, clear the
        ** interrupt flag and service it. Please note that we implement this
        ** via polling as to avoid using interrupts in the bootloader.
        */
        if ( IFS2bits.C1IF ) {

            IFS2bits.C1IF = 0;

            CanInterruptHandler();
        }

        /* Check to see if Timer 5 has expired. If it has expired then the
        ** Timer's count will roll over and the interrupt flag will be set. By
        ** polling the flag here we can remove the need to set a flag in
        ** interrupt context, disable interrupts, and then poll and clear the
        ** flag here. This will ultimately reduce interrupt latency.
        */
        if ( IFS1bits.T5IF ) {

            IFS1bits.T5IF = 0;

            CanTick();

            LedTick();
        }
    }

    return 0;
}

/* ------------------------------------------------------------ */
/***    DeviceInit
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
**      This routine initializes on-chip and on-board peripheral
**      devices to their default state.
*/
void DeviceInit(void) {

    /* Configure PLL prescaler, postscaler, and divisor for 64 MIPS (MHz).
    */
    PLLFBD = 62; // M = 64
    CLKDIVbits.PLLPOST = 0; // N2 = 2
    CLKDIVbits.PLLPRE = 0; // N1 = 2

    /* Initiate clock switch.
    */
    __builtin_write_OSCCONH(0x03);
    __builtin_write_OSCCONL(OSCCON | 0x01);

    /* Wait for clock switch to occur.
    */
    while ( 0b011 != OSCCONbits.COSC ) {
        asm("nop");
    }

    /* Wait for PLL lock.
    */
    while ( 1 != OSCCONbits.LOCK ) {
        asm("nop");
    }

    /* Configure all Analog Pins as Digital IO.
    */
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;

    /* Configure onboard button as an input.
    */
    trsBtnCal |= (1 << bnBtnCal);

    /* The CAL button RC network requires 20ms to stabalize after power on.
    */
    DELAY_20_MS

    /* Configure Timer 3. This timer will be used to generate a Session ID when
    ** a CAN link is established. Please note that this timer should run forever
    ** and that interrupts should NOT be enabled.
    */
    T3CONbits.TON = 0; // stop the timer
    T3CONbits.TSIDL = 0; // continue to operate in idle mode
    T3CONbits.TGATE = 0; // disable gated timer mode
    T3CONbits.TCKPS = 0b00; // 1:1 (EG run at 64MHz)
    T3CONbits.TCS = 0; // use peripheral bus as clock source
    TMR3 = 0; // clear timer count
    PR3 = 65535; // no period match
    IEC0bits.T3IE = 0; // disable interrupts
    T3CONbits.TON = 1; // start the timer

    /* Initialize the CRC module.
    */
    CrcInit();

    /* Read the device configuration which includes the string descripts from
    ** flash memory into RAM.
    */
    CfgInit();

    /* Initialize the LED module.
    */
    LedInit();
}

/* ------------------------------------------------------------ */
/***    DeviceTerm
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
**      This routine turns off any on-chip and on-board peripherals
**      that were previously enabled.
*/
void DeviceTerm(void) {

    T3CONbits.TON = 0; // stop timer 3
    TMR3 = 0; // clear timer count
    IFS0bits.T3IF = 0; // clear interrupt

    LedTerm();
    CfgTerm();
    CrcTerm();
}

/* ------------------------------------------------------------ */
/***	AppInit
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
**      This routine performs application specific initialization.
**      It configures devices and global variables for the application.
*/
void AppInit( void ) {

#if !defined(DEAD)
    /*********************** M00TODO: REMOVE THIS CODE ********************/
    latIcspDat &= ~(1 << bnIcspDat);
    trsIcspDat &= ~(1 << bnIcspDat);
    /*********************** M00TODO: REMOVE THIS CODE ********************/
#endif

    /* Map pins for external interrupts.
    */
    __builtin_write_OSCCONL(OSCCON & ~(1<<6));
    RPINR0bits.INT1R = ppsBtnCal;
    RPINR1bits.INT2R = ppsBtnCal;
    __builtin_write_OSCCONL(OSCCON | (1<<6));

    /* Initialize the CAN module.
    */
    CanInit();

    /* Check to see if this was a software initiated reset or of the reset was
    ** the result of a hardware event. If the reset was the result of a hardware
    ** reset then the bootload flags need to be initialized.
    */
    if ( 0 == RCONbits.SWR ) {
        sessidPrev = 0;
        bootflgs.fsFlags = 0;
    }
    else {
        RCONbits.SWR = 0;
    }
}

/* ------------------------------------------------------------ */
/***	AppTerm
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
**      This routine performs application specific de-initialization
**      that may be required prior to handing over execution to the application
**      firmware.
*/
void AppTerm(void) {

    CanTerm();
}

/******************************************************************************/
